//! DM-MC02 hardware setup for the STM32H723VG training firmware.
//! DM-MC02 开发板的 STM32H723VG 硬件初始化。
//!
//! The CPU uses 64 MHz HSI; CAN1 uses the board's 24 MHz HSE crystal on PD0/PD1.
//! CPU 使用 64 MHz HSI 内部时钟；CAN1 使用板载 24 MHz HSE 晶振，引脚为 PD0/PD1。
//! TIM2 provides Embassy's time driver, so keep it reserved for timers and the control ticker.
//! TIM2 用作 Embassy 时间驱动，请勿将其用于其他外设功能。

use core::num::{NonZeroU8, NonZeroU16};
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_stm32::can::config::NominalBitTiming;
use embassy_stm32::can::{self, Can, CanConfigurator};
use embassy_stm32::rcc::mux::Fdcansel;
use embassy_stm32::rcc::{Hse, HseMode};
use embassy_stm32::time::Hertz;
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{bind_interrupts, pac};

pub type Watchdog = IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG1>;

/// Nominal timeout; the internal low-speed oscillator varies with voltage and temperature.
/// 标称超时时间，内部低速振荡器的频率会随电压和温度变化。
const WATCHDOG_TIMEOUT_US: u32 = 100_000;

// ========= CAN interrupts / CAN 中断 =========

// Bind both FDCAN1 interrupt lines to Embassy so its async driver can wake waiting tasks.
// 将 FDCAN1 的两路中断绑定到 Embassy，使异步驱动能够唤醒等待中的任务。
bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
});

// ========= Board setup / 开发板配置 =========
// Keep hardware setup here. Implement motor and control exercises in main.rs.
// 硬件配置放在此模块，电机和控制练习在 main.rs 中实现。

/// Check the reset cause, start the watchdog, and initialize classic CAN1 at 1 Mbit/s.
/// 检查复位原因，启动看门狗，并以 1 Mbit/s 初始化经典 CAN1。
///
/// Call once at startup: Embassy takes ownership of the MCU peripherals here.
/// 启动时只调用一次：Embassy 在此取得 MCU 外设的所有权。
/// The caller owns the returned driver and passes it to the control task.
/// 调用方持有返回的驱动，并将其交给控制任务。
///
/// Power the board from USB or its normal input; probe power alone cannot power CAN transceivers.
/// 请通过 USB 或正常电源输入为开发板供电，调试探针供电不足以使 CAN 收发器工作。
pub fn init() -> (Can<'static>, Watchdog) {
    // ========= Watchdog reset lockout / 看门狗复位后锁定 =========
    // RM0468, RCC_RSR: flags persist until RMVF is set. Clear before halting so a later
    // manual reset can resume. Never reach CAN setup or the student task on this boot.
    // RCC_RSR 标志由 RMVF 清除。停机前清除标志，便于下次手动复位恢复。
    // 本次启动不进入 CAN 初始化或学生任务。
    let watchdog_reset = pac::RCC.rsr().read().iwdg1rstf();
    pac::RCC.rsr().modify(|w| w.set_rmvf(true));
    if watchdog_reset {
        rtt_target::rprintln!("Watchdog reset: halted. Press RESET to run again.");
        cortex_m::interrupt::disable();
        loop {
            core::hint::spin_loop();
        }
    }
    // ========= Clocks / 时钟 =========
    // CPU 使用默认的 64 MHz 内部时钟，CAN 使用板载 24 MHz 晶振，不使用 PLL。
    // CPU stays on the default 64 MHz HSI. CAN uses the board's 24 MHz crystal, without a PLL.
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(24_000_000),
        mode: HseMode::Oscillator,
    });
    // Select HSE as the CAN kernel clock; the CPU clock source stays unchanged.
    // 选择 HSE 作为 CAN 内核时钟，CPU 时钟源保持不变。
    config.rcc.mux.fdcansel = Fdcansel::HSE;
    let p = embassy_stm32::init(config);

    // ========= Control watchdog / 控制看门狗 =========
    // Only the control task may feed this, after a completed tick.
    // 只允许控制任务在完成一个周期后喂狗。
    let mut watchdog = Watchdog::new(p.IWDG1, WATCHDOG_TIMEOUT_US);
    watchdog.unleash();

    // ========= CAN1 at 1 Mbit/s / CAN1 波特率 1 Mbit/s =========
    // PD0 is RX and PD1 is TX. Keep classic CAN for this exercise.
    // PD0 为接收，PD1 为发送，本练习使用经典 CAN。
    let mut can = CanConfigurator::new(p.FDCAN1, p.PD0, p.PD1, Irqs);
    // 24 MHz / (1 * (1 + 17 + 6)) = 1 Mbit/s, with a 75% sample point.
    // 每位包含同步段 1、第一段 17、第二段 6 个时间量子，采样点位于 75%。
    can.set_config(can.config().set_nominal_bit_timing(NominalBitTiming {
        prescaler: NonZeroU16::new(1).unwrap(),
        seg1: NonZeroU8::new(17).unwrap(),
        seg2: NonZeroU8::new(6).unwrap(),
        sync_jump_width: NonZeroU8::new(6).unwrap(),
    }));
    // Normal mode connects to the physical bus; this is not an internal loopback test.
    // 正常模式连接实际总线，此处不使用内部回环测试。
    (can.into_normal_mode(), watchdog)
}

// ========= Panic watchdog reset / Panic 看门狗复位 =========
/// Force a watchdog reset even if panic occurred before normal initialization.
/// 即使 panic 发生在正常初始化之前，也通过看门狗触发复位。
///
/// This terminal path uses PAC registers because the task owns the HAL watchdog token.
/// 此终止路径使用 PAC 寄存器，因为 HAL 看门狗句柄由任务持有。
/// With prescaler 4 and reload 0, expiry takes roughly 125 us at nominal 32 kHz LSI,
/// plus register synchronization. This is not a synchronous reset instruction.
/// 分频为 4、重装值为 0 时，32 kHz 标称 LSI 下约 125 us 超时，另需寄存器同步时间。
/// 它不是同步复位指令。
/// See RM0468, independent watchdog (IWDG) and RCC reset-status register (RCC_RSR).
/// 参见 RM0468 的独立看门狗和 RCC 复位状态寄存器章节。
fn panic_reset() -> ! {
    use pac::iwdg::vals::{Key, Pr};

    let wdg = pac::IWDG1;
    wdg.kr().write(|w| w.set_key(Key::START));
    wdg.kr().write(|w| w.set_key(Key::ENABLE));
    while wdg.sr().read().pvu() || wdg.sr().read().rvu() {
        core::hint::spin_loop();
    }
    wdg.pr().write(|w| w.set_pr(Pr::DIVIDE_BY4));
    wdg.rlr().write(|w| w.set_rl(0));
    while wdg.sr().read().pvu() || wdg.sr().read().rvu() {
        core::hint::spin_loop();
    }
    wdg.kr().write(|w| w.set_key(Key::RESET));
    loop {
        core::hint::spin_loop();
    }
}

// ========= Panic shutdown / Panic 停机 =========
/// Print once without waiting for the host, then force an independent-watchdog reset.
/// 不等待主机，尝试打印一次消息，然后触发独立看门狗复位。
///
/// RTT delivery is best effort: reset can occur before the host reads the message.
/// RTT 消息不保证送达：主机读取之前可能已经复位。
/// Interrupts stay disabled so no interrupt can feed the watchdog or continue CAN service.
/// 禁用中断，防止中断继续喂狗或执行 CAN 服务。
/// On the next boot, dm_mc02::init halts before CAN and student code until a manual reset.
/// 下次启动时，dm_mc02::init 会在 CAN 和学生代码之前停机，等待手动复位。
#[panic_handler]
fn panic(info: &PanicInfo<'_>) -> ! {
    static PANICKED: AtomicBool = AtomicBool::new(false);

    cortex_m::interrupt::disable();
    if !PANICKED.swap(true, Ordering::Relaxed) {
        rtt_target::rprintln!("{}", info);
    }
    panic_reset()
}
