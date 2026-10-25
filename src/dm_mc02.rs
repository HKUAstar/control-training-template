//! DM-MC02 hardware setup for the STM32H723VG training firmware.
//! DM-MC02 开发板的 STM32H723VG 硬件初始化。
//!
//! The CPU uses 64 MHz HSI; CAN1 uses the board's 24 MHz HSE crystal on PD0/PD1.
//! CPU 使用 64 MHz HSI 内部时钟；CAN1 使用板载 24 MHz HSE 晶振，引脚为 PD0/PD1。
//! UART5 receives DR16 DBUS through the board inverter on PD2.
//! UART5 通过 PD2 上的板载反相电路接收 DR16 DBUS。
//! TIM2 provides Embassy's time driver, so keep it reserved for timers and the control ticker.
//! TIM2 用作 Embassy 时间驱动，请勿将其用于其他外设功能。

use core::fmt::Write as _;
use core::num::{NonZeroU8, NonZeroU16};
use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_stm32::can::config::NominalBitTiming;
use embassy_stm32::can::{self, BufferedCan, CanConfigurator, RxBuf, TxBuf};
use embassy_stm32::dma;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::rcc::mux::Fdcansel;
use embassy_stm32::rcc::{Hse, HseMode};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{self, Parity, StopBits, UartRx};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{bind_interrupts, pac};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};

use static_cell::StaticCell;

pub type RemoteRx = UartRx<'static, Async>;
pub static REMOTE: Signal<CriticalSectionRawMutex, Option<Dr16>> = Signal::new();

pub type Can1 = BufferedCan<'static, 4, 16>;
static CAN_TX: StaticCell<TxBuf<4>> = StaticCell::new();
static CAN_RX: StaticCell<RxBuf<16>> = StaticCell::new();

pub type Watchdog = IndependentWatchdog<'static, embassy_stm32::peripherals::IWDG1>;

/// Nominal timeout; the internal low-speed oscillator varies with voltage and temperature.
/// 标称超时时间，内部低速振荡器的频率会随电压和温度变化。
const WATCHDOG_TIMEOUT_US: u32 = 100_000;

// ========= CAN interrupts / CAN 中断 =========

// Bind both FDCAN1 interrupt lines to Embassy so its async driver can wake waiting tasks.
// 将 FDCAN1 的两路中断绑定到 Embassy，使异步驱动能够唤醒等待中的任务。
bind_interrupts!(struct Irqs {
    UART5 => usart::InterruptHandler<embassy_stm32::peripherals::UART5>;
    DMA1_STREAM5 => dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH5>;
    FDCAN1_IT0 => can::IT0InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
});

// ========= Board setup / 开发板配置 =========
// Keep hardware setup here. Implement motor and control exercises in main.rs.
// 硬件配置放在此模块，电机和控制练习在 main.rs 中实现。

/// Check reset cause and return buffered CAN1, the started watchdog, and DR16 UART RX.
/// 检查复位原因，返回带缓冲的 CAN1、已启动的看门狗和 DR16 UART 接收驱动。
///
/// Call once at startup: Embassy takes ownership of the MCU peripherals here.
/// 启动时只调用一次：Embassy 在此取得 MCU 外设的所有权。
/// The caller owns the returned driver and passes it to the control task.
/// 调用方持有返回的驱动，并将其交给控制任务。
///
/// Power the board from USB or its normal input; probe power alone cannot power CAN transceivers.
/// 请通过 USB 或正常电源输入为开发板供电，调试探针供电不足以使 CAN 收发器工作。
pub fn init() -> (Can1, Watchdog, RemoteRx) {
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
    // Interrupts buffer frames; the control task reads and writes without waiting.
    // 中断负责缓存帧，控制任务以非阻塞方式读取和发送。
    let can = can
        .into_normal_mode()
        .buffered(CAN_TX.init(TxBuf::new()), CAN_RX.init(RxBuf::new()));
    // ========= DR16 DBUS input / DR16 遥控输入 =========
    // The board inverts DBUS before PD2, so software RX inversion stays disabled.
    // 板载电路已在 PD2 前反相 DBUS，软件无需再次反相。
    let mut remote_config = usart::Config::default();
    remote_config.baudrate = 100_000;
    remote_config.parity = Parity::ParityEven;
    remote_config.stop_bits = StopBits::STOP2;
    let remote = UartRx::new(p.UART5, p.PD2, p.DMA1_CH5, Irqs, remote_config).unwrap();
    // Keep the board's switched 5 V rail enabled for the receiver.
    // 保持板载受控 5 V 电源开启，为接收机供电。
    core::mem::forget(Output::new(p.PC15, Level::High, Speed::Low));
    (can, watchdog, remote)
}

// ========= DR16 receiver / DR16 接收机 =========

/// One valid 18-byte DBUS frame. No axis remapping or stick deadzone is applied.
/// 一帧有效的 18 字节 DBUS 数据，不进行坐标变换或摇杆死区处理。
#[derive(Clone, Copy)]
pub struct Dr16 {
    /// Right horizontal/vertical, left horizontal/vertical, wheel; centered counts, -660..=660.
    /// 右横、右纵、左横、左纵、拨轮，中心为 0，范围为 -660..=660。
    pub channels: [i16; 5],
    /// Wire values: 1 up, 2 down, 3 middle.
    /// 协议值：1 上，2 下，3 中。
    pub right_switch: u8,
    pub left_switch: u8,
    pub mouse: [i16; 3],
    pub mouse_left: bool,
    pub mouse_right: bool,
    /// Bits 0..15: W S A D Shift Ctrl Q E R F G Z X C V B.
    pub keys: u16,
}

impl Dr16 {
    /// Reject partial frames, invalid channels, and invalid switch values.
    /// 拒绝不完整帧、越界通道值和无效开关值。
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        let b: &[u8; 18] = bytes.try_into().ok()?;
        let u = |i: usize| u16::from(b[i]);
        let channels = [
            u(0) | u(1) << 8,
            u(1) >> 3 | u(2) << 5,
            u(2) >> 6 | u(3) << 2 | u(4) << 10,
            u(4) >> 1 | u(5) << 7,
            u(16) | u(17) << 8,
        ]
        .map(|value| (value & 0x7ff) as i16 - 1024);
        let right_switch = b[5] >> 4 & 3;
        let left_switch = b[5] >> 6 & 3;
        if channels.iter().any(|value| !(-660..=660).contains(value))
            || !(1..=3).contains(&right_switch)
            || !(1..=3).contains(&left_switch)
        {
            return None;
        }
        let signed = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
        Some(Self {
            channels,
            right_switch,
            left_switch,
            mouse: [signed(6), signed(8), signed(10)],
            mouse_left: b[12] != 0,
            mouse_right: b[13] != 0,
            keys: u16::from_le_bytes([b[14], b[15]]),
        })
    }
}

/// Wait for UART idle to align frames; publish the latest input without queueing old commands.
/// 通过 UART 空闲间隔对齐数据帧，只发布最新输入，不积压旧遥控指令。
/// See the DJI DT7/DR16 manual for the 18-byte DBUS layout.
/// 18 字节 DBUS 布局见 DJI DT7/DR16 手册。
#[embassy_executor::task]
pub async fn remote_task(mut rx: RemoteRx) {
    // Larger than one frame so a full read does not hide a missing idle boundary.
    // 缓冲区大于一帧，避免刚好填满时掩盖缺失的空闲间隔。
    let mut bytes = [0; 36];
    loop {
        match rx.read_until_idle(&mut bytes).await {
            Ok(n) => REMOTE.signal(Dr16::parse(&bytes[..n])),
            Err(_) => {
                REMOTE.signal(None);
                Timer::after(Duration::from_millis(1)).await;
            }
        }
    }
}

// ========= Panic watchdog reset / Panic 看门狗复位 =========
/// Configure the panic-only watchdog path, including before normal initialization.
/// 配置仅用于 panic 的看门狗路径，也适用于正常初始化之前。
///
/// This terminal path uses PAC registers because the task owns the HAL watchdog token.
/// 此终止路径使用 PAC 寄存器，因为 HAL 看门狗句柄由任务持有。
/// At nominal 32 kHz LSI, reload 799 gives 100 ms and reload 0 gives 125 us,
/// plus register synchronization. Neither duration is an exact wall-clock guarantee.
/// 在 32 kHz 标称 LSI 下，重装值 799 对应 100 ms，0 对应 125 us，另需寄存器同步时间。
/// 两种时长都不是精确的实际时间保证。
/// See RM0468, independent watchdog (IWDG) and RCC reset-status register (RCC_RSR).
/// 参见 RM0468 的独立看门狗和 RCC 复位状态寄存器章节。
fn configure_panic_watchdog(reload: u16) {
    use pac::iwdg::vals::{Key, Pr};

    let wdg = pac::IWDG1;
    wdg.kr().write(|w| w.set_key(Key::START));
    wdg.kr().write(|w| w.set_key(Key::ENABLE));
    while wdg.sr().read().pvu() || wdg.sr().read().rvu() {
        core::hint::spin_loop();
    }
    wdg.pr().write(|w| w.set_pr(Pr::DIVIDE_BY4));
    wdg.rlr().write(|w| w.set_rl(reload));
    while wdg.sr().read().pvu() || wdg.sr().read().rvu() {
        core::hint::spin_loop();
    }
    wdg.kr().write(|w| w.set_key(Key::RESET));
}

// ========= Panic shutdown / Panic 停机 =========
/// Give blocking panic output a watchdog deadline, then request a short watchdog reset.
/// 为阻塞式 panic 输出设置看门狗期限，然后请求短超时看门狗复位。
///
/// Allow nominally 100 ms for RTT writing and flushing, even if no host is connected.
/// 即使未连接主机，也只给 RTT 写入和刷新留出标称 100 ms 时间。
/// Delivery is still best effort; the watchdog expires if the host cannot keep up.
/// 消息仍不保证送达，主机读取不及时会导致看门狗超时。
/// Interrupts stay disabled so no interrupt can feed the watchdog or continue CAN service.
/// 禁用中断，防止中断继续喂狗或执行 CAN 服务。
/// On the next boot, dm_mc02::init halts before CAN and student code until a manual reset.
/// 下次启动时，dm_mc02::init 会在 CAN 和学生代码之前停机，等待手动复位。
#[panic_handler]
fn panic(info: &PanicInfo<'_>) -> ! {
    static PANICKED: AtomicBool = AtomicBool::new(false);

    cortex_m::interrupt::disable();
    if !PANICKED.swap(true, Ordering::Relaxed) {
        // Arm before logging, including for panics before init started the normal watchdog.
        // 打印前先启动看门狗，也覆盖正常看门狗尚未启动时发生的 panic。
        configure_panic_watchdog(799);
        rtt_target::with_terminal_channel(|term| {
            term.set_mode(rtt_target::ChannelMode::BlockIfFull);
            let _ = writeln!(term.write(0), "{}", info);
            // Wait for the probe to read the message; the watchdog bounds this wait.
            // 等待探针读走消息，看门狗限制此处等待时间。
            term.flush();
        });
    }
    configure_panic_watchdog(0);
    loop {
        core::hint::spin_loop();
    }
}
