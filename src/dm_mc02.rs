//! DM-MC02 hardware setup for the STM32H723VG training firmware.
//!
//! The CPU uses 64 MHz HSI; CAN1 uses the board's 24 MHz HSE crystal on PD0/PD1.
//! UART5 receives DR16 DBUS through the board inverter on PD2.
//! TIM2 provides Embassy's time driver, so keep it reserved for timers and the control ticker.

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
const WATCHDOG_TIMEOUT_US: u32 = 100_000;

// ========= CAN interrupts =========

// Bind both FDCAN1 interrupt lines to Embassy so its async driver can wake waiting tasks.
bind_interrupts!(struct Irqs {
    UART5 => usart::InterruptHandler<embassy_stm32::peripherals::UART5>;
    DMA1_STREAM5 => dma::InterruptHandler<embassy_stm32::peripherals::DMA1_CH5>;
    FDCAN1_IT0 => can::IT0InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<embassy_stm32::peripherals::FDCAN1>;
});

// ========= Board setup =========
// Keep hardware setup here. Implement motor and control exercises in main.rs.

/// Check reset cause and return buffered CAN1, the started watchdog, and DR16 UART RX.
///
/// Call once at startup: Embassy takes ownership of the MCU peripherals here.
/// The caller owns the returned driver and passes it to the control task.
///
/// Power the board from USB or its normal input; probe power alone cannot power CAN transceivers.
pub fn init() -> (Can1, Watchdog, RemoteRx) {
    // ========= Watchdog reset lockout =========
    // RM0468, RCC_RSR: flags persist until RMVF is set. Clear before halting so a later
    // manual reset can resume. Never reach CAN setup or the student task on this boot.
    let watchdog_reset = pac::RCC.rsr().read().iwdg1rstf();
    pac::RCC.rsr().modify(|w| w.set_rmvf(true));
    if watchdog_reset {
        rtt_target::rprintln!("Watchdog reset: halted. Press RESET to run again.");
        cortex_m::interrupt::disable();
        loop {
            core::hint::spin_loop();
        }
    }
    // ========= Clocks =========
    // CPU stays on the default 64 MHz HSI. CAN uses the board's 24 MHz crystal, without a PLL.
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz(24_000_000),
        mode: HseMode::Oscillator,
    });
    // Select HSE as the CAN kernel clock; the CPU clock source stays unchanged.
    config.rcc.mux.fdcansel = Fdcansel::HSE;
    let p = embassy_stm32::init(config);

    // ========= Control watchdog =========
    // Only the control task may feed this, after a completed tick.
    let mut watchdog = Watchdog::new(p.IWDG1, WATCHDOG_TIMEOUT_US);
    watchdog.unleash();

    // ========= CAN1 at 1 Mbit/s =========
    // PD0 is RX and PD1 is TX. Keep classic CAN for this exercise.
    let mut can = CanConfigurator::new(p.FDCAN1, p.PD0, p.PD1, Irqs);
    // 24 MHz / (1 * (1 + 17 + 6)) = 1 Mbit/s, with a 75% sample point.
    can.set_config(can.config().set_nominal_bit_timing(NominalBitTiming {
        prescaler: NonZeroU16::new(1).unwrap(),
        seg1: NonZeroU8::new(17).unwrap(),
        seg2: NonZeroU8::new(6).unwrap(),
        sync_jump_width: NonZeroU8::new(6).unwrap(),
    }));
    // Normal mode connects to the physical bus; this is not an internal loopback test.
    // Interrupts buffer frames; the control task reads and writes without waiting.
    let can = can
        .into_normal_mode()
        .buffered(CAN_TX.init(TxBuf::new()), CAN_RX.init(RxBuf::new()));
    // ========= DR16 DBUS input =========
    // The board inverts DBUS before PD2, so software RX inversion stays disabled.
    let mut remote_config = usart::Config::default();
    remote_config.baudrate = 100_000;
    remote_config.parity = Parity::ParityEven;
    remote_config.stop_bits = StopBits::STOP2;
    let remote = UartRx::new(p.UART5, p.PD2, p.DMA1_CH5, Irqs, remote_config).unwrap();
    // Keep the board's switched 5 V rail enabled for the receiver.
    core::mem::forget(Output::new(p.PC15, Level::High, Speed::Low));
    (can, watchdog, remote)
}

// ========= DR16 receiver =========

/// One valid 18-byte DBUS frame. No axis remapping or stick deadzone is applied.
#[derive(Clone, Copy)]
pub struct Dr16 {
    /// Right horizontal/vertical, left horizontal/vertical, wheel; centered counts, -660..=660.
    pub channels: [i16; 5],
    /// Wire values: 1 up, 2 down, 3 middle.
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
/// See the DJI DT7/DR16 manual for the 18-byte DBUS layout.
#[embassy_executor::task]
pub async fn remote_task(mut rx: RemoteRx) {
    // Larger than one frame so a full read does not hide a missing idle boundary.
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

// ========= Panic watchdog reset =========
/// Configure the panic-only watchdog path, including before normal initialization.
///
/// This terminal path uses PAC registers because the task owns the HAL watchdog token.
/// At nominal 32 kHz LSI, reload 799 gives 100 ms and reload 0 gives 125 us,
/// plus register synchronization. Neither duration is an exact wall-clock guarantee.
/// See RM0468, independent watchdog (IWDG) and RCC reset-status register (RCC_RSR).
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

// ========= Panic shutdown =========
/// Give blocking panic output a watchdog deadline, then request a short watchdog reset.
///
/// Allow nominally 100 ms for RTT writing and flushing, even if no host is connected.
/// Delivery is still best effort; the watchdog expires if the host cannot keep up.
/// Interrupts stay disabled so no interrupt can feed the watchdog or continue CAN service.
/// On the next boot, dm_mc02::init halts before CAN and student code until a manual reset.
#[panic_handler]
fn panic(info: &PanicInfo<'_>) -> ! {
    static PANICKED: AtomicBool = AtomicBool::new(false);

    cortex_m::interrupt::disable();
    if !PANICKED.swap(true, Ordering::Relaxed) {
        // Arm before logging, including for panics before init started the normal watchdog.
        configure_panic_watchdog(799);
        rtt_target::with_terminal_channel(|term| {
            term.set_mode(rtt_target::ChannelMode::BlockIfFull);
            let _ = writeln!(term.write(0), "{}", info);
            // Wait for the probe to read the message; the watchdog bounds this wait.
            term.flush();
        });
    }
    configure_panic_watchdog(0);
    loop {
        core::hint::spin_loop();
    }
}
