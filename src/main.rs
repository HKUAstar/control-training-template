#![no_std]
#![no_main]
#![forbid(unsafe_code)]

mod dm_mc02;

use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Ticker};

// GM6020 current-mode group command for motor IDs 1 to 4.
const MOTOR_COMMAND_ID: u16 = 0x1FE;
// CAN frames read per tick, and ticks without remote input before it counts as lost.
const MAX_RX_FRAMES_PER_TICK: usize = 8;
const REMOTE_SILENT_TICKS: u16 = 100;

/// Feedback and current command for one motor.
/// `Motor::read` and `Motor::transmit` convert between these fields and CAN values.
#[derive(Clone, Copy, Default)]
pub struct Motor {
    /// Single-turn motor shaft angle in rad; turns are not counted.
    pub angle_rad: f32,
    pub speed_rpm: f32,
    /// Feedback current as sent on the wire. The motor manual gives no scale to amperes.
    pub current_raw: i16,
    /// Current command in A, encoded by `Motor::transmit`.
    pub command_current_a: f32,
}

impl Motor {
    /// Clear the current command in RAM. This sends no CAN frame;
    /// the motor receives zero only when `Control::transmit` next runs.
    pub fn disable(&mut self) {
        self.command_current_a = 0.0;
    }

    pub fn read(&mut self, _data: &[u8; 8]) {
        // TODO: decode angle, speed and current as the manual describes.
        todo!("Decode motor feedback");
    }

    /// Pack the commands for motor IDs 1 to 4. `None` slots encode zero.
    pub fn transmit(_motors: &[Option<Motor>; 4]) -> [u8; 8] {
        // TODO: limit, convert and pack each current command.
        todo!("Encode the four motor commands");
    }
}

/// PID gains, plus the integral and history kept between ticks.
// TODO: add gains, history, an integral limit and a symmetric output limit `maxout`.
pub struct Pid {}

impl Pid {
    // TODO: clear the integral and history.
    pub fn reset(&mut self) {
        todo!("Clear PID history");
    }

    /// `target` and `measured` share a unit, set by the caller. The position loop takes rad
    /// and returns rpm; the speed loop takes rpm and returns A. `dt_s` is in seconds.
    /// If a position loop returns rad/s, convert it to rpm before passing it to the speed loop.
    pub fn calculate(&mut self, _target: f32, _measured: f32, _dt_s: f32) -> f32 {
        // TODO: compute the PID with dt_s, and limit the integral and output.
        todo!("Implement PID");
    }
}

/// The gimbal's stage, which decides the allowed operations and outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Zero output. Waits for valid remote input and fresh feedback from both motors,
    /// then for a new enable request.
    Waiting,
    /// Recenters both axes to the reference pose with limited speed and output.
    /// Enters `Running` once both axes settle.
    Prepare,
    /// The left stick moves each axis's `target_angle_rad`, and its cascade tracks it.
    Running,
    /// Zero output until a manual recovery.
    Error,
}

/// One gimbal axis: its motor slot, targets and cascaded PIDs.
pub struct Axis {
    /// Index into `Control::motors`, which is the motor ID minus 1.
    pub slot: usize,
    /// Position loop target in rad.
    pub target_angle_rad: f32,
    /// Speed loop target in rpm, set from the position loop output.
    pub target_rpm: f32,
    /// Position loop: rad in, rpm out.
    pub position_pid: Pid,
    /// Speed loop: rpm in, A out.
    pub speed_pid: Pid,
}

impl Axis {
    pub const fn new(slot: usize) -> Self {
        Self {
            slot,
            target_angle_rad: 0.0,
            target_rpm: 0.0,
            position_pid: Pid {},
            speed_pid: Pid {},
        }
    }
}

/// State shared by every control tick.
// TODO: add feedback ages and a fault record.
pub struct Control {
    pub state: State,
    /// Slots hold motor IDs 1 to 4.
    pub motors: [Option<Motor>; 4],
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<dm_mc02::Dr16>,
    remote_silent_ticks: u16,
}

/// Start in `Waiting` with no motors and no remote input.
/// Set the initial value of each field you add here.
impl Default for Control {
    fn default() -> Self {
        Self {
            state: State::Waiting,
            motors: [None; 4],
            // TODO: use the motor IDs from the rig configuration.
            yaw: Axis::new(0),
            pitch: Axis::new(1),
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
        }
    }
}

impl Control {
    /// Disable every configured motor. This only clears commands in RAM.
    pub fn disable_all(&mut self) {
        for motor in self.motors.iter_mut().flatten() {
            motor.disable();
        }
    }

    /// Runs every 1 ms: read the remote and motor feedback, update the state, then
    /// compute and send the outputs. Missing remote input forces `Waiting`.
    pub fn tick(&mut self, can: &mut Can1) {
        self.read_remote();
        self.read_feedback(can);
        self.update_mode();
        if self.remote.is_none() {
            self.state = State::Waiting;
        }
        self.update();
        self.transmit(can);
    }

    /// Take the latest input published by the remote task. After an invalid frame, or
    /// `REMOTE_SILENT_TICKS` ticks without one, clear the motor commands and return to `Waiting`.
    pub fn read_remote(&mut self) {
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1);
        if let Some(remote) = dm_mc02::REMOTE.try_take() {
            self.remote = remote;
            if self.remote.is_some() {
                self.remote_silent_ticks = 0;
            }
        }
        if self.remote_silent_ticks >= REMOTE_SILENT_TICKS {
            self.remote = None;
        }
        if self.remote.is_none() {
            self.disable_all();
            self.state = State::Waiting;
        }
    }

    /// Read up to `MAX_RX_FRAMES_PER_TICK` queued CAN frames without waiting for more.
    pub fn read_feedback(&mut self, can: &mut Can1) {
        let receiver = can.reader();
        for _ in 0..MAX_RX_FRAMES_PER_TICK {
            let Ok(Ok(envelope)) = receiver.try_receive() else {
                break;
            };
            // TODO: check the ID and length, pass the data to the matching motor's
            // `Motor::read`, and record its feedback age.
            let _frame = envelope.frame;
        }
    }

    /// Choose the state from the remote, feedback and fault conditions.
    // TODO: the transitions, and initialising targets and PIDs on entering a state.
    pub fn update_mode(&mut self) {
        match self.state {
            State::Waiting => {
                // TODO: enter Prepare after valid feedback and a new enable request.
            }
            State::Prepare => {
                // TODO: enter Running once recentering settles; stop on a timeout or fault.
            }
            State::Running => {
                // TODO: handle disable, feedback timeouts and control faults.
            }
            State::Error => {
                // TODO: keep zero output until your manual recovery condition.
            }
        }
    }

    /// Set targets for the current state, run each axis's PIDs and write the current commands.
    // TODO: limited recentering, remote cascade control, and zero output elsewhere.
    // Each tick is 0.001 s.
    pub fn update(&mut self) {}

    /// Encode the four current commands into one group frame and queue it on CAN1
    /// without waiting. Sends nothing while no motor is configured. A full transmit
    /// queue panics, which leads to the watchdog reset lockout.
    pub fn transmit(&mut self, can: &mut Can1) {
        if self.motors.iter().all(Option::is_none) {
            return;
        }
        // Only Prepare and Running may drive the motors, and only with valid remote input.
        if self.remote.is_none() || !matches!(self.state, State::Prepare | State::Running) {
            self.disable_all();
        }
        let data = Motor::transmit(&self.motors);
        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        can.writer()
            .try_write(frame)
            .expect("CAN1 transmit queue full");
    }
}

// Firmware entry: set up RTT, the board and its interfaces, then start both tasks.
// The remote task receives input; the control task owns CAN and the watchdog.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!("change me");
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}

// Run `tick` at 1 kHz and feed the watchdog after each one. A tick must finish
// within 1 ms; if it hangs, the watchdog resets the MCU.
#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(Duration::from_millis(1));
    loop {
        ticker.next().await;
        control.tick(&mut can);
        watchdog.pet();
    }
}
