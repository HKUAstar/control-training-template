#![no_std]
#![no_main]
#![forbid(unsafe_code)]

mod dm_mc02;

use embassy_executor::Spawner;
use embassy_stm32::can::Can;
use embassy_time::{Duration, Ticker};

// ========= Motor feedback and commands / 电机反馈与指令 =========
// Implement the wire protocol here. Keep physical values in the units shown below.
// 在此实现通信协议，物理量使用下方字段标明的单位。

#[derive(Clone, Copy, Default)]
pub struct Motor {
    /// Shaft angle in radians, before any multi-turn tracking.
    pub angle_rad: f32,
    pub speed_rpm: f32,
    pub current_a: f32,
    /// Desired shaft speed. Use this as the first control exercise.
    pub target_rpm: f32,
    /// Controller output to encode in the group command.
    pub command_current_a: f32,
}

impl Motor {
    /// Clear the software target and current command. This does not send a CAN frame.
    /// 清除软件目标值和电流指令，此函数不会发送 CAN 帧。
    pub fn disable(&mut self) {
        self.target_rpm = 0.0;
        self.command_current_a = 0.0;
    }

    /// Decode one motor's eight-byte feedback payload, including scaling and sign.
    pub fn read(&mut self, _data: &[u8; 8]) {
        // Decode byte order, signed speed/current, and angle/current scaling from the manual.
        // 根据手册解析字节序、转速和电流的符号，并换算角度和电流的单位。
        todo!("Decode motor feedback");
    }

    /// Encode one command for motor IDs 1 through 4. None means zero command for that slot.
    /// Choose the CAN group ID when sending this payload from the control task.
    pub fn transmit(_motors: &[Option<Motor>; 4]) -> [u8; 8] {
        // Clamp current, convert amperes to protocol counts, and pack four commands.
        // 限制电流，将安培换算为协议数值，再打包四个电机的指令。
        // Encode zero for every None slot.
        // 每个 None 槽位都应编码为零指令。
        todo!("Encode the four motor commands");
    }
}

// ========= PID / PID 控制器 =========
// Add gains, error history, integral state, and output limits.
// 添加增益、历史误差、积分状态和输出限幅。

pub struct Pid {}

impl Pid {
    pub fn reset(&mut self) {
        // Clear integral and previous-error state before enabling control.
        // 启用控制前清除积分和历史误差状态。
        todo!("Clear PID history");
    }

    /// For speed control, inputs are rpm and output is amperes. dt_s is the tick period.
    pub fn calculate(&mut self, _target_rpm: f32, _measured_rpm: f32, _dt_s: f32) -> f32 {
        // Calculate P, I, and D using dt_s; handle integral windup and clamp the output.
        // 使用 dt_s 计算比例、积分和微分项，处理积分饱和并限制输出。
        todo!("Implement PID");
    }
}

// ========= Modes and control state / 模式与控制状态 =========
// Start in Waiting. Add feedback-age tracking and one PID per controlled motor as needed.
// 从 Waiting 开始，按需添加反馈超时计数和各电机的 PID。

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Wait for hardware feedback before enabling anything.
    Waiting,
    /// Reset controllers and prepare to enable.
    Prepare,
    /// Run normal control.
    Running,
    /// Optional debugging state.
    Error,
}

pub struct Control {
    pub state: State,
    /// Array slots correspond to motor IDs 1 through 4.
    pub motors: [Option<Motor>; 4],
    pub pid: Pid,
}

impl Default for Control {
    fn default() -> Self {
        Self {
            state: State::Waiting,
            motors: [None; 4],
            pid: Pid {},
        }
    }
}

impl Control {
    // ========= Clear commands / 清除指令 =========
    /// Clear every configured motor's software command without calling unfinished PID code.
    /// 清除所有已配置电机的软件指令，不调用尚未实现的 PID 代码。
    /// The transmit stage must still send the zero commands to the motor controllers.
    /// 仍需由发送阶段将零指令发送到电机控制器。
    pub fn disable_all(&mut self) {
        for motor in self.motors.iter_mut().flatten() {
            motor.disable();
        }
    }

    // ========= Tick order / 每周期执行顺序 =========
    // Run every 1 ms. Check feedback and mode before calculating or sending commands.
    // 每 1 ms 执行一次，先检查反馈和模式，再计算和发送指令。
    pub fn tick(&mut self, can: &mut Can<'static>) {
        self.read_feedback(can);
        self.update_mode();
        self.update();
        if self.state != State::Running {
            self.disable_all();
        }
        self.transmit(can);
    }

    // ========= Read feedback / 读取反馈 =========
    // Read available frames, route CAN IDs to motors, and call Motor::read.
    // 读取已有帧，根据 CAN ID 找到对应电机，再调用 Motor::read。
    // Track feedback age in ticks. Bound receive work so missing frames cannot block a tick.
    // 用周期数记录反馈年龄，限制每周期接收工作量，避免等待缺失帧阻塞控制。
    pub fn read_feedback(&mut self, _can: &mut Can<'static>) {}

    // ========= Update mode / 更新模式 =========
    // Define transition conditions. Missing feedback must return to Waiting before control.
    // 定义状态切换条件，反馈超时必须在控制计算前返回 Waiting。
    pub fn update_mode(&mut self) {
        match self.state {
            State::Waiting => {
                // Require fresh feedback and an explicit enable request before entering Prepare.
                // 进入 Prepare 前，必须收到所有必需电机的新反馈以及明确的使能请求。
                // Do not automatically restart motors after a panic reset.
                // panic 复位后不可自动重启电机。
            }
            State::Prepare => {
                // Reset PID history and targets once, then enter Running when ready.
                // 重置 PID 历史状态和目标值，准备完成后进入 Running。
            }
            State::Running => {
                // Stay here while hardware is ready; return to Waiting on feedback timeout.
                // 硬件就绪时保持运行，反馈超时则返回 Waiting。
            }
            State::Error => {
                // Optional debugging state. Define an explicit recovery condition.
                // 可选的调试状态，请定义明确的恢复条件。
            }
        }
    }

    // ========= Update control / 更新控制量 =========
    // In Running, call PID with dt_s = 0.001 and write each motor's command_current_a.
    // 在 Running 中，以 dt_s = 0.001 调用 PID，更新各电机的 command_current_a。
    // Set all commands to zero in other modes and for motors with missing feedback.
    // 其他模式下以及电机反馈缺失时，必须将相应指令清零。
    pub fn update(&mut self) {}

    // ========= Transmit commands / 发送指令 =========
    // Call Motor::transmit, construct a frame with the group CAN ID, and send it on CAN1.
    // 调用 Motor::transmit，使用组指令 CAN ID 构造帧，然后通过 CAN1 发送。
    // Send zero commands outside Running. Do not wait indefinitely for a free transmit slot.
    // 非 Running 模式发送零指令，不要无限等待空闲的发送槽位。
    pub fn transmit(&mut self, _can: &mut Can<'static>) {}
}

// ========= Startup and RTT / 启动与 RTT =========
// Initialize the board, change the example message, and start the control task.
// 初始化开发板，修改示例消息，并启动控制任务。

/// Firmware entry point, run by Embassy's executor after reset.
/// 固件入口，由 Embassy 执行器在复位后运行。
///
/// Set up RTT before board initialization, then print the example message after setup succeeds.
/// 先配置 RTT，再初始化开发板；初始化成功后打印示例消息。
/// Move CAN1 into the control task. The executor keeps that task running after main returns.
/// 将 CAN1 移交给控制任务；main 返回后，执行器仍会继续调度该任务。
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog) = dm_mc02::init();
    rtt_target::rprintln!("change me");
    spawner.spawn(control_task(can, watchdog).unwrap());
}

// ========= Control task / 控制任务 =========
// Keep the 1 kHz schedule here; implement each tick stage in Control above.
// 在此保持 1 kHz 调度，在上方 Control 中实现每个周期阶段。

/// Own CAN1, run the control stages every 1 ms, and feed the watchdog after each completed tick.
/// 持有 CAN1 驱动，每 1 ms 执行控制阶段，并在每个周期完成后喂狗。
///
/// Ticker schedules periodic deadlines. Keep each tick below 1 ms to avoid missing them.
/// Ticker 按固定周期安排截止时间，每次 tick 应在 1 ms 内完成，避免错过截止时间。
#[embassy_executor::task]
async fn control_task(mut can: Can<'static>, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(Duration::from_millis(1));
    loop {
        ticker.next().await;
        control.tick(&mut can);
        // Feed the watchdog to prevent a reset while the control loop is healthy.
        watchdog.pet();
    }
}
