#![no_std]
#![no_main]
#![forbid(unsafe_code)]

mod dm_mc02;

use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Ticker};

// GM6020 电流模式下 ID 1～4 的组指令。
// 同时限定每周期 CAN 接收数量和遥控输入的超时周期数。
const MOTOR_COMMAND_ID: u16 = 0x1FE;
const MAX_RX_FRAMES_PER_TICK: usize = 8;
const REMOTE_SILENT_TICKS: u16 = 100;

// 保存一个电机的角度、转速和电流反馈，以及速度目标和电流指令。
// 字段使用 rad、rpm、A；与 CAN 原始数值之间的换算由编解码函数完成。
#[derive(Clone, Copy, Default)]
pub struct Motor {
    /// 电机轴单圈角度（rad），尚未累计圈数。
    pub angle_rad: f32,
    pub speed_rpm: f32,
    pub current_a: f32,
    pub target_rpm: f32,
    /// 待编码发送的电流指令（A）。
    pub command_current_a: f32,
}

impl Motor {
    /// 仅清零内存，不发送 CAN 帧。
    // 清零本电机的目标速度和电流指令，只修改内存。
    // 仍需经过发送阶段，电机才能收到零指令。
    pub fn disable(&mut self) {
        self.target_rpm = 0.0;
        self.command_current_a = 0.0;
    }

    pub fn read(&mut self, _data: &[u8; 8]) {
        // 待补全：按手册解码角度、转速和电流。
        todo!("Decode motor feedback");
    }

    /// 打包 ID 1～4 的指令；None 槽位填零。
    pub fn transmit(_motors: &[Option<Motor>; 4]) -> [u8; 8] {
        // 待补全：限幅、换算并打包电流指令，None 槽位填零。
        todo!("Encode the four motor commands");
    }
}

// PID 控制器保存参数及跨周期使用的积分和历史数据。
// 待补全：增益、历史状态、积分限制和输出限制。
pub struct Pid {}

impl Pid {
    // 待补全：清除 PID 历史状态。
    pub fn reset(&mut self) {
        todo!("Clear PID history");
    }

    /// 单位由调用处约定：速度环 rpm → A，角度环 rad → rpm；dt_s 为秒。
    pub fn calculate(&mut self, _target: f32, _measured: f32, _dt_s: f32) -> f32 {
        // 待补全：使用 dt_s 计算 PID，并限制积分和输出。
        todo!("Implement PID");
    }
}

// 记录云台当前所处阶段，决定允许的操作和控制输出。
// 本次分别对应等待、回正、遥控调试和故障恢复。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Waiting,
    Prepare,
    Running,
    Error,
}

// 汇总状态、电机和遥控输入，供每个控制周期共同使用。
// 待补全：两轴独立的双环 PID、目标角度、反馈年龄和故障记录。
pub struct Control {
    pub state: State,
    /// 槽位依次对应电机 ID 1～4。
    pub motors: [Option<Motor>; 4],
    pub pid: Pid,
    pub remote: Option<dm_mc02::Dr16>,
    remote_silent_ticks: u16,
}

// 设置上电初值：进入 Waiting，电机槽位和遥控输入尚未就绪。
// 添加成员后，也需要在这里设置对应的初始值。
impl Default for Control {
    fn default() -> Self {
        Self {
            state: State::Waiting,
            motors: [None; 4],
            pid: Pid {},
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
        }
    }
}

impl Control {
    // 对所有 Some 电机调用 disable，未配置的槽位直接跳过。
    // 本函数只清除软件指令，不调用 PID，也不直接发送 CAN 帧。
    pub fn disable_all(&mut self) {
        for motor in self.motors.iter_mut().flatten() {
            motor.disable();
        }
    }

    // 每 1 ms 执行一次：读取遥控器和电机反馈，再判断状态、计算并发送输出。
    // 遥控输入无效时强制回到 Waiting，避免继续执行运行状态的控制。
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

    // 取走接收任务发布的最新遥控输入，没有新帧时继续累计反馈年龄。
    // 输入无效或超过超时周期后，清除电机指令并返回 Waiting。
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

    // 从 CAN 接收队列取出已有帧，每周期最多处理指定数量，不等待新帧。
    // 需要将匹配帧交给对应电机解码，并记录各电机最近收到反馈的时间。
    pub fn read_feedback(&mut self, can: &mut Can1) {
        let receiver = can.reader();
        for _ in 0..MAX_RX_FRAMES_PER_TICK {
            let Ok(Ok(envelope)) = receiver.try_receive() else {
                break;
            };
            // 待补全：筛选 ID 和长度、路由并解码，记录反馈年龄。
            let _frame = envelope.frame;
        }
    }

    // 根据遥控操作、反馈有效性和故障条件决定当前状态。
    // 待补全：切换判断，以及进入、退出状态时的目标和 PID 初始化。
    pub fn update_mode(&mut self) {
        match self.state {
            State::Waiting => {
                // 待补全：有效反馈和新的使能操作满足后进入回正。
            }
            State::Prepare => {
                // 待补全：初始化目标与 PID，稳定回正后进入调试。
            }
            State::Running => {
                // 待补全：判断停用、反馈超时和控制故障。
            }
            State::Error => {
                // 待补全：保持零输出，自行设计人工恢复条件。
            }
        }
    }

    // 根据当前状态设置目标，调用各轴 PID，并写入电流指令。
    // 待补全：受限回正、遥控双环和禁止输出时的行为；PID 周期为 0.001 s。
    pub fn update(&mut self) {}

    // 将四个槽位的电流指令打包为组控制帧，通过 CAN1 非阻塞发送。
    // 未配置任何电机时不发送；发送队列满会触发 panic 和复位锁定。
    pub fn transmit(&mut self, can: &mut Can1) {
        if self.motors.iter().all(Option::is_none) {
            return;
        }
        // 仅允许回正或调试状态输出，且遥控输入必须有效。
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

// 固件入口：初始化 RTT 日志、开发板和通信接口，再启动两个异步任务。
// 遥控任务接收输入，控制任务持有 CAN 和看门狗并持续运行。
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!("change me");
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}

// 用 Ticker 安排 1 kHz 控制周期，调用 tick 完成一次控制后再喂狗。
// 每次 tick 应在 1 ms 内完成；卡住而不能喂狗时，由看门狗触发复位。
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
