# Control training template

An Embassy starter for the DM-MC02 with an STM32H723VG. Students implement a GM6020-style
motor protocol, PID, and state machine. The starter stays in `Waiting` and sends no motor commands.

There are two source files:

- `src/dm_mc02.rs`: clocks and CAN1 setup.
- `src/main.rs`: entry point, RTT, 1 kHz control task, and exercises.

## Build and run

Install Rust through rustup and install `probe-rs` with `cargo install probe-rs-tools --locked`.
The pinned toolchain installs the Cortex-M7 target.

```sh
cargo build --release --locked
cargo run --release --locked
```

`cargo run` flashes the board through a connected debug probe and displays RTT text. The startup
message is `change me`. Stop the host session with Ctrl-C.

CAN1 uses PD0 for RX and PD1 for TX, with classic CAN at 1 Mbit/s. The CPU uses the default
64 MHz HSI. CAN uses the board's 24 MHz crystal with a 75% sample point. No PLL is configured.
Power the board through USB or its normal input; probe power alone does not power its CAN
transceivers. Connect CAN ground and terminate both ends of the bus with 120 ohms.

## Exercises

1. Implement `Motor::read` and `Motor::transmit`. Use the GM6020 manual to choose CAN IDs, byte
   order, signed values, scaling, and limits. The transmit array has four slots for motor IDs
   1 through 4. Encode zero for `None`. The function returns eight payload bytes; choose the
   group CAN ID when constructing the frame in the task.
2. Add PID fields and implement `reset` and `calculate`. The initial exercise controls speed in
   rpm and produces a current command in amperes. Pass `0.001` for the 1 ms tick period.
3. Fill in `Control::read_feedback` and `Control::transmit`. Route feedback CAN IDs to motors
   and send the group command on CAN1. Keep reads and writes bounded so a missing frame or full
   transmit queue cannot stall the control tick. Embassy's buffered CAN API is one option.
4. Implement `Control::update_mode`: `Waiting` waits for hardware, `Prepare` resets controllers,
   and `Running` enables normal control. Require an explicit enable request before leaving
   `Waiting`, including after a reset. `Error` is available for debugging. Track feedback age
   in ticks and return to `Waiting` when required feedback goes missing.
5. Implement `Control::update` to calculate current commands. Keep outputs zero outside `Running`
   and for motors with missing feedback.

Each tick calls `read_feedback`, `update_mode`, `update`, then `transmit`. The empty stages keep
startup runnable. Section comments provide instructions in English and Chinese.
每个周期依次读取反馈、更新模式、计算控制量并发送指令。各阶段初始为空，便于直接运行。
代码分区注释提供中英文练习说明。

The `todo!()` functions panic if called. The starter does not call them. Add motors to the array
and PID instances as needed for your exercise. The angle is in radians, speed and target are in
rpm, and measured and commanded currents are in amperes. Students implement the wire conversions;
do not send amperes as raw protocol counts.

## Stopping motors

`Motor::disable()` clears a motor's target and current command in RAM. `Control::disable_all()`
clears all configured motors. Each tick clears commands outside `Running`, before transmission.
Students must implement the transmit stage to send those zero commands over CAN.

`Motor::disable()` 清除内存中的目标值和电流指令，`Control::disable_all()` 清除所有已配置电机的指令。
每个周期在非 `Running` 模式下清零，再进入发送阶段。仍需实现 CAN 发送才能让电机收到零指令。

IWDG1 runs with a nominal 100 ms timeout and is fed only after a completed control tick.
If the task hangs, the watchdog resets the MCU. On panic, interrupts are disabled, the panic
message is written to nonblocking RTT, and the watchdog is reconfigured for its shortest timeout.
At nominal 32 kHz LSI, that is about 125 us after synchronization and reload, not an instant reset.
RTT delivery is best effort; the host may not read the panic text before RAM is reinitialized.

IWDG1 的标称超时时间为 100 ms，只在控制周期完成后喂狗，任务卡死会导致看门狗复位。
panic 时禁用中断，将消息写入非阻塞 RTT，再把看门狗设为最短超时。
32 kHz 标称 LSI 下，同步和重装后约 125 us 复位，并非立即复位。
RTT 消息不保证送达，主机可能来不及在 RAM 重新初始化之前读取 panic 信息。

On a watchdog reset, `dm_mc02::init()` clears the reset flags, prints a lockout message, and spins
before enabling CAN or starting the student task. Press RESET to run again; a power cycle also
works. Clearing the flags before halting allows that next manual reset to proceed. This assumes
the normal software-start IWDG option-byte setting; the template does not change option bytes.

看门狗复位后，`dm_mc02::init()` 清除复位标志，打印停机消息，并在启用 CAN 或启动学生任务前循环等待。
按 RESET 可重新运行，断电重启也可以。停机前清除标志，使下一次手动复位能够正常启动。
此逻辑假定 IWDG 选项字节采用正常的软件启动配置，本模板不会修改选项字节。

Motor output relies on the motor controller's command timeout after MCU/CAN reset. Before driving
a motor, establish that timeout and verify shutdown after a panic, a hung tick, and CAN loss.
The template does not implement or verify the motor-side timeout. Zero current does not imply
active braking or an immediate mechanical stop. Require fresh feedback and an explicit enable
request before entering `Prepare` after any restart.

MCU/CAN 复位后，电机输出依靠电机控制器端的指令超时机制关闭。驱动电机前，先确定该超时时间，
并测试 panic、周期卡死和 CAN 断线后的停止输出行为。本模板不实现或验证电机端超时机制。
零电流不代表主动刹车或机械上的立即停止。任何重启后都需要新反馈和明确的使能请求，才能进入 `Prepare`。

References: [Embassy CAN API](https://docs.embassy.dev/embassy-stm32/0.6.0/stm32h723vg/can/index.html)
and [DJI GM6020 documentation](https://www.robomaster.com/en-US/products/components/general/GM6020).

Watchdog/reset reference: [STM32H723 reference manual RM0468](https://www.st.com/resource/en/reference_manual/dm00603761.pdf).
