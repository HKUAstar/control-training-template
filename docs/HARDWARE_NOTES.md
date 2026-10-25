> 本文保留模板原有底层说明。Exercises 中的默认模式仅描述原始骨架；本次作业的回正、遥控器操作和输出许可要求以根目录 README 为准。

# Control training template

An Embassy starter for the DM-MC02 with an STM32H723VG. Students implement a GM6020-style
motor protocol, PID, and state machine. With no motors configured, the starter stays in `Waiting`
and sends no motor commands.

There are two source files:

- `src/dm_mc02.rs`: clocks, CAN1, DR16 reception, watchdog, and panic handling.
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

## DR16 remote input

Connect the DR16 to the board's DBUS port: UART5 RX on PD2, using DMA1 channel 5 at 100 kbaud,
8 data bits, even parity, and 2 stop bits. The board provides signal inversion; do not enable
software inversion. Initialization enables the board's switched 5 V output on PC15.

将 DR16 接到开发板的 DBUS 接口，UART5 的 PD2 引脚接收，使用 DMA1 通道 5。
串口配置为 100 kbaud、8 位数据、偶校验、2 位停止位。板载电路已反相，无需软件再次反相。
初始化时会开启 PC15 控制的 5 V 输出。

The receiver task waits for UART idle and accepts exactly one valid 18-byte frame. It publishes
only the latest decoded input, without blocking the control task. `self.remote` is `Some(Dr16)`
while input is valid, or `None` before reception, after an invalid frame/UART error, or after
100 control ticks without a valid frame. Missing input clears motor commands and forces `Waiting`.
Remote recovery does not enable motors automatically; implement the explicit enable condition.

接收任务等待 UART 空闲间隔，只接受完整有效的 18 字节帧，并只发布最新遥控输入。
有效时 `self.remote` 为 `Some(Dr16)`；未接收到数据、收到无效帧或串口错误，
以及连续 100 个控制周期没有有效帧时为 `None`。遥控输入缺失会清除电机指令并返回 `Waiting`。
遥控恢复后不会自动使能电机，请在模式切换练习中实现明确的使能条件。

`remote.channels` contains right horizontal, right vertical, left horizontal, left vertical, and
wheel counts, centered at zero in `-660..=660`. Divide by `660.0` for a normalized value and add
a deadzone as needed. Both switches use wire values: 1 up, 2 down, 3 middle. Mouse deltas, mouse
buttons, and keyboard bits are also available. No axis remapping or automatic target assignment
is applied. For example, use `remote.channels[1]` in `update` to choose a speed target.

`remote.channels` 依次为右横、右纵、左横、左纵和拨轮，中心为 0，范围为 `-660..=660`。
除以 `660.0` 可归一化，按需添加死区。两开关的值为 1 上、2 下、3 中。
还提供鼠标位移、鼠标按键和键盘位图。模板不自动变换坐标或设置目标值。
例如，在 `update` 中使用 `remote.channels[1]` 设置速度目标。

This is a basic idle-framed receiver, without a CRC or a stream resynchronizer. UART framing,
receiver power, channel directions, and disconnect behavior have not been tested on hardware.

这是按空闲间隔分帧的基础接收实现，不包含 CRC 或字节流重同步器。
串口分帧、接收机供电、通道方向和断线行为尚未经过硬件测试。

## Exercises

1. Implement `Motor::read` and `Motor::transmit`. Use the GM6020 manual to choose CAN IDs, byte
   order, signed values, scaling, and limits. GM6020 IDs 1 through 4 use feedback IDs `0x205`
   through `0x208`. The provided current-mode command ID is `0x1FE`; adapt for another protocol.
   The transmit array has four slots for motor IDs 1 through 4. Encode zero for `None`.
   The function returns eight payload bytes; `MOTOR_COMMAND_ID` sets the group CAN ID.
2. Add PID fields and implement `reset` and `calculate`. The initial exercise controls speed in
   rpm and produces a current command in amperes. Pass `0.001` for the 1 ms tick period.
3. CAN transport is provided: interrupts buffer incoming frames, and `Control::read_feedback`
   shows a nonblocking `try_receive()` loop for at most eight queued entries per tick. Implement
   CAN ID filtering, payload checks, and routing to the matching `Motor::read` yourself.
   `Control::transmit` calls `Motor::transmit` and queues
   one group frame per tick when any motor is configured. A full transmit queue triggers panic
   and the existing watchdog-reset lockout, rather than blocking the control loop.
4. Implement `Control::update_mode`: `Waiting` waits for hardware, `Prepare` resets controllers,
   and `Running` enables normal control. Require an explicit enable request before leaving
   `Waiting`, including after a reset. `Error` is available for debugging. Track feedback age
   in ticks and return to `Waiting` when required feedback goes missing.
5. Implement `Control::update` to calculate current commands. Keep outputs zero outside `Running`
   and for motors with missing feedback.

Each tick calls `read_remote`, `read_feedback`, `update_mode`, `update`, then `transmit`. With all motor slots
set to `None`, startup stays runnable. Section comments provide instructions in English and Chinese.
每个周期依次读取遥控输入和电机反馈、更新模式、计算控制量并发送指令。所有电机槽位初始为 `None`，便于直接运行。
代码分区注释提供中英文练习说明。

The `todo!()` functions panic if called. With no motors configured, the starter does not call them.
Implement both motor stubs before configuring a motor: transmission calls the encoder even in
`Waiting`. Your feedback-routing code should call the decoder for matching frames.
Add motors to the array and PID instances as needed for your exercise. The angle is in radians, speed and target are in
rpm, and measured and commanded currents are in amperes. Students implement the wire conversions;
do not send amperes as raw protocol counts.

## Stopping motors

`Motor::disable()` clears a motor's target and current command in RAM. `Control::disable_all()`
clears all configured motors. The transmit stage clears commands outside `Running` before
calling the student encoder.
Students must implement `Motor::transmit` so those cleared commands encode as zero on the wire.

`Motor::disable()` 清除内存中的目标值和电流指令，`Control::disable_all()` 清除所有已配置电机的指令。
发送阶段在非 `Running` 模式下清零。仍需实现 `Motor::transmit`，
将清零后的指令编码为协议零值。

IWDG1 runs with a nominal 100 ms timeout and is fed only after a completed control tick.
If the task hangs, the watchdog resets the MCU. On panic, interrupts are disabled and a fresh
nominal 100 ms watchdog deadline is armed, including for panics before normal initialization.
The RTT channel switches to blocking mode for the panic message and flushes it so the probe can
read it before reset. Normal logging stays nonblocking. If there is no reader, the watchdog ends
the wait. This allows up to nominally 100 ms of extra time before MCU/CAN reset on panic.

任务卡死会导致看门狗复位。panic 时禁用中断，并重新设置标称 100 ms 的看门狗期限，
也覆盖正常初始化之前发生的 panic。RTT 为 panic 消息切换至阻塞模式，并等待探针读走消息。
正常日志仍为非阻塞。没有主机读取时，看门狗会结束等待。
这意味着 panic 后 MCU/CAN 复位前可能额外等待标称 100 ms。

Once the probe has read the message, the watchdog is configured for its shortest timeout.
At nominal 32 kHz LSI, that is about 125 us after synchronization and reload, not an instant reset.
Delivery is still best effort if the host is too slow, and debug watchdog-freeze settings can
prevent the watchdog from bounding the wait while the core is halted.

探针读走消息后，看门狗切换到最短超时；32 kHz 标称 LSI 下，同步和重装后约 125 us 复位。
如果主机读取过慢，消息仍可能丢失。调试时若配置了看门狗冻结，内核暂停期间的等待将不再有此期限。

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
and
[DJI GM6020 documentation](https://www.robomaster.com/en-US/products/components/general/GM6020).

Watchdog/reset reference:
[STM32H723 reference manual RM0468](https://www.st.com/resource/en/reference_manual/dm00603761.pdf).

DR16 protocol: [DJI DT7/DR16 manual][dr16-manual].

[dr16-manual]: https://cdn-hz.robomaster.com/tem/09f88aafe7ae51516874829340553427.pdf
