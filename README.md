# STM32 Encoder Motor PID Speed Control

This repository contains an STM32-based DC encoder motor speed control test using PID control.

The purpose of this test was to verify encoder-based RPM measurement and PID speed control before applying the control logic to a conveyor motor system in a recycling automation project.

---

## Project Context

In the main recycling automation system, a conveyor belt motor was originally planned to be controlled using encoder feedback and PID speed control.

However, during hardware testing, the encoder sensor of the actual conveyor motor was found to be faulty. Because replacing the motor was costly, the final conveyor system was operated using open-loop PWM control for stable operation.

This repository documents the PID control test that was successfully performed using a separate encoder motor setup.

---

## Key Features

- STM32-based DC motor speed control
- Encoder feedback using TIM2 Encoder Mode
- PWM motor output using TIM3
- PID-based RPM control
- Button-based motor start/stop control
- UART output for RPM and PWM monitoring
- Serial data output for graph visualization

---

## Hardware Configuration

| Component | Description |
|---|---|
| MCU | STM32 development board |
| Motor | DC encoder motor |
| Motor Driver | BTS7960 motor driver |
| Encoder Input | TIM2 Encoder Mode |
| PWM Output | TIM3 Channel 1 / Channel 2 |
| UART | USART2, 115200 bps |
| Button | On-board user button |

---

## Pin / Peripheral Usage

| Function | STM32 Peripheral / Pin |
|---|---|
| Encoder Input | TIM2 Encoder Mode |
| PWM Output | TIM3 CH1, TIM3 CH2 |
| Motor Driver R_EN | PC7 |
| Motor Driver L_EN | PA9 |
| UART Debug Output | USART2 |
| Start / Stop Button | B1 Button |

---

## Control Parameters

```c
#define PPR 1430
#define DT 0.1f   // 100ms

float TEST_RPM = 100.0f;

float Kp = 0.4f;
float Ki = 0.001f;
float Kd = 0.00f;
```

The control loop calculates the motor RPM every 100 ms using encoder count differences.

```c
rpm = (diff * 60.0f) / (PPR * DT);
```

The PID controller updates the PWM command based on the RPM error.

```c
error = target_rpm - rpm_abs;
integral += error * DT;
derivative = (error - prev_error) / DT;

float output = Kp * error + Ki * integral + Kd * derivative;
pwm += (int)output;
```

PWM output is limited between 0 and 999.

```c
if (pwm > 999) pwm = 999;
if (pwm < 0) pwm = 0;
```

---

## Control Flow

1. Start PWM output on TIM3 CH1 and CH2.
2. Enable the BTS7960 motor driver.
3. Start TIM2 Encoder Mode.
4. Wait for the user button input.
5. When the button is pressed, set the target speed to `TEST_RPM`.
6. Read encoder count every 100 ms.
7. Calculate current RPM.
8. Calculate PID output.
9. Update PWM duty.
10. Print RPM and PWM values through UART.

---

## UART Output Format

The code outputs RPM and PWM values through UART.

```c
printf("%d,%d\r\n", (int)rpm_abs, pwm);
```

Output format:

```text
RPM,PWM
```

Example:

```text
98,421
100,430
101,428
```

This format can be used with STM32CubeMonitor, Serial Plotter, or other graph monitoring tools.

---

## Result

The PID control logic was verified using a separate encoder motor test setup.

The motor speed was controlled toward the target RPM, and RPM/PWM data were monitored through UART. This test confirmed that encoder feedback, RPM calculation, PWM output, and PID control logic could be implemented on STM32.

---

## Why PWM Was Used in the Final Conveyor System

Although the PID control test worked with a separate encoder motor, the encoder sensor of the actual conveyor motor was not reliable.

Because stable encoder feedback is required for closed-loop PID control, applying PID directly to the faulty conveyor motor caused unreliable speed feedback.

Replacing the conveyor motor was not practical due to cost constraints. Therefore, the final conveyor system was operated using open-loop PWM control to secure stable operation with the available hardware.

---

## Repository Structure

```text
stm32-encoder-pid-control/
 ├─ README.md
 └─ Core/
     └─ Src/
         └─ main.c
```

---

## Portfolio Relevance

This code demonstrates:

- STM32 timer-based PWM control
- Encoder-based RPM measurement
- PID speed control implementation
- UART-based debugging and graph monitoring
- Hardware limitation analysis
- Practical engineering decision-making under real project constraints

This test was part of the motor control validation process for an embedded recycling automation project.
