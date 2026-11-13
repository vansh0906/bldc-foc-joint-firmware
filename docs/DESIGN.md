# Design

## Hardware assumed

| Item | Value |
|---|---|
| MCU | STM32G431 (Cortex-M4F, FPU, 170 MHz), 128 KB flash, 22 KB SRAM used |
| Power stage | 3-phase MOSFET bridge, 24 V bus, low-side shunts into the G4 op-amps (as on ST's B-G431B-ESC1) |
| Motor | 7 pole pairs, R 0.25 ohm, L 0.25 mH, flux 8.5 mWb (Kt 0.089 Nm/A), 10 A peak |
| Position | 3 hall sensors (absolute, 60 deg electrical) + 4096-count ABZ encoder on TIM4 |
| Bus | RS-485 half duplex, 4 Mbaud, USART2 with hardware driver-enable, DMA transmit |
| Temperature | 10k NTC on the stator |

## Timing

| What | Rate | Where | Budget |
|---|---|---|---|
| ADC sample (3 currents + bus voltage) | 20 kHz | PWM center, TIM1 TRGO2 -> injected conversions | |
| Current loop (`app_isr`) | 20 kHz | ADC IRQ, NVIC priority 0 | 50 us per period |
| Outer loops + bus voltage + comm timeout (`app_ctrl_tick`) | 1 kHz | FreeRTOS task, priority 3 | |
| Bus frames (`app_comm_poll`) | per byte / 1 ms | FreeRTOS task, priority 2, woken by the RX IRQ | reply slot starts 30 us after the frame |
| Temperature, derating, watchdog (`app_monitor`) | 100 Hz | FreeRTOS task, priority 1 | |

The ADC IRQ runs above `configMAX_SYSCALL_INTERRUPT_PRIORITY`, so the kernel never masks it and it makes
no RTOS calls. Data shared with the tasks is single 32-bit floats (atomic on the M4); multi-field command
updates are done with interrupts briefly disabled.

New duties written in period k take effect in period k+1 and act for that whole period, so the inverse
Park transform uses the rotor angle advanced by 1.5 periods.

## Control

- **Current loop:** PI on d and q with pole-zero cancellation, `Kp = L * wc`, `Ki = R * wc`, wc = 2*pi*1 kHz,
  using the R and L measured at commissioning; cross-coupling and back-EMF feed-forward; voltage limited to
  the SVPWM hexagon's inscribed circle (d axis first); conditional integration as anti-windup.
- **SVPWM** by min/max zero-sequence injection, duty capped at 94 % so the low-side switch stays on long
  enough to sample the shunts.
- **Angle:** hall sectors at power-up (interpolated between edges at speed), encoder after the first hall
  edge, where the electrical angle is known from calibration. Every later edge cross-checks the encoder.
- **Speed:** PLL on the encoder count, 400 Hz bandwidth (critically damped).
- **Outer loops (1 kHz):** torque; velocity PI (anti-windup, frozen while the braking limit is active);
  position P -> velocity PI with feed-forward; impedance `tau = kp (q* - q) + kd (dq* - dq) + tau_ff`.

## Protections

| Condition | Detection | Action |
|---|---|---|
| Phase current > 15 A | every ISR (and TIM1 break input on the board) | bridge off, latched OVERCURRENT |
| Bus > 30 V for 2 ms / < 18 V for 10 ms | 1 kHz, averaged ISR samples | bridge off, latched |
| Braking pushes the bus up | every ISR, instantaneous bus voltage | braking current scaled to 0 between 26 and 28.5 V |
| Winding hot | 100 Hz: NTC + winding-over-stator model | current limit derated 100 -> 120 C (estimate), trip at 125 C |
| Encoder slips or disconnects | at hall edges: angle mismatch > 25 deg twice, or no counts across 2 edges | run on halls, current limit halved, ENCODER warning |
| No command for 100 ms | 1 kHz | DAMP: `tau = -kd_safe * vel` until CLEAR |
| Control task stops | ISR sees no control tick for 5 ms | bridge off, latched CTRL_STALL; watchdog resets the MCU |
| Watchdog | IWDG 50 ms, kicked only when the control and comm tasks both ran | reset, boots with the bridge off |

## Joint bus

Frame: `AA 55 | id | len | instr | data | CRC-16/MODBUS` (CRC over id..data, little-endian).

| Instr | Meaning |
|---|---|
| 0x10 SYNC_CMD (broadcast) | `n` entries of 16 bytes: id, mode, pos f32, vel f32, tau_ff i16 mNm, kp u16 (0.01), kd u16 (1e-4). Joint at list position k replies in slot k. |
| 0x11 CMD | the same 15-byte entry for one joint |
| 0x01 PING, 0x20 ENABLE, 0x21 DISABLE, 0x22 CLEAR, 0x23 CALIBRATE, 0x24 SAVE | reply: 17-byte status (state, mode, faults, pos, vel, iq, bus V, winding temp) |
| 0x02 READ / 0x03 WRITE | registers, 4 bytes each (gains, limits, timeout, calibration, statistics); out-of-range writes refused |

Replies set bit 0x80 in `instr`; a refused request also sets 0x40. Joints ignore frames with 0x80 set
(other joints' replies).

Cycle budget at 4 Mbaud (`hil/jbus.py: bus_cycle_us`):

| Joints | Command frame | Cycle (command + all replies) |
|---|---|---|
| 1 | 60 us | 150 us |
| 4 | 180 us | 465 us |
| 6 | 260 us | 675 us |
| 8 | 340 us | 885 us |

## Commissioning (`firmware/src/calib.c`)

1. Lock the rotor with 2 A on the d axis at angle 0.
2. R from 2 A and 4 A: `R = (V2 - V1) / (I2 - I1)` (cancels the dead-time drop).
3. L from the current response to a 3 V d-axis step, 8 repeats.
4. Sweep a 4 A current vector 2.25 electrical turns forward and back at 1 turn/s; record every hall
   transition both ways and average; check encoder direction and counts per electrical turn.
5. Current-loop gains from the measured R and L; SAVE writes everything to flash with a CRC.

## Simulator (test equipment, `sim/`)

PMSM in the rotor frame (RK2, 4 sub-steps per PWM period) with slight saliency, 6th-harmonic back-EMF and
torque ripple, cogging, Coulomb and viscous friction; inverter with 200 ns dead time; DC bus with a
470 uF capacitor and a supply that can or cannot sink current; winding -> stator -> ambient thermal model
with a lagging NTC; ADC gain/offset errors and noise; hall placement errors and hysteresis; encoder
mounting offset. Unit-to-unit spread: R +-8 %, L +-10 %, flux +-3 %, hall placement +-4 deg.
