# Findings: what the HIL caught, and the fixes

Each of these was found by a scenario failing; numbers are before -> after on the same scenario.

| # | Problem | Root cause | Fix | Before -> after |
|---|---|---|---|---|
| 1 | Measured inductance 7x too high (1.76 mH for 0.25 mH) | calibration read `id` from the previous period: the dq currents are computed inside the current loop, which runs *after* the calibration step | compute this period's dq currents before the calibration step (`foc_dq`) | L error +600 % -> within 2.2 % on 20 units |
| 2 | Resistance read 22 % high with a single V/I ratio | the inverter's dead time drops a fixed voltage that looks like resistance | measure at 2 A and 4 A and use the difference | +22 % -> within 1.3 % |
| 3 | One unit in a 3-joint setup failed calibration | its hall edge sat 1.3 deg before angle 0; the rotor lags the swept vector, so the edge came after the sweep ended | sweep 2.25 turns instead of 2 | the failing unit and 40 more single units all calibrate |
| 4 | Watchdog reset during calibration | the comm task only marked itself alive when bytes arrived; with no bus traffic the watchdog was never kicked | the comm task also wakes on a 1 ms timeout | no resets |
| 5 | OVERVOLT trip when braking on a bench supply | braking energy charges the 470 uF bus capacitor at ~10 V/ms; a 1 kHz limiter is far too slow | braking-current limit in the 20 kHz ISR from the instantaneous bus voltage (26 -> 28.5 V) | 36.0 V trip -> 29.6 V peak, stops 150 -> 0 rad/s in 705 ms |
| 6 | After a limited braking phase the joint overshot to -53 rad/s | the velocity integrator kept winding while the ISR cut the current | freeze the velocity integrator while the braking limit is active | -53 -> -2.1 rad/s |
| 7 | 56 % overshoot on a small speed step | the 150 Hz speed PLL lagged ~2 ms | PLL at 400 Hz, speed gains retuned (kp 0.010, ki 0.3) | 56 % -> 12.5 % |
| 8 | Position step slow after #7 | position gain tuned for the old speed loop | pos_kp 40 -> 80 | 186 ms -> 110 ms |
| 9 | Winding reached 142 C in a stall | derating on the stator NTC, which lags the copper by minutes | derate on NTC + first-order winding-over-stator model driven by the measured copper loss | 142 C -> 104 C, estimate within 2.7 C |
| 10 | 1.3 A braking spike when enabling a spinning joint | the first PWM periods used zero-voltage duties | first duties = the back-EMF voltage vector | 1.3 A -> 0.07 A |
| 10b | ...my first fix made it worse (3.5 A) | I preloaded the q-axis integrator with the back-EMF, but the current loop's feed-forward already adds it, so the voltage doubled | removed the preload; set the first duties instead | 3.5 A -> 0.07 A |
| 11 | GCC-only warning in the flash driver (signed/unsigned compare) | clang does not warn there; the target build used clang | `tools/check_port.sh` compiles the port with host GCC and -Werror in CI | caught before hardware |

Things that were test-harness mistakes (not firmware): calibrating with a load attached, enabling joint 1
while joint 2 was still calibrating (joint 1 correctly timed out), and a fuzz test that "flipped" the same bit
twice.
