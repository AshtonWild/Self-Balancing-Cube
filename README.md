# Self-Balancing-Cube

**UPDATE (2026-11-11)**

You can print redesigned cube https://www.thingiverse.com/thing:6695891

Folow this video https://youtu.be/ZU0oTBRDgOE

---

I created a PCB with KiCad, as it was cleaner than using a prototype board and provided GERBER files for PCB Manufacturing in "Gerber".  I used JLCPCB to have the PCB built.

All parts are available via Digikey or AliExpress.  

CPU: ESP32 DEVKIT v1 Type-C

Battery: 3S1P LiPo 500 mAH (11.1V)

Motor: Nidec 24H brushless motors (8 Pin Required)

Other: Regulator (L7805), Transistor 2N2222, 5V Active Buzzer, Capacitor 100uF / 0.1uF, Register 6.8k / 33k / 10k

All red connections not nescesary for this project! But if you are designing a PCB I recommend making these connections. Maybe I use encoders in the future, you will be able to use the new firmware without any changes.
 
How to build:

https://youtu.be/AJQZFHJzwt4

If something doesn't work, try the motors test sketch. It tests all motors, rotation directions and speeds. This helps you understand the problem is in software or in hardware.

ESP32 version also has an updated balancing point setting procedure. Important! In this video you can learn how to set the balancing points:

https://youtu.be/Nkm9PoihZOI

# Self-Balancing Cube Calibration Manual

Calibrate in this order: sensor check, motor test, edge, then vertex. Each stage depends on the one before it, and nothing is kept until you type `save`.

## Before you start

- **Firmware**: the cube's Arduino code (ESP32, based on remrc's Self-Balancing-Cube), including the `watch` command and the automatic MPU6050 re-wake.
- **Console over USB**: Arduino Serial Monitor at **115200 baud**, line ending set to **New Line**. With "No line ending", commands never run.
- **Console over Bluetooth**: pair with **ESP32-Cube** and use any serial terminal app.
- **Chaining**: several commands can go on one line, separated by `;` (for example `zK2 8; zK3 0.3`).
- **Unsaved changes are lost** on every reboot and every upload. Type `save` once something works.
- **Safety**: keep fingers clear of the wheels. They can kick hard when balancing starts or a gain is wrong. Type `stop` at any time to brake the wheels, and `start` to re-enable them.

## Step 1: Power up and check the sensor

The sensor must give live readings before any calibration means anything. Our first vertex calibration was recorded while it read all zeros.

1. Power up with the cube **lying still**. The blue LED stays on for about 1 s while the gyro zeroes.  
   If the console says the cube moved, keep it still and type `gyrocal`.
2. Type `watch` (or `w`). You get a live line about 4 times a second, starting with `raw X … Y … Z …`.
3. Tilt the cube by hand. All three raw values must change. Lying flat, one of them reads about ±16384 (1 g).
4. Type `w off` to stop the live view.

If the raw values are all 0: the MPU6050 is powered but asleep, usually because it wasn't connected at boot. Type `reboot` with the cube still. The firmware now also re-wakes the sensor by itself and prints `MPU6050 found - woken up and configured`.

If you see `IMU read errors - check the MPU6050 wiring`: check SDA on GPIO 21, SCL on GPIO 22, VCC and GND, and that AD0 is low (address 0x68). If errors only appear when the motors run, keep the I2C wires short and away from the motor wires.

## Step 2: Check the battery and motors

1. **Battery (once)**: measure the pack with a multimeter and type the reading, for example `vcal 11.8`. This makes the battery value in `s` and the low-battery alarm (8 to 9.5 V) correct.  
   A 3S pack below about 11 V gives the wheels less top speed, so the cube falls sooner.
2. **Motors**: lay the cube on a side so the wheels spin freely, then type `mtest`. Each wheel spins for 0.6 s.  
   - **OK** for all three: continue.  
   - **encoder counts backwards**: swap `ENCx_1` and `ENCx_2` for that motor in the PINS section and re-upload.  
   - **not spinning**: check the battery and the BRAKE, DIR, PWM and encoder wiring.  
   Type `stop` at any point to abort the test and brake the wheels.

Don't continue until all three motors pass.

## Step 3: Calibrate the edge

The edge uses one motor (motor 3) and one tilt axis, so it's the easiest place to start. The original gains, eK1 190 and eK2 31, were correct: don't flip their signs.

1. Check the gains: type `eK1` and `eK2`. They should read 190 and 31. If not, type `defaults` (this keeps your calibration).
2. Record the balance point: type `cal e`. During the 3 s countdown, stand the cube on its edge and hold it lightly where it would fall neither way.  
   Run `watch` during the countdown to see the check: `|X|` must be 7000 to 10000 and `|Y|` below 2000, both marked `ok`.  
   Blue LED on = right position. Fast flicker = wrong position.  
   Hold still while it samples (about 1.5 s). LED on for 2 s = accepted; flicker = rejected, and the console says why.
3. Give it room: type `fall 10` so it doesn't cut out at 7° while you tune. Optionally type `arme 1` to make it start more easily.
4. Let it balance: stand it on the same edge. The console prints `Balancing on EDGE`.
5. Tune if it rocks: a slow, growing rock means it needs more damping. Change eK2 one step at a time and leave eK1 alone.  
   - Rocking gets smaller: keep going.  
   - Buzzing or jittering: one step too far, so back off.
6. Trim: while it balances on its own, type `autotrim` and don't touch it for about 9 s. Run it a second time. Our result was `te = −1.09`.
7. Save: type `save`. Then `reboot` and check it still balances on the edge.

## Step 4: Calibrate the vertex

The vertex worked with the original gains too: K1 180, K2 30, zK2 8, zK3 0.3. The step that fixed it was setting the balance-point trims by hand while watching the live tilt.

1. Check the gains: type `K1`, `K2`, `zK2` and `zK3`. They should read 180, 30, 8 and 0.3.
2. Record the balance point: type `cal v`. During the countdown, stand the cube on its corner and hold it lightly at its balance point.  
   In `watch`, `|X|` and `|Y|` must both be below 2000 and marked `ok`. The rotation must stay under 15 deg/s while it samples.
3. Make starting easier: type `armv 1`. With the default 0.4° window, it's very hard to start by hand.
4. Measure the offset with the motors off: type `stop`, then `watch`. Hold the cube at the point where it really balances (it would fall equally either way). Read the line, for example `vertex tilt X +2.10 off, Y -1.30 off`.
5. Set the trims: type `tvx` and `tvy` to see the current values. Add the readings to them: with both at 0 and the example above, that's `tvx 2.1; tvy -1.3`.
6. Check: at the real balance point, `watch` should now read close to `X +0.00 ok, Y +0.00 ok`.
7. Balance: type `start`, hold it at the balance point, and let go when it takes over. The console prints `Balancing on VERTEX`.
8. Trim: while it balances, run `autotrim` twice (about 9 s each, hands off).
9. Save: type `save`, then `reboot` and check both the corner and the edge still balance.

## Troubleshooting

Most problems we hit were the balance point or the sensor, not the gains. Change one thing at a time.

| Symptom | Cause | Fix |
|---------|-------|-----|
| Raw X, Y, Z all read 0 | MPU6050 asleep (not connected at boot) | `reboot` with the cube still |
| IMU read errors | I2C wiring or noise | Check SDA 21, SCL 22, power, AD0; short wires away from motors |
| Motors never start | Tilt outside armv / arme, motors stopped, or not calibrated | Read the `watch` line; `armv 1`, `start`, or `cal v` / `cal e` |
| `watch` never shows vertex tilt | Raw \|X\| at the corner is 2000 or more | Redo `cal v`; send the raw values if it persists |
| Falls at the real balance point, starts only when tilted further | Zero point recorded in the wrong spot | Set `tvx` / `tvy` by hand (Step 4) |
| Slow rock that grows until it falls | Too little damping | Nudge eK2 (edge) or K2 (vertex) one step at a time |
| Buzzing or fast jitter | Gains too high | Back off one step; try `dlpf 3` |
| Balances, then the wheels speed up and it falls | Small balance-point offset; wheels hit full speed | `autotrim`; or watch `ix` / `iy` in `tel 20` and nudge the trim until flat |
| Cube spins faster and faster around its vertical axis | Yaw gains wrong | Keep `zK2 8`; try `zK3 0.4` or `0.2`. `zK2 -8` made it much worse |
| Gains seem to change by themselves | Not saved before a reboot or upload | `save` after every change you want to keep |
| A command prints nothing | Line ending, baud, or output scrolled past | New Line, 115200, `w off`; `tel off`; type a name alone, e.g. `eK1` |

Flipping the gain signs (negative eK1 / eK2 or K1 / K2) did not help on this cube: the original positive values are correct.

## Reference

Working settings on this cube. Type `p` for the full list. Gains marked "default" come back with `defaults`; the others are saved with `save` only.

| Setting | Value | Notes |
|---------|-------|-------|
| eK1 / eK2 | 190 / 31 | Edge gains, default |
| te | −1.09 | Edge trim from `autotrim` |
| K1 / K2 | 180 / 30 | Vertex gains, default |
| zK2 / zK3 | 8 / 0.3 | Yaw gains, default |
| tvx / tvy | from Step 4 | Vertex trims, set by hand then `autotrim` |
| armv | 1 | Default 0.4; reset by `defaults` |
| fall | 10 | Default 7; set back with `fall 7` if you like |

## Console menu explained

Type `help` to show the menu on the cube. Commands aren't case-sensitive, short forms are in brackets, and `help <topic>` (for example `help cal` or `help K1`) gives more detail on any item.

### Info

| Command | What it does |
|---------|--------------|
| `help` (`?`, `menu`) | Shows the menu. `help <topic>` explains one command or setting in detail. |
| `help leds` | Explains the blue LED blink patterns and RGB colours (table below). |
| `status` (`s`) | One-screen health check: mode (IDLE, VERTEX or EDGE), motors enabled or stopped, which position the cube is in, battery volts, angles, raw sensor values, wheel speeds and loop timing. Worst period should stay near 15000 us; late and I2C errors should stay at 0. |
| `params` (`p`) | Lists every tunable value, its current setting and what it does. Says if there are unsaved changes. |
| `tel [hz]` / `tel off` | Streams live data for the Arduino Serial Plotter (Tools > Serial Plotter). Default 20 Hz, maximum 66; keep it at 20 Hz or less over Bluetooth. Fields: m mode (0 idle, 1 vertex, 2 edge), ax ay tilt angle, gx gy tilt rate, w1–w3 wheel speeds, px py pz motor output (−255 to 255), ix iy wheel-position term. A steady ramp in ix or iy means the balance point is off. |
| `watch [hz]` (`w`) / `w off` | Readable live guide, default 4 Hz. Shows the raw sensor values and what's in or out of range: the calibration windows during `cal`, or the tilt from the balance point and the start window otherwise. Notes `[motors STOPPED]` when the wheels are off. |

### Setup and calibration

| Command | What it does |
|---------|--------------|
| `mtest` | Motor and encoder test, about 3 s. Lay the cube on a side first. Spins each wheel for 0.6 s and reports OK, backwards (swap that motor's encoder pins) or not spinning (check power and wiring). Won't start while the motors are stopped; `stop` aborts it. |
| `cal v [sec]` | Records the vertex balance point after a countdown (default 3 s). Needs raw \|X\| and \|Y\| below 2000 and rotation under 15 deg/s. Resets the vertex trims to 0. |
| `cal e [sec]` | Records the edge balance point the same way. Needs raw \|X\| of 7000 to 10000 and \|Y\| below 2000. Resets the edge trim to 0. |
| `autotrim` | Run while the cube balances on its own. Averages the lean for about 9 s and moves the trims (`tvx` / `tvy` on the vertex, `te` on the edge) to the real balance point. Run it twice, then `save`. Aborts if the cube stops balancing. |
| `gyrocal [sec]` | Re-zeroes the gyro (also done automatically at power-up). Keep the cube still; rejected if it moves. Use it if the angle drifts while the cube sits still. |
| `vcal <volts>` | Type a multimeter reading of the battery, e.g. `vcal 11.8`, to make the battery value and low-battery alarm correct. Rejects readings that don't make sense (for example a disconnected divider). |
| `buzz` | Two-part buzzer test. Test 1 (steady on): an active buzzer beeps. Test 2 (2 kHz tone): a passive buzzer beeps. The project needs an active 5 V buzzer. Silent in both: check polarity, 5 V and the transistor on GPIO 27. |

### Tuning

| Command | What it does |
|---------|--------------|
| `<name> <value>` | Sets a value, e.g. `K1 185`. Also works as `set K1 185` or `K1=185`. Values outside the allowed range are limited. |
| `<name>` | Prints one value, e.g. `eK2`. |
| `<name>+` / `<name>-` | Nudges a value by one step; repeat the sign for more steps (`eK3+++`). |

| Setting | What it controls | Default | Range (step) |
|---------|------------------|---------|--------------|
| K1 | Vertex: angle gain | 180 | −2000 to 2000 (5) |
| K2 | Vertex: tilt-rate gain (damping) | 30 | −500 to 500 (1) |
| K3 | Vertex: wheel-speed gain | 1.6 | −50 to 50 (0.1) |
| K4 | Vertex: wheel-position gain | 0.008 | −1 to 1 (0.001) |
| zK2 | Vertex: yaw-rate gain (stops turning) | 8 | −100 to 100 (0.5) |
| zK3 | Vertex: yaw wheel-speed gain | 0.3 | −10 to 10 (0.05) |
| eK1 | Edge: angle gain | 190 | −2000 to 2000 (5) |
| eK2 | Edge: tilt-rate gain (damping) | 31 | −500 to 500 (1) |
| eK3 | Edge: wheel-speed gain | 2.5 | −50 to 50 (0.1) |
| eK4 | Edge: wheel-position gain | 0.014 | −1 to 1 (0.001) |
| tvx / tvy | Vertex trims, degrees | 0 (set by `cal v`) | −5 to 5 (0.05) |
| te | Edge trim, degrees | 0 (set by `cal e`) | −5 to 5 (0.05) |
| gyroamt | How much the angle trusts the gyro over the accelerometer | 0.996 | 0.9 to 0.9999 (0.001) |
| alpha | Tilt-rate smoothing (1 = off) | 0.7 | 0.05 to 1 (0.05) |
| armv | Vertex: starts balancing within this angle, degrees | 0.4 | 0.1 to 5 (0.1) |
| arme | Edge: starts balancing within this angle, degrees | 0.3 | 0.1 to 5 (0.1) |
| fall | Stops balancing above this tilt, degrees | 7 | 2 to 20 (0.5) |
| vdiv | Battery ADC counts per volt (set by `vcal`) | 204 | 50 to 1000 (1) |
| dlpf | MPU6050 filter: 0 = 260 Hz, 1 = 184, 2 = 94, 3 = 44, 4 = 21 | 0 | 0 to 6 (1) |

### Motors and settings

| Command | What it does |
|---------|--------------|
| `stop` / `start` | `stop` brakes the wheels and keeps them off (and cancels a countdown or motor test) until `start`. Settings aren't affected. |
| `save` | Writes all settings and calibration to flash. Changes work straight away but are lost on reboot until saved. |
| `load` | Re-reads flash, undoing anything not yet saved. |
| `defaults` | Puts back the original gains, filters and start/stop angles. Keeps calibration, trims and vdiv. Type `save` to keep it. |
| `factory` | Erases everything including calibration, and saves immediately. The cube must be lying down. |
| `reboot` | Restarts the ESP32. Keep the cube still afterwards while the gyro zeroes. |

## LED meanings

The optional RGB LEDs on GPIO 19 show the same states in colour: green = good, red = problem, yellow = sampling or getting close, purple = not calibrated, cyan = motor test, dim green = balancing, dim red = stopped.

| Blue LED (GPIO 2) | Meaning |
|-------------------|---------|
| On at power-up | Zeroing the gyro: keep the cube still |
| Double blink | Not calibrated yet |
| On during a `cal` countdown | Right position |
| Fast flicker during a countdown | Wrong position |
| Steady blinking while sampling | Hold still |
| On for 2 s after a job | Accepted |
| Fast flicker for 2 s after a job | Rejected (the console says why) |
| Slow / quicker blink / on, when ready | Within 5° / 2.5° / 1° of the balance point |
| On | Balancing |
| Short blip every 2 s | Motors stopped (type `start`) |
| Slow blink | Motor test, or a `gyrocal` / `mtest` countdown |
