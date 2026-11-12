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

Calibrate in this order: sensor check, motor test, edge, then vertex. Each stage depends on the one before it, and nothing is kept until you type save. 

Before you start:
- Firmware: the cube's Arduino code (ESP32, based on remrc's Self-Balancing-Cube), including the watch command and the automatic MPU6050 re-wake.
- Console over USB: Arduino Serial Monitor at 115200 baud, line ending set to New Line. With "No line ending", commands never run.
- Console over Bluetooth: pair with ESP32-Cube and use any serial terminal app. 
- Chaining: several commands can go on one line, separated by ; (for example zK2 8; zK3 0.3 ). Unsaved changes are lost on every reboot and every upload. Type save once something works.
- Safety: keep fingers clear of the wheels. They can kick hard when balancing starts or a gain is wrong. Type stop at any time to brake the wheels, and start to re-enable them.
  
Step 1 - Power up and check the sensor: 

The sensor must give live readings before any calibration means anything. Our first vertex calibration was recorded while it read all zeros.
1. Power up with the cube lying still. The blue LED stays on for about 1 s while the gyro zeroes. If the console says the cube moved, keep it still and type gyrocal.
2. Type watch (or w ). You get a live line about 4 times a second, starting with raw X … Y … Z … .
3. Tilt the cube by hand. All three raw values must change. Lying flat, one of them reads about ±16384 (1 g).
4. Type w off to stop the live view.
If the raw values are all 0: the MPU6050 is powered but asleep, usually because it wasn't connected at boot. Type reboot with the cube still. The firmware now also re-wakes the sensor by itself and prints MPU6050 found - woken up and configured.

If you see IMU read errors - check the MPU6050 wiring : check SDA on GPIO 21, SCL on GPIO 22, VCC and GND, and that AD0 is low (address 0x68). If errors only appear when the motors run, keep the I2C wires short and away from the motor wires.

Step 2 - Check the battery and motors:
1. Battery (once): measure the pack with a multimeter and type the reading, for example vcal 11.8 . This makes the battery value in s and the low-battery alarm (8 to 9.5 V) correct.
   - A 3S pack below about 11 V gives the wheels less top speed, so the cube falls sooner.
2. Motors: lay the cube on a side so the wheels spin freely, then type mtest . Each wheel spins for 0.6 s.
   - OK for all three: continue.
   - encoder counts backwards : swap ENCx_1 and ENCx_2 for that motor in the PINS section and re-upload.
   - not spinning : check the battery and the BRAKE, DIR, PWM and encoder wiring.
   - Type stop at any point to abort the test and brake the wheels.
Don't continue until all three motors pass.

Step 3 - Calibrate the edge:
The edge uses one motor (motor 3) and one tilt axis, so it's the easiest place to start. The original gains, eK1 190 and eK2 31, were correct: don't flip their signs.
1. Check the gains: type eK1 and eK2 . They should read 190 and 31. If not, type defaults (this keeps your calibration).
2. Record the balance point: type cal e . During the 3 s countdown, stand the cube on its edge and hold it lightly where it would fall neither way.
   - Run watch during the countdown to see the check: |X| must be 7000 to 10000 and |Y| below 2000, both marked ok .
   - Blue LED on = right position. Fast flicker = wrong position.
   - Hold still while it samples (about 1.5 s). LED on for 2 s = accepted; flicker = rejected, and the console says why.
3. Give it room: type fall 10 so it doesn't cut out at 7° while you tune. Optionally type arme 1 to make it start more easily.
4. Let it balance: stand it on the same edge. The console prints Balancing on EDGE.
5. Tune if it rocks: a slow, growing rock means it needs more damping. Change eK2 one step at a time and leave eK1 alone.
   - Rocking gets smaller: keep going.
   - Buzzing or jittering: one step too far, so back off.
6. Trim: while it balances on its own, type autotrim and don't touch it for about 9 s. Run it a second time. Our result was te = −1.09.
7. Save: type save . Then reboot and check it still balances on the edge.

Step 4 - Calibrate the vertex:
The vertex worked with the original gains too: K1 180, K2 30, zK2 8, zK3 0.3. The step that fixed it was setting the balance-point trims by hand while watching the live tilt.
1. Check the gains: type K1 , K2 , zK2 and zK3 . They should read 180, 30, 8 and 0.3.
2. Record the balance point: type cal v . During the countdown, stand the cube on its corner and hold it lightly at its balance point.
   - In watch , |X| and |Y| must both be below 2000 and marked ok . The rotation must stay under 15 deg/s while it samples.
3. Make starting easier: type armv 1 . With the default 0.4° window, it's very hard to start by hand.
4. Measure the offset with the motors off: type stop , then watch . Hold the cube at the point where it really balances (it would fall equally either way). Read the line, for example vertex tilt X +2.10 off, Y -1.30 off .
5. Set the trims: type tvx and tvy to see the current values. Add the readings to them: with both at 0 and the example above, that's tvx 2.1; tvy -1.3 .
6. Check: at the real balance point, watch should now read close to X +0.00 ok, Y +0.00 ok .
7. Balance: type start , hold it at the balance point, and let go when it takes over. The console prints Balancing on VERTEX.
8. Trim: while it balances, run autotrim twice (about 9 s each, hands off).
9. Save: type save , then reboot and check both the corner and the edge still balance.
