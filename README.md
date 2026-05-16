# Self-Balancing-Cube

**UPDATE (2026-05-16)**

Code is in "esp32_cube" folder. 

Pair via BlueTooth or USB Serial, there is a help menu:

=== Commands ===

h or ?   - Show help

a        - Toggle live angles

d        - Toggle debug (Serial Monitor only)

p+/p-    - Adjust K1

i+/i-    - Adjust K2

s+/s-    - Adjust K3

c+       - Start calibration

c-       - Save current position

You can print redesigned cube https://www.thingiverse.com/thing:6695891

Folow this video https://youtu.be/ZU0oTBRDgOE

---

I created a PCB with KiCad, as it was cleaner than using a prototype board and provided GERBER files for PCB Manufacturing in "Gerber" 

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


