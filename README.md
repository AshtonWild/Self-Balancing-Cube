# Self-Balancing-Cube

**UPDATE (2026-05-16)**

Code is in "esp32_cube" folder. 

Pair via BlueTooth or USB Serial, there is a help menu:

void showHelp() {
  SerialBT.println("\n=== Commands ===");
  SerialBT.println("h or ?   - Show help");
  SerialBT.println("a        - Toggle live angles");
  SerialBT.println("d        - Toggle debug (Serial Monitor only)");
  SerialBT.println("p+/p-    - Adjust K1");
  SerialBT.println("i+/i-    - Adjust K2");
  SerialBT.println("s+/s-    - Adjust K3");
  SerialBT.println("c+       - Start calibration");
  SerialBT.println("c-       - Save current position");
}

You can print redesigned cube https://www.thingiverse.com/thing:6695891

Folow this video https://youtu.be/ZU0oTBRDgOE

---

All parts available via Digikey or AliExpress

ESP32, MPU6050, Nidec 24H brushless motors, 500 mAh LiPo battery.

About schematic:

Battery: 3S1P LiPo (11.1V). 
Buzzer: any 5V active buzzer.
Voltage regulator: any 5V regulator (7805).
All red connections not nescesary for this project! But if you are designing a PCB I recommend making these connections. Maybe I use encoders in the future, you will be able to use the new firmware without any changes.
 
How to build:

https://youtu.be/AJQZFHJzwt4

If something doesn't work, try the motors test sketch. It tests all motors, rotation directions and speeds. This helps you understand the problem is in software or in hardware.

ESP32 version also has an updated balancing point setting procedure. Important! In this video you can learn how to set the balancing points:

https://youtu.be/Nkm9PoihZOI


