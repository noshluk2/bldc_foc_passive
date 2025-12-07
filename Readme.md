### Features
- Overheating
- Pinned Wires
- Different power buses
- MicroController serial logging while powered on
- Current limiting and current sensing
- Heat decipation

### Problems
- Wires are alot of the motors , around each motor we have 9 wires so we are bringin in CAN communication to limit wires to only 4 ( 12V bus and Can Bus) with 2 of each. To pass through Hollow shaft.
- Holding torque with zero motion is always going to produce heat. As current have to convert to something else if not motion.
    - Al the very least we can add current sensing + shunts to measure current going in and hard cut that with a limit based on temperaure sensor on motor.
    - Link : https://chatgpt.com/share/68c27ee4-58ac-8005-99c9-39c60b1bd00c


### Tasks
[] - Variable Torque Resistance in motor
[] - Motor Attaing a positiong
[] - Reading Encoders precision
[] - Force Sensing
[] - Variable resistance upon force.


- Understanding thoery
    - kv , votlage, speed passive

-----
### Current Setup
- 4 gimbal axes (3× CubeMars GL30, 1× GL40)
- Each axis: Arduino Pro Mini + SimpleFOC, AS5048A (SPI) encoder
- One 12 V power bus and CAN (2-wire) daisy-chained through the hollow shafts


#### Future additions
- Add fuse + TVS + ≥470 µF at each joint’s 12 V entry.
- Confirm Pro Mini is 5 V (best with AS5048A). If 3.3 V, add SPI level shifting.
- Route E-STOP to the driver ENABLE so torque is killed in hardware; keep MCU + CAN powered.
- Place buck + logic away from the MOSFET switching node; keep encoder wires short and shielded.
- CAN: two terminators only; short stubs; twisted pair.
### Problems
1. Motors heating up on resistance :
    - Cause : Constant power supplied - need to limit unneccasry power applied to coils
    - Solution : Current limiting
1. Wire managment : cannot make all encoders + 12V buss wires throuh hollow shaft
    - solution researched : can bus


#### New Board Testing
- Udp Data Sending
    - /home/luqman/bldc_foc_passive/test/wifi/udp_test_sender.cpp -> ESP32
    - /home/luqman/bldc_foc_passive/test/wifi/udp_test_server.py -> PC

- Encoders testing
    - /home/luqman/bldc_foc_passive/test/encoders/read_encoders_udp.cpp
- Motor Driving testing
    - /home/luqman/bldc_foc_passive/test/motor/simple_drive.cpp