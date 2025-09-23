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


