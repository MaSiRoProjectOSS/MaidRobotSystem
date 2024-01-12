# neck_controller

## Modbus data

|             |                              |     |
| ----------- | ---------------------------- | --- |
| Address     |                              |     |
| Function    |                              |     |
| Data-length |                              |     |
| Data        | Num(H)                       |     |
| ^           | Num(L)                       |     |
| ^           | 上記のアドレスから何個とるか |     |
| Error check |                              |     |



## レジスタ

| Num    | Device type | --      | Note |
| ------ | ----------- | ------- | ---- |
| 0x6810 | IMU         | accel.x | --   |
| 0x6811 | IMU         | accel.y | --   |
| 0x6812 | IMU         | accel.z | --   |
| 0x6813 | IMU         | gyro.x  | --   |
| 0x6814 | IMU         | gyro.y  | --   |
| 0x6815 | IMU         | gyro.z  | --   |

| Num    | Device type              | --                      | Note          |
| ------ | ------------------------ | ----------------------- | ------------- |
| 0x7000 | PWM Motor(I2C) - Setting | Oscillator Frequency(H) |               |
| 0x7001 | PWM Motor(I2C) - Setting | Oscillator Frequency(L) |               |
| 0x7002 | PWM Motor(I2C) - Setting | PWM Frequency(H)        |               |
| 0x7003 | PWM Motor(I2C) - Setting | PWM Frequency(L)        |               |
| 0x7100 | PWM Motor(I2C) - ALL     | --                      | for Broadcast |
| 0x7101 | PWM Motor(I2C) - No.01   | --                      |               |
| 0x7102 | PWM Motor(I2C) - No.02   | --                      |               |
| 0x7103 | PWM Motor(I2C) - No.03   | --                      |               |
| 0x7104 | PWM Motor(I2C) - No.04   | --                      |               |
| 0x7105 | PWM Motor(I2C) - No.05   | --                      |               |






