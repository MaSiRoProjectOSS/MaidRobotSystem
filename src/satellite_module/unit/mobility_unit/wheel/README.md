# README

## ちろちやの車輪用マイコン

| 項目     | 説明                                 |
| -------- | ------------------------------------ |
| 構築環境 | AduinoIDE（変更予定）                |
| OS       | Arduino                              |
| ボード   | Arduino mega                         |
| フォルダ | CIYA_steper_and_wheel-megaatmega2560 |

## ちろ/ちやの切り替え

「ちろ」と「ちや」は、使える腕が反対になっています。
プログラムの書き換えは```platformio.ini```のenvで切り替えを行っています。

## CAN DATA

|   S/R   | CAN_ID |  Size | Index | Range | UNIT   | Module | DATA            |
| :-----: | -----: | ----: | ----: | :---: | :----- | :----: | :-------------- |
| Receive |  0x312 | 1以上 |     0 | N * 4 | [deg?] | Waist  | request pitch   |
|         |        |       |       |       |        |        |                 |
| Receive |  0x310 |     2 |     0 |   N   | bool   | Wheel  | Enable/Disable  |
| Receive |  0x310 |     2 |     1 |   N   | bool   |  Leg   | Enable/Disable  |
|         |        |       |       |       |        |        |                 |
| Receive |  0x311 |     2 |   0-1 |   N   | [deg?] |   ？   | 傾ける角度？    |
|         |        |       |       |       |        |        |                 |
| Receive |  0x333 |     4 |   0-1 |   N   | ?      |        | target position |
| Receive |  0x333 |     4 |   2-3 |   N   | ?      |        | target speed    |
|         |        |       |       |       |        |        |                 |
| Receive |  0x315 |     5 |     0 |   N   | [mode] | Wheel  | control mode    |
| Receive |  0x315 |     5 |   1-2 |   N   | [?]    | Wheel  | right motor     |
| Receive |  0x315 |     5 |   3-4 |   N   | [?]    | Wheel  | left motor      |


|  S/R  | CAN_ID | Size | Index |  Range  | UNIT  | Module | DATA                  |
| :---: | -----: | ---: | ----: | :-----: | :---- | :----: | :-------------------- |
| Send  |  0x321 |    6 |   0-1 | N * 10  | [mG?] |  Gyro  | Gyro yaw              |
| Send  |  0x321 |    6 |   2-3 | N * 10  | [mG?] |  Gyro  | Gyro pitch            |
| Send  |  0x321 |    6 |   4-6 | N * 10  | [mG?] |  Gyro  | Gyro roll             |
|       |        |      |       |         |       |        |                       |
| Send  |  0x322 |    6 |   0-1 |  N * 1  | [%]   |  Leg   | Leg height percentage |
| Send  |  0x322 |    6 |   2-3 | N * 100 | [mG?] | Waist  | waist pitch           |
| Send  |  0x322 |    6 |   4-6 |  N * 1  | [%]   |  Leg   | Leg step percentage   |
