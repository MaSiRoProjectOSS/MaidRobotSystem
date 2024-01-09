# Specification


## Function code

| Function type |               |                                                 | Function name                    | Function code | Comment     |
| ------------- | ------------- | ----------------------------------------------- | -------------------------------- | :-----------: | ----------- |
| Data Access   | Bit access    | Physical Discrete Inputs                        | Read Discrete Inputs             |    2(0x02)    | --          |
| ^             | ^             | Internal Bits or Physical Coils                 | Read Coils                       |    1(0x01)    | --          |
| ^             | ^             | ^                                               | Write Single Coil                |    5(0x05)    | --          |
| ^             | ^             | ^                                               | Write Multiple Coils             |   15(0x0F)    | --          |
| ^             | 16-bit access | Physical Input Registers                        | Read Input Registers             |    4(0x04)    | --          |
| ^             | ^             | Internal Registers or Physical Output Registers | Read Multiple Holding Registers  |    3(0x03)    | --          |
| ^             | ^             | ^                                               | Write Single Holding Register    |    6(0x06)    | --          |
| ^             | ^             | ^                                               | Write Multiple Holding Registers |   16(0x10)    | --          |
| ^             | ^             | ^                                               | Read/Write Multiple Registers    |   23(0x17)    | --          |
| ^             | ^             | ^                                               | Mask Write Register              |   22(0x16)    | --          |
| ^             | ^             | ^                                               | Read FIFO Queue                  |   24(0x18)    | --          |
| ^             | ^             | File Record Access                              | Read File Record                 |   20(0x14)    | --          |
| ^             | ^             | ^                                               | Write File Record                |   21(0x15)    | --          |
| Diagnostics   |               |                                                 | Read Exception Status            |    7(0x07)    | serial only |
| ^             | ^             | ^                                               | Diagnostic                       |    8(0x08)    | serial only |
| ^             | ^             | ^                                               | Get Com Event Counter            |   11(0x0B)    | serial only |
| ^             | ^             | ^                                               | Get Com Event Log                |   12(0x0C)    | serial only |
| ^             | ^             | ^                                               | Report Server ID                 |   17(0x11)    | serial only |
| ^             | ^             | ^                                               | Read Device Identification       |   43(0x2B)    | --          |
| Other         |               |                                                 | Encapsulated Interface Transport |   43(0x2B)    | --          |


## 無応答

* 下記の条件の時は、応答を返さない
  * スレーブアドレスが自身のアドレスではない
  * ブロードキャスト
    * 受信した処理を行うが、応答は行わない。


