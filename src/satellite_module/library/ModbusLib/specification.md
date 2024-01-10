# Specification

## Message frame

* ASCII mode

| Function type | byte          | Range        | Note |
| ------------- | ------------- | ------------ | ---- |
| Address       | 2byte         | '00' ～ 'F7' | --   |
| Function code | 2byte         | '00' ～ '79' | --   |
| Data          | 0 ～ 243 byte | ...          | --   |
| Error check   | 2byte         | '00' ～ 'FF  | LRC  |


* RTU mode

| Function type | byte          | Range            | Note   |
| ------------- | ------------- | ---------------- | ------ |
| Address       | 1byte         | 0x00 ～ 0xF7     | --     |
| Function code | 1byte         | 0x00 ～ 0x79     | --     |
| Data          | 0 ～ 243 byte | ...              | --     |
| Error check   | 2byte         | 0x0000 ～ 0xFFFF | CRC-16 |

* RTU(EX) mode

| Function type | byte          | Range            | Note   |
| ------------- | ------------- | ---------------- | ------ |
| Address       | 1byte         | 0x00 ～ 0xF7     | --     |
| Function code | 1byte         | 0x00 ～ 0x79     | --     |
| Data Length   | 1byte         | 0x00 ～ 0xFF     | --     |
| Data          | 0 ～ 243 byte | ...              | --     |
| Error check   | 2byte         | 0x0000 ～ 0xFFFF | CRC-16 |

* TCP mode
  * 調査中

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




## Exception Codes

| Code | Name                                    | Meaning                                                                                                                                                                                                                     |
| ---- | --------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0x01 | ILLEGAL FUNCTION                        | The server can't process the received function code.                                                                                                                                                                        |
| 0x02 | ILLEGAL DATA ADDRESS                    | The server rejects queries with invalid data addresses.                                                                                                                                                                     |
| 0x03 | ILLEGAL DATA VALUE                      | The server rejects queries with invalid data values, indicating a structural fault in the request, like incorrect implied length.                                                                                           |
| 0x04 | SERVER DEVICE FAILURE                   | The server encountered a fatal error while trying to execute a task.                                                                                                                                                        |
| 0x05 | ACKNOWLEDGE                             | The server is processing a programming command which takes a long time, and sends a response to prevent client timeout. The client can check if processing is done by sending a Poll Program Complete message.              |
| 0x06 | SERVER DEVICE BUSY                      | The server is busy processing a long-lasting programming command. The client should resend the message when the server is available.                                                                                        |
| 0x08 | MEMORY PARITY ERROR                     | With function codes 20 and 21 and reference type 6, the server detected a parity error in the memory while reading a file, indicating a consistency check failure. The client can retry, but the server may need servicing. |
| 0x0A | GATEWAY PATH UNAVAILABLE                | The gateway couldn't allocate an internal communication path for request processing, typically indicating misconfiguration or overload.                                                                                     |
| 0x0B | GATEWAY TARGET DEVICE FAILED TO RESPOND | In the context of gateways, no response from the target device usually indicates its absence from the network.                                                                                                              |


