## 設計思想

PC側で動いているプログラムは統括的な制御を行うことに集中して欲しく、
モータードライバの制御はプロトコルレベルまで抽象化して、PC側のプログラムが気にすることが少ないようにする。

モータードライバの制御がシリアルだったりCANだったりインターフェイスが異なるための、USB（シリアル）で接続されたM5Atomというデバイスを介する設計とする。

このことにより、PC側はUSB(シリアル)とプロトコルのみを、
M5Atom側は上記プロトコルを受け取ってモータードライバの制御を行うことに集中できるようにして、双方のに影響を与えないようにする。


```plantuml
@startuml
top to bottom direction

rectangle PC

rectangle TARGET as " " {
    rectangle M5Atom
    note "PCの代わりに\nダメージが行かないようにするための\nドライバ" as N2
    (M5Atom) --  N2

    rectangle RS485_Module as "RS485 モジュール"
    rectangle CAN_Module as "CAN モジュール"
        rectangle ZLAC8015D {
            portin CAN
            portin RS485
        }
        circle Motor1
        circle Motor2
}

PC -- M5Atom : USB
M5Atom -- CAN_Module : UART
M5Atom -- RS485_Module : UART
CAN_Module -- CAN : 終端抵抗-必要
RS485_Module -- RS485 : 終端抵抗-必要

ZLAC8015D -- Motor1
ZLAC8015D -- Motor2

@enduml
```


