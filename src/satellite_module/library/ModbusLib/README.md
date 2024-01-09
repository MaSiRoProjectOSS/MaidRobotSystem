# Modbus Lib

Modbusプロトコルで使うメッセージフレームを作成するライブラリです。


## Project Status

<div style="display: flex">
    <div style="width:1em; background-color: red;margin-right:1em;"></div>
    <span style="">This project will not be refurbished unless requested.</span>
</div>

## Description

Serial通信を行うためArduinoをメインターゲットにしています。下記のモードをサポートしています。

| Mode         | Support     | Notes                                                                                   |
| ------------ | ----------- | --------------------------------------------------------------------------------------- |
| RTU mode     | support     | --                                                                                      |
| RTU(EX) mode | support     | Enlarge the message frame.The first line of the data frame contains the number of data. |
| ASCII mode   | support     | --                                                                                      |
| TCP mode     | NOT support | --                                                                                      |


## Installation

This system is compiled using PlatformIO. Please install the extension from VS Code's extension recommendations. If you want to know more, check [here](https://docs.platformio.org/en/latest/).

* platformio.ini

```ini
[env:m5stack_atom]
platform = espressif32
board = m5stack-atom
framework = arduino
lib_deps =
	m5stack/M5Atom@^0.1.0
	fastled/FastLED@^3.6.0
	(This Library path)
build_flags =
	-D ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE=(8192*4)
```

## Notes

* 'ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE'のサイズを変更する必要がある場合があります。

```ini
build_flags =
	-D ARDUINO_SERIAL_EVENT_TASK_STACK_SIZE=(8192*4)
```

* 動作検証
  * M5Atom Liteで動作確認を行っています。

## Requirement

This system uses the following libraries.

* M5Atom
  * [m5stack/M5Atom](https://github.com/m5stack/M5Atom?utm_source=platformio&utm_medium=piohome)
  * [fastled/FastLED](https://github.com/Makuna/NeoPixelBus?utm_source=platformio&utm_medium=piohome)

## Changelog

It is listed [here](./Changelog.md).



## Roadmap

- 重要
  - [ ] CRC/LRCの計算が正しく行えているか他システムと通信し検証を行う。
  - [ ] RTU mode のエラー時発生後、正常に通信が行えるか検証を行う。
    - RTU mode のStartフレーム（3.5文字の無通信）の検証を行う。

- その他
  - [ ] Modbus TCPをサポートするかの検討
    - Arduinoをメインターゲットにしているため、サポートするかの検討が必要
  - [ ] バグの修正
  - [ ] StackSizeの調整
  - [ ] ドキュメントの整備

## Contributing

We welcome pull requests from the community. If you're considering significant changes, we kindly ask you to begin by opening an issue to initiate a discussion about your proposed modifications. Additionally, when submitting a pull request, please ensure that any relevant tests are updated or added as needed.

## Authors and acknowledgment

We offer heartfelt thanks to the open-source community for the invaluable gifts they've shared with us. The hardware, libraries, and tools they've provided have breathed life into our journey of development. Each line of code and innovation has woven a tapestry of brilliance, lighting our path. In this symphony of ingenuity, we find ourselves humbled and inspired. These offerings infuse our project with boundless possibilities. As we create, they guide us like stars, reminding us that collaboration can turn dreams into reality. With deep appreciation, we honor the open-source universe that nurtures us on this journey of discovery and growth.

## License

[MIT License](./LICENSE)


<div style="display:none">

---

<style>
body {
    counter-reset: chapter;
}

h2 {
    counter-reset: sub-chapter;
}

h3 {
    counter-reset: section;
}

h4 {
    counter-reset: indexlist;
}

h1::before {
    counter-reset: chapter;
}

h2::before {
    counter-increment: chapter;
    content: counter(chapter) ". ";
}

h3::before {
    counter-increment: sub-chapter;
    content: counter(chapter) "-" counter(sub-chapter) ". ";
}

h4::before {
    counter-increment: section;
    content: counter(chapter) "-" counter(sub-chapter) "-" counter(section) ". ";
}

h5::before {
    counter-increment: indexlist;
    content: "(" counter(indexlist) ") ";
}

#sidebar-toc-btn {
    bottom: unset;
    top: 8px;
}

.markdown-preview.markdown-preview {
    h2 {
        border-bottom: 4px solid #eaecef;
    }

    h3 {
        border-bottom: 1px solid #eaecef;
    }
}

.md-sidebar-toc.md-sidebar-toc {
    padding-top: 40px;
}

#sidebar-toc-btn {
    bottom: unset;
    top: 8px;
}
</style>

</div>
