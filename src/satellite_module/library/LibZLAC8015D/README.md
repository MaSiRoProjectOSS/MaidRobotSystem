# LibZLAC8015D

ZLAC8015Dを制御するためのライブラリです。

## Project Status

<div style="display: flex">
    <div style="width:1em; background-color: red;margin-right:1em;"></div>
    <span style="">This project will not be refurbished unless requested.</span>
</div>

## Installation

This system is compiled using PlatformIO. Please install the extension from VS Code's extension recommendations. If you want to know more, check [here](https://docs.platformio.org/en/latest/).

* platformio.ini

```ini
[env:m5stack_atom]
platform = espressif32
framework = arduino
board = m5stack-atom
lib_deps =
  ${env.lib_deps}
  m5stack/M5Atom@^0.1.0
  fastled/FastLED@^3.6.0
  ../ModbusLib
build_flags =
  -D MODBUS_ADDRESS=0x00
  -D MODBUS_TARGET_ADDRESS=0x01
  -D CORE_DEBUG_LEVEL=-1
  -D TX1=G26
  -D RX1=G32
```

## Notes

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

- TODO
  - [ ] モードごとにバラバラに呼び出している関数をまとめたい
    - データシートに従って関数を追加しているため

```c*++
bool set_mode_position_absolute(){
    this->set_control_word(ZLAC8015DCtrl::ZLAC_CONTROL_WORD::CONTROL_WORD_UNDEFINED);
    this->set_synchronous_control_status(false);
    this->set_control_mode(ZLAC::DRIVER_MODE::POSITION_ABSOLUTE);
    this->set_s_shape_acceleration_time(500, 500);
    this->set_s_shape_deceleration_time(500, 500);
    this->set_target_position(0, 0);
    this->set_max_speed(60, 60);
    return true;
}
```

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
    counter-reset: index-list;
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
    counter-increment: index-list;
    content: "(" counter(index-list) ") ";
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
