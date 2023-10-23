# Maid Robot System Library

"Maid Robot System"で使用する汎用ライブラリ

## Project Status

必要に応じて追加

## Description

* move_average
  * 移動平均を扱うクラス
* time_check
  * 経過時間を管理するためのストップウォッチクラス
* log_callback
  * デバッグ表示を制御するためのクラス
* types/coordinate_euler
  * pitch/roll/yawでデータを管理するための構造体
* types/pid_struct
  * PIDで使う構造体
* types/pose_2d
  * poseデータで使う構造体
* interface/mode_list
  * モード一覧
* interface/control_if
  * MVCモデルのコントローラで使う親クラス
* interface/model_if
  * MVCモデルのモデルで使う親クラス
* chart/candle_stick
  * ローソク足グラフに使うデータ保管クラス


## Installation

iniファイル```platformio.ini```の```lib_deps```に本プロジェクトを追加してください。

```ini
[env]
lib_deps =
    https://github.com/MasterAkari/STAB_Arduino
```

## Notes


## Support


## Roadmap


## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## Authors and acknowledgment

Show your appreciation to those who have contributed to the project.

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
