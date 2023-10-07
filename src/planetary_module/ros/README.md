# ros

* package
* script
  * env.shを実行すると、MaidRobotSystemで扱っているROS2の環境変数を設定します。
* vs_tasks
  * これはVisual Studio Code で、タスクを実行するための設定ファイルです。
VS codeの拡張機能「[Task Runner](https://marketplace.visualstudio.com/items?itemName=actboy168.tasks)」をインストールしてください。
このフォルダを、VS codeのワークスペースに追加してください。
VS codeのワークスペースの設定ファイルに、以下のように追記してください。

```json
{
  "folders": [
    {
      "name": "ros2_help",
      "path": "/opt/MaidRobotSystem/src/planetary_module/ros/vs_tasks/ros2_help"
    },
    ...
}
```


## メモ

Ubuntu 20.04だと、mediapipeのバージョンは" 0.8.9.1"となりprotobufのバージョンは"3.20.*"となる。
そのため、下記のバージョンにしておく必要がある。

```bash
pip install protobuf==3.20.*
```

### ビルドに必要なソフトウェア

```bash
sudo apt install -y build-essential libfontconfig1 mesa-common-dev libglu1-mesa-dev qt*5-dev qml-module-qtquick-controls qml-module-qtquick-controls2
sudo apt install -y install nlohmann-json3-dev
```
