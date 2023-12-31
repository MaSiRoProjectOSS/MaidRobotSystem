# Skinの作り方

## env.shによる設定

Cast用のフォルダを作り、settings.jsonを指定してください。
このsettings.jsonのパスは、環境変数MRS_CAST_DATAに設定してください。

env.shによる設定を推奨してます。「/opt/MaidRobotSystem/data/config.json」の"MRS_CAST_NAME"にCast名を設定してください。

```bash
export MRS_CONFIG=/opt/MaidRobotSystem/data/config.json
source /opt/MaidRobotSystem/src/planetary_module/ros/env.sh
# export MRS_CAST_NAME=Iris
# export MRS_CAST_DATA=/opt/MaidRobotSystem/data/cast/${MRS_CAST_NAME}/settings.json
```


## settings.jsonの設定

* pathには、Cast用のフォルダを指定してください。
* eyeには、目の画像のパスを指定してください。
  * eyeballには、目玉の画像のpathからのパスを指定してください。
  * eyelidには、まぶたの画像のpathからのパスを指定してください。
  * corneaには、瞳の画像のpathからのパスを指定してください。

```json
{
  "path": "/opt/MaidRobotSystem/data/cast/Iris",
  "eye": {
    "eyeball": {
        "normally": "eye/eyeball/eyeball_normally.png"
    },
    "eyelid": {
        "normally": [
            "eye/eyelid/normally/000.png",
            "eye/eyelid/normally/001.png",
            "eye/eyelid/normally/002.png"
        ]
    },
    "cornea": {
        "normally": "eye/cornea/cornea_normally.png"
    }
  }
}
```



## eyeの色変更

コマンド```ros2 param list```からパラメータリストを確認できて、```ros2 param set```でパラメータを変更できます。


```bash
# ノード名
WORK_NODE_NAME=/maid_robot_system/Ayame/head_unit/device/eye_node

# 全体
WORK_VALUE=100
ros2 param set ${WORK_NODE_NAME} ciliary/color/r ${WORK_VALUE}
ros2 param set ${WORK_NODE_NAME} ciliary/color/g ${WORK_VALUE}
ros2 param set ${WORK_NODE_NAME} ciliary/color/b ${WORK_VALUE}

# 白目部分
WORK_VALUE=255
ros2 param set ${WORK_NODE_NAME} eyelid/color/r ${WORK_VALUE}
ros2 param set ${WORK_NODE_NAME} eyelid/color/g ${WORK_VALUE}
ros2 param set ${WORK_NODE_NAME} eyelid/color/b ${WORK_VALUE}

# パラメータ全部を確認したい場合は、下記のコマンドを用意してます。
aros2_param_list

```
