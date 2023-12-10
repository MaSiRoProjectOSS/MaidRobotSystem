# MaidRobotSystem

## Head unit




```plantuml
@startuml
title ROS NODE on Head unit
scale max 1024 width
top to bottom direction

!$topic_hv_left_text = "**/holistic_view/left"
!$topic_hv_right_text = "**/holistic_view/right"
!$topic_eye_control_text = "**/head_control/eye_control"
!$topic_neck_control_text = "**/head_control/neck_control"

!$msg_pose_data = "maid_robot_system_interfaces/msg/PoseData"
!$msg_eye_gaze = "maid_robot_system_interfaces/msg/EyeGaze"
!$msg_neck_angle = "maid_robot_system_interfaces/msg/NeckAngle"


rectangle head_unit as " " {

  storage recognition_in_node_left as "recognition_in_node [left]"  #Physical  {
      rectangle TOPIC_CAMERA_RAW_LEFT [
      "**/camera/raw/left"
      ----
      "**/image"
      ]
      rectangle TOPIC_CAMERA_WRITE_LEFT[
      "**/camera/recognition/left"
      ----
      "**/image"
      ]
      rectangle TOPIC_JV_LEFT [
      $topic_hv_left_text
      ----
      $msg_pose_data
      ]
  }
  storage recognition_in_node_right as "recognition_in_node [right]"  #Physical  {
      rectangle TOPIC_CAMERA_RAW_RIGHT [
      "**/camera/raw/right"
      ----
      "**/image"
      ]
      rectangle TOPIC_CAMERA_WRITE_RIGHT[
      "**/camera/recognition/right"
      ----
      "**/image"
      ]
      rectangle TOPIC_JV_RIGHT [
      $topic_hv_right_text
      ----
      $msg_pose_data
      ]
  }
TOPIC_CAMERA_RAW_LEFT -down-> TOPIC_CAMERA_WRITE_LEFT
TOPIC_CAMERA_RAW_RIGHT -down-> TOPIC_CAMERA_WRITE_RIGHT
TOPIC_CAMERA_WRITE_LEFT -down-> TOPIC_JV_LEFT
TOPIC_CAMERA_WRITE_RIGHT -down-> TOPIC_JV_RIGHT


TOPIC_CAMERA_RAW_LEFT -[hidden]right- TOPIC_CAMERA_RAW_RIGHT
TOPIC_CAMERA_WRITE_LEFT -[hidden]right- TOPIC_CAMERA_WRITE_RIGHT
TOPIC_JV_LEFT -[hidden]right- TOPIC_JV_RIGHT

  storage head_control_calc_node        #Application {
    portin head_control_port_in_01 as " "
    portin head_control_port_in_02 as " "
    rectangle TOPIC_EYE [
    $topic_eye_control_text
    ----
    $msg_eye_gaze
    ]
    rectangle TOPIC_NECK [
    $topic_neck_control_text
    ----
    $msg_neck_angle
    ]


  }
  storage eye_op_node            #Physical
  storage neck_op_node              #Physical
}

TOPIC_JV_LEFT ---(  head_control_port_in_01
TOPIC_JV_RIGHT ---( head_control_port_in_02

TOPIC_EYE --(   eye_op_node
TOPIC_NECK --(  neck_op_node

TOPIC_EYE -[hidden]down- TOPIC_NECK

@enduml
```






---


### head_control_node

### neck_node


### face_recognition_node

USB-Webカメラより画像を取得し、姿勢制御情報を通達する。


```plantuml
@startuml
scale max 1024 width

title face_recognition_node
!include <tupadr3/devicons/python.puml>
!include <material/folder.puml>

salt
{{T
    + <$python> recognition_in_node.py      | recognition_in_nodeの本体
     ++ <$ma_folder> utils
     +++ <$python> usb_video_device.py        | USB-WebカメラのデバイスIDを取得する
     +++ <$python> cv_fps_calc.py             | FPSの計算を行う
     ++ <$ma_folder>features
     +++ <$ma_folder>ros
     ++++ <$python> face_recognition_ros.py   | recognition_in_nodeのROSインターフェース
     +++ <$ma_folder>mp
     ++++ <$python> imageanalysis.py          | mediapipeの画像解析
     ++++ <$python> poseinformation.py        | mediapipeの姿勢情報
     ++++ <$python> schematicdiagram.py       | 画像データに情報を書き込む
  }
}

@enduml
```

### Todo

計算処理はこのノードから切り離すべき？



## ノード名規則

<機能名>_<type>_node

| type   |                                                |     |
| ------ | ---------------------------------------------- | --- |
| calc   | 入力に従って演算するノードタイプ               |     |
| op     | 受け付けた処理にしたがって動作するノードタイプ |     |
| in     | 入力装置からの処理をROSに変換するノードタイプ  |     |
| notify | 外部へ通達するノードタイプ                     |     |


