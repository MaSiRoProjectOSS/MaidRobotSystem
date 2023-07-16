# MaSiRo Project.
#     Predictive Failure Detection message
#           現在の状態を通知する仕組み
#     File : CastResume.msg
#     Date : 2022/05/16
####################################################
#---------------------------------------------------
# list : cast name
#---------------------------------------------------
string CAST_NAME_EVERYONE          = "--"
string CAST_NAME_MASIRO            = "Masiro"
string CAST_NAME_CIRO              = "Ciro"
string CAST_NAME_CIYA              = "Ciya"

####################################################
# cast id
#---------------------------------------------------
uint8 CAST_ID_EVERYONE  = 0
uint8 CAST_ID_MASIRO    = 100
uint8 CAST_ID_CIRO      = 101
uint8 CAST_ID_CIYA      = 102
#------
std_msgs/UInt8      cast_id
#------

####################################################
# header
#---------------------------------------------------
std_msgs/UInt16     frame_no
std_msgs/UInt32     record_id

####################################################
# vital_signs
#---------------------------------------------------
std_msgs/Float32    vital_signs

####################################################
# seat_no
#---------------------------------------------------
std_msgs/UInt16     seat_no

####################################################
# action_id
#---------------------------------------------------
std_msgs/UInt32     action_id

####################################################
# wheel_switch
#---------------------------------------------------
# コマンドベロティのどれをホイールに発行する
uint8 WHEEL_SWITCH_STOP             =  0    ##  0：停止
uint8 WHEEL_SWITCH_NAVIGATION       =  1    ##  1：ナビゲーション操作
uint8 WHEEL_SWITCH_HAND_HOLDING     =  2    ##  2：手つなぎ
uint8 WHEEL_SWITCH_OTHER            =  3    ##  3：それ以外（ポーズなど
uint8 WHEEL_SWITCH_REMOTE_CONTROL   = 10    ## 10：遠隔操作
#------
std_msgs/UInt32     wheel_switch
#------

####################################################
# task_id
#---------------------------------------------------
std_msgs/UInt32     task_id
