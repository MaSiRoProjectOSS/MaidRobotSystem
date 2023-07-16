# MaSiRo Project.
#     This message notify data
#          from "Receptionist node" to "Dispatcher node"
#     Date : 2022/06/05
####################################################
#---------------------------------------------------
# list : notify
#---------------------------------------------------
##
## notify : QR code
## notify : IoT bell
## notify : GATE
##
##
uint32 NOTIFY_ID_NONE            = 0
uint32 NOTIFY_ID_QR              = 1
uint32 NOTIFY_ID_IOT_BELL        = 2
uint32 NOTIFY_ID_GATE            = 4
uint32 NOTIFY_ID_DICE            = 5

uint32 NOTIFY_ID_NONE_2          = 8
#---------------------------------------------------

std_msgs/UInt32     seq

std_msgs/UInt32     notify_id
std_msgs/UInt32     gate_state

std_msgs/UInt32     bell_event
std_msgs/UInt32     reserve_no
std_msgs/UInt32     seat_no

std_msgs/UInt32     battery
