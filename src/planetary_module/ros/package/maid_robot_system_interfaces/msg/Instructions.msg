# MaSiRo Project.
#     Order from Receptionist message
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
# receptionist_id (task_id)
#---------------------------------------------------
std_msgs/UInt32     receptionist_id

####################################################
# seat_no
#---------------------------------------------------
std_msgs/UInt32     seat_no

####################################################
# address
#---------------------------------------------------
std_msgs/Int32      address

####################################################
# action_id
#---------------------------------------------------
#std_msgs/UInt32     action_id
