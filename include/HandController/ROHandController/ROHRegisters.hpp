#ifndef ROH_REGISTERS_HPP
#define ROH_REGISTERS_HPP

// ModBus-RTU registers for ROH

#define MODBUS_PROTOCOL_VERSION_MAJOR 1

#define ROH_PROTOCOL_VERSION       1000   // R
#define ROH_FW_VERSION             1001   // R
#define ROH_FW_REVISION            1002   // R
#define ROH_HW_VERSION             1003   // R
#define ROH_BOOT_VERSION           1004   // R
#define ROH_NODE_ID                1005   // R/W
#define ROH_SUB_EXCEPTION          1006   // R
#define ROH_BATTERY_VOLTAGE        1007   // R
#define ROH_SELF_TEST_LEVEL        1008   // R/W
#define ROH_BEEP_SWITCH            1009   // R/W
#define ROH_BEEP_PERIOD            1010   // W
#define ROH_BUTTON_PRESS_CNT       1011   // R/W
#define ROH_RECALIBRATE            1012   // W
#define ROH_START_INIT             1013   // W
#define ROH_RESET                  1014   // W
#define ROH_POWER_OFF              1015   // W
#define ROH_RESERVED0              1016   // R/W
#define ROH_RESERVED1              1017   // R/W
#define ROH_RESERVED2              1018   // R/W
#define ROH_RESERVED3              1019   // R/W

#define ROH_CALI_END0              1020
#define ROH_CALI_END1              1021
#define ROH_CALI_END2              1022
#define ROH_CALI_END3              1023
#define ROH_CALI_END4              1024
#define ROH_CALI_END5              1025
#define ROH_CALI_END6              1026
#define ROH_CALI_END7              1027
#define ROH_CALI_END8              1028
#define ROH_CALI_END9              1029

#define ROH_CALI_START0            1030
#define ROH_CALI_START1            1031
#define ROH_CALI_START2            1032
#define ROH_CALI_START3            1033
#define ROH_CALI_START4            1034
#define ROH_CALI_START5            1035
#define ROH_CALI_START6            1036
#define ROH_CALI_START7            1037
#define ROH_CALI_START8            1038
#define ROH_CALI_START9            1039

#define ROH_CALI_THUMB_POS0        1040
#define ROH_CALI_THUMB_POS1        1041
#define ROH_CALI_THUMB_POS2        1042
#define ROH_CALI_THUMB_POS3        1043
#define ROH_CALI_THUMB_POS4        1044

#define ROH_FINGER_P0              1045
#define ROH_FINGER_P1              1046
#define ROH_FINGER_P2              1047
#define ROH_FINGER_P3              1048
#define ROH_FINGER_P4              1049
#define ROH_FINGER_P5              1050
#define ROH_FINGER_P6              1051
#define ROH_FINGER_P7              1052
#define ROH_FINGER_P8              1053
#define ROH_FINGER_P9              1054

#define ROH_FINGER_I0              1055
#define ROH_FINGER_I1              1056
#define ROH_FINGER_I2              1057
#define ROH_FINGER_I3              1058
#define ROH_FINGER_I4              1059
#define ROH_FINGER_I5              1060
#define ROH_FINGER_I6              1061
#define ROH_FINGER_I7              1062
#define ROH_FINGER_I8              1063
#define ROH_FINGER_I9              1064

#define ROH_FINGER_D0              1065
#define ROH_FINGER_D1              1066
#define ROH_FINGER_D2              1067
#define ROH_FINGER_D3              1068
#define ROH_FINGER_D4              1069
#define ROH_FINGER_D5              1070
#define ROH_FINGER_D6              1071
#define ROH_FINGER_D7              1072
#define ROH_FINGER_D8              1073
#define ROH_FINGER_D9              1074

#define ROH_FINGER_G0              1075
#define ROH_FINGER_G1              1076
#define ROH_FINGER_G2              1077
#define ROH_FINGER_G3              1078
#define ROH_FINGER_G4              1079
#define ROH_FINGER_G5              1080
#define ROH_FINGER_G6              1081
#define ROH_FINGER_G7              1082
#define ROH_FINGER_G8              1083
#define ROH_FINGER_G9              1084

#define ROH_FINGER_STATUS0         1085
#define ROH_FINGER_STATUS1         1086
#define ROH_FINGER_STATUS2         1087
#define ROH_FINGER_STATUS3         1088
#define ROH_FINGER_STATUS4         1089
#define ROH_FINGER_STATUS5         1090
#define ROH_FINGER_STATUS6         1091
#define ROH_FINGER_STATUS7         1092
#define ROH_FINGER_STATUS8         1093
#define ROH_FINGER_STATUS9         1094

#define ROH_FINGER_CURRENT_LIMIT0  1095
#define ROH_FINGER_CURRENT_LIMIT1  1096
#define ROH_FINGER_CURRENT_LIMIT2  1097
#define ROH_FINGER_CURRENT_LIMIT3  1098
#define ROH_FINGER_CURRENT_LIMIT4  1099
#define ROH_FINGER_CURRENT_LIMIT5  1100
#define ROH_FINGER_CURRENT_LIMIT6  1101
#define ROH_FINGER_CURRENT_LIMIT7  1102
#define ROH_FINGER_CURRENT_LIMIT8  1103
#define ROH_FINGER_CURRENT_LIMIT9  1104

#define ROH_FINGER_CURRENT0        1105
#define ROH_FINGER_CURRENT1        1106
#define ROH_FINGER_CURRENT2        1107
#define ROH_FINGER_CURRENT3        1108
#define ROH_FINGER_CURRENT4        1109
#define ROH_FINGER_CURRENT5        1110
#define ROH_FINGER_CURRENT6        1111
#define ROH_FINGER_CURRENT7        1112
#define ROH_FINGER_CURRENT8        1113
#define ROH_FINGER_CURRENT9        1114

#define ROH_FINGER_FORCE_LIMIT0    1115
#define ROH_FINGER_FORCE_LIMIT1    1116
#define ROH_FINGER_FORCE_LIMIT2    1117
#define ROH_FINGER_FORCE_LIMIT3    1118
#define ROH_FINGER_FORCE_LIMIT4    1119

#define ROH_FINGER_FORCE0          1120
#define ROH_FINGER_FORCE1          1121
#define ROH_FINGER_FORCE2          1122
#define ROH_FINGER_FORCE3          1123
#define ROH_FINGER_FORCE4          1124

#define ROH_FINGER_SPEED0          1125
#define ROH_FINGER_SPEED1          1126
#define ROH_FINGER_SPEED2          1127
#define ROH_FINGER_SPEED3          1128
#define ROH_FINGER_SPEED4          1129
#define ROH_FINGER_SPEED5          1130
#define ROH_FINGER_SPEED6          1131
#define ROH_FINGER_SPEED7          1132
#define ROH_FINGER_SPEED8          1133
#define ROH_FINGER_SPEED9          1134

#define ROH_FINGER_POS_TARGET0     1135
#define ROH_FINGER_POS_TARGET1     1136
#define ROH_FINGER_POS_TARGET2     1137
#define ROH_FINGER_POS_TARGET3     1138
#define ROH_FINGER_POS_TARGET4     1139
#define ROH_FINGER_POS_TARGET5     1140
#define ROH_FINGER_POS_TARGET6     1141
#define ROH_FINGER_POS_TARGET7     1142
#define ROH_FINGER_POS_TARGET8     1143
#define ROH_FINGER_POS_TARGET9     1144

#define ROH_FINGER_POS0            1145
#define ROH_FINGER_POS1            1146
#define ROH_FINGER_POS2            1147
#define ROH_FINGER_POS3            1148
#define ROH_FINGER_POS4            1149
#define ROH_FINGER_POS5            1150
#define ROH_FINGER_POS6            1151
#define ROH_FINGER_POS7            1152
#define ROH_FINGER_POS8            1153
#define ROH_FINGER_POS9            1154

#define ROH_FINGER_ANGLE_TARGET0   1155
#define ROH_FINGER_ANGLE_TARGET1   1156
#define ROH_FINGER_ANGLE_TARGET2   1157
#define ROH_FINGER_ANGLE_TARGET3   1158
#define ROH_FINGER_ANGLE_TARGET4   1159
#define ROH_FINGER_ANGLE_TARGET5   1160
#define ROH_FINGER_ANGLE_TARGET6   1161
#define ROH_FINGER_ANGLE_TARGET7   1162
#define ROH_FINGER_ANGLE_TARGET8   1163
#define ROH_FINGER_ANGLE_TARGET9   1164

#define ROH_FINGER_ANGLE0          1165
#define ROH_FINGER_ANGLE1          1166
#define ROH_FINGER_ANGLE2          1167
#define ROH_FINGER_ANGLE3          1168
#define ROH_FINGER_ANGLE4          1169
#define ROH_FINGER_ANGLE5          1170
#define ROH_FINGER_ANGLE6          1171
#define ROH_FINGER_ANGLE7          1172
#define ROH_FINGER_ANGLE8          1173
#define ROH_FINGER_ANGLE9          1174

#endif // ROH_REGISTERS_HPP
