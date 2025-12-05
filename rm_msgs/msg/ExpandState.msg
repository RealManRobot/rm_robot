bool get_state
int16 pos	    #扩展关节角度，单位度，精度 0.001°
int16 err_flag	#驱动错误代码
uint16 en_flag	#扩展关节使能状态
uint16 current	#当前驱动电流，单位：mA，精度：1mA。
byte mode	    #当前扩展关节状态，0-空闲，1-正方向速度运动，2-正方向位置运动，3-负方向速度运动，4-负方向位置运动
uint16 joint_id	#扩展关节ID