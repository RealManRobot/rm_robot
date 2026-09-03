#设置灵巧手角度，灵巧手有 6 个自由度，从 1~6 分别为小拇指，无名指，中指，食指，大拇指弯曲，大拇指旋转

int16[6] hand_angle	#手指角度数组，范围：0~1000.另外，-1 代表该自由度不执行任何操作，保持当前状态
int16 num_dof #Hand_FollowAngle 必填 1 或 6；Set_Hand_Angle 仅支持 6，兼容旧发布端的默认值 0。
bool block #Set_Hand_Angle 使用阻塞标志；Hand_FollowAngle 不使用此字段。
