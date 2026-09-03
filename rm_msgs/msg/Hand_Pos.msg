#位置跟随：num_dof 必须匹配实际末端的自由度，1 或 6。

int16[6] hand_angle	#手指角度数组，范围：0~1000.另外，-1 代表该自由度不执行任何操作，保持当前状态
int16 num_dof #仅下发 hand_angle 的前 num_dof 项；0 为无效值。
bool block #兼容保留字段，Hand_FollowPos 不使用此字段。
