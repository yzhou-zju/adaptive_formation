int16 drone_id #来源于哪个飞机
int32 traj_id   
int32 traj_piecenum  
time start_time
bool agent_add
bool change_front
bool use_leader

uint8 order #多项式阶次
float32[] coef_x_pos       #x方向系数
float32[] coef_y_pos
float32[] coef_z_pos
float32[] duration

int32 ass_num
float32[] assig_id

float32[] coef_r       #r系数

float32[] coef_l       #l系数


float32[] assi_x_pos       #assig_pos
float32[] assi_y_pos
float32[] assi_z_pos