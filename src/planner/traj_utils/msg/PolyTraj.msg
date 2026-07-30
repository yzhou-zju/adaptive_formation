int16 drone_id #来源于哪个飞机
int32 traj_id   
time start_time

uint8 order #多项式阶次
float32[] coef_x       #x方向系数
float32[] coef_y
float32[] coef_z
float32[] duration
float32   constraint_aware
float32   vis_aware