


extern float domain_max;//模糊论域范围，由遗传算法优化迭代最终得到最适合的范围
typedef struct 
{
    float kp;                       //基本kp
    float ki;                       //基本ki
    float kd;                       //基本kd
    float qdetail_kp;               //增量kp对应论域中的值
    float qdetail_ki;               //增量ki对应论域中的值
    float qdetail_kd;               //增量kd对应论域中的值
    float qfuzzy_output;            
    float detail_kp;                //输出增量kp
    float detail_ki;                //输出增量ki
    float detail_kd;                //输出增量kd
    float fuzzy_output;             //模糊pid输出
    float qerro;                    //输入e对应论域中的值
    float qerro_c;                  //输入de/dt对应论域中的值
    float erro_sum;                  //误差积分/累积


    int  num_area; //划分区域个数
    //float e_max;  //误差做大值
    //float e_min;  //误差最小值
    //float ec_max;  //误差变化最大值
    //float ec_min;  //误差变化最小值
    //float kp_max, kp_min;
    float e_membership_values[7]; //输入e的隶属值
    float ec_membership_values[7];//输入de/dt的隶属值
    float kp_menbership_values[7];//输出增量kp的隶属值
    float ki_menbership_values[7]; //输出增量ki的隶属值
    float kd_menbership_values[7];  //输出增量kd的隶属值
    float fuzzyoutput_menbership_values[7];
   

    float e_gradmembership[2];      //输入e的隶属度
    float ec_gradmembership[2];     //输入de/dt的隶属度
    int e_grad_index[2];            //输入e隶属度在规则表的索引
    int ec_grad_index[2];           //输入de/dt隶属度在规则表的索引
    float gradSums[7];
    float KpgradSums[7] ;   //输出增量kp总的隶属度
    float KigradSums[7] ;   //输出增量ki总的隶属度
    float KdgradSums[7] ;   //输出增量kd总的隶属度
    int Kp_rule_list[7][7];



}FuzzyPid;

void fuzzy_pid_init(void);



float fuzzy_pid_control(float e_max, float e_min, float ec_max, float ec_min,
    float kp_max, float kp_min, float erro, float erro_c,
    float ki_max, float ki_min, float kd_max, float kd_min,
    float erro_pre, float errp_ppre);


