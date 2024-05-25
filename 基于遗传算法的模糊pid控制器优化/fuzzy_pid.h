


extern float domain_max;//ģ������Χ�����Ŵ��㷨�Ż��������յõ����ʺϵķ�Χ
typedef struct 
{
    float kp;                       //����kp
    float ki;                       //����ki
    float kd;                       //����kd
    float qdetail_kp;               //����kp��Ӧ�����е�ֵ
    float qdetail_ki;               //����ki��Ӧ�����е�ֵ
    float qdetail_kd;               //����kd��Ӧ�����е�ֵ
    float qfuzzy_output;            
    float detail_kp;                //�������kp
    float detail_ki;                //�������ki
    float detail_kd;                //�������kd
    float fuzzy_output;             //ģ��pid���
    float qerro;                    //����e��Ӧ�����е�ֵ
    float qerro_c;                  //����de/dt��Ӧ�����е�ֵ
    float erro_sum;                  //������/�ۻ�


    int  num_area; //�����������
    //float e_max;  //�������ֵ
    //float e_min;  //�����Сֵ
    //float ec_max;  //���仯���ֵ
    //float ec_min;  //���仯��Сֵ
    //float kp_max, kp_min;
    float e_membership_values[7]; //����e������ֵ
    float ec_membership_values[7];//����de/dt������ֵ
    float kp_menbership_values[7];//�������kp������ֵ
    float ki_menbership_values[7]; //�������ki������ֵ
    float kd_menbership_values[7];  //�������kd������ֵ
    float fuzzyoutput_menbership_values[7];
   

    float e_gradmembership[2];      //����e��������
    float ec_gradmembership[2];     //����de/dt��������
    int e_grad_index[2];            //����e�������ڹ���������
    int ec_grad_index[2];           //����de/dt�������ڹ���������
    float gradSums[7];
    float KpgradSums[7] ;   //�������kp�ܵ�������
    float KigradSums[7] ;   //�������ki�ܵ�������
    float KdgradSums[7] ;   //�������kd�ܵ�������
    int Kp_rule_list[7][7];



}FuzzyPid;

void fuzzy_pid_init(void);



float fuzzy_pid_control(float e_max, float e_min, float ec_max, float ec_min,
    float kp_max, float kp_min, float erro, float erro_c,
    float ki_max, float ki_min, float kd_max, float kd_min,
    float erro_pre, float errp_ppre);


