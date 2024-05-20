//论域可以说是一个人为确定的范围

//要实现模糊化首先需要对模糊化进行初始化，初始化包括论域的确定以及隶属度函数的确定。
#include "fuzzy_pid.h"
FuzzyPid fuzzy_pid;//声明模糊pid变量


int NB = -3, NM = -2, NS = -1, ZO = 0, PS = 1, PM = 2, PB = 3; //论域隶属值



//初始化模糊pid参数
void fuzzy_pid_init(void) {
	fuzzy_pid.kp = 0;			//初始化基本kp
	fuzzy_pid.ki = 0;			//初始化基本ki
	fuzzy_pid.kd = 0;			//初始化基本kd

	fuzzy_pid.qdetail_kp = 0;			//初始化论域中模糊增量kp
	fuzzy_pid.qdetail_ki = 0;			//初始化论域中模糊增量ki
	fuzzy_pid.qdetail_kd = 0;			//初始化论域中模糊增量kd

	fuzzy_pid.fuzzy_output = 0;			//初始化模糊pid输出
	fuzzy_pid.qfuzzy_output = 0;			//初始化模糊pid论域中输出

	fuzzy_pid.erro_sum = 0;
	fuzzy_pid.num_area = 6; //划分区域个数

	for (int i = 0; i < 7;i++) {
		fuzzy_pid.e_membership_values[i] = i - 3; //输入e的隶属值
		fuzzy_pid.ec_membership_values[i] = i - 3;//输入de/dt的隶属值
		fuzzy_pid.kp_menbership_values[i] = i - 3;//输出增量kp的隶属值
		fuzzy_pid.ki_menbership_values[i] = i - 3; //输出增量ki的隶属值
		fuzzy_pid.kd_menbership_values[i] = i - 3;  //输出增量kd的隶属值
		fuzzy_pid.fuzzyoutput_menbership_values[i] = i - 3;
		//e_membership_values[7] = {-3,-2,-1,0,1,2,3}; //输入e的隶属值
	}


}


//区间映射函数，映射误差，误差变化率到论域中
//该模糊pid隶属度函数为固定三角形隶属度函数，论域固定为[-3, 3]。
// 后续使用遗传算法优化
float Quantization(float maximum, float minimum, float x)
{
	if (maximum == minimum)
	{
		// 处理错误或返回默认值，因为不能除以零  
		return 0; // 或者抛出一个异常  
	}
	// 计算量化值  
	float qvalues = 6.0f * (x - minimum) / (maximum - minimum) - 3.0f;
	//论域固定为[-3, 3]，可根据条件改变6.0f，3.0f
	return qvalues;
}

//输入模糊论域中误差与误差变化率，计算二者的隶属度,隶属度函数为固定三角形隶属度函数
void get_grad_membership(float erro,float erro_c) {

	//计算误差隶属度
	if (erro>fuzzy_pid.e_membership_values[0] && erro< fuzzy_pid.e_membership_values[6]) {

		for (int i = 0; i < fuzzy_pid.num_area;i++) {

			if (erro>=fuzzy_pid.e_membership_values[i] && erro<= fuzzy_pid.e_membership_values[i+1]) {

				fuzzy_pid.e_gradmembership[0] = -(erro - fuzzy_pid.e_membership_values[i + 1]) / (fuzzy_pid.e_membership_values[i + 1] - fuzzy_pid.e_membership_values[i]);
				fuzzy_pid.e_gradmembership[1] = 1+(erro - fuzzy_pid.e_membership_values[i + 1]) / (fuzzy_pid.e_membership_values[i + 1] - fuzzy_pid.e_membership_values[i]);
				fuzzy_pid.e_grad_index[0] = i;
				fuzzy_pid.e_grad_index[1] = i+1;
				break;

			}
		}
	}
	else {

		if (erro<= fuzzy_pid.e_membership_values[0]) {

			fuzzy_pid.e_gradmembership[0] = 1;
			fuzzy_pid.e_gradmembership[1] = 0;
			fuzzy_pid.e_grad_index[0] = 0;
			fuzzy_pid.e_grad_index[1] = -1;

		}
		else if (erro >= fuzzy_pid.e_membership_values[6]) {
			fuzzy_pid.e_gradmembership[0] = 1;
			fuzzy_pid.e_gradmembership[1] = 0;
			fuzzy_pid.e_grad_index[0] = 6;
			fuzzy_pid.e_grad_index[1] = -1;

		}
	}

	//计算误差变化率隶属度
	if (erro_c>fuzzy_pid.ec_membership_values[0] && erro_c< fuzzy_pid.ec_membership_values[6]) {

		for (int i = 0; i < fuzzy_pid.num_area;i++) {

			fuzzy_pid.ec_gradmembership[0] = -(erro - fuzzy_pid.ec_membership_values[i + 1]) / (fuzzy_pid.ec_membership_values[i + 1] - fuzzy_pid.ec_membership_values[i]);
			fuzzy_pid.ec_gradmembership[1] = 1+(erro - fuzzy_pid.ec_membership_values[i + 1]) / (fuzzy_pid.ec_membership_values[i + 1] - fuzzy_pid.ec_membership_values[i]);
			fuzzy_pid.ec_grad_index[0] = i;
			fuzzy_pid.ec_grad_index[1] = i + 1;
			break;

		}

	}
	else {

		if (erro_c <= fuzzy_pid.ec_membership_values[0]) {

			fuzzy_pid.ec_gradmembership[0] = 1;
			fuzzy_pid.ec_gradmembership[1] = 0;
			fuzzy_pid.ec_grad_index[0] = 0;
			fuzzy_pid.ec_grad_index[1] = -1;

		}
		else if (erro_c >= fuzzy_pid.ec_membership_values[6]) {

			fuzzy_pid.ec_gradmembership[0] = 1;
			fuzzy_pid.ec_gradmembership[1] = 0;
			fuzzy_pid.ec_grad_index[0] = 6;
			fuzzy_pid.ec_grad_index[1] = -1;

		}
	}


}

// 通过误差，误差变化率的隶属度来获取输出增量kp, ki, kd的总隶属度 

void GetSumGrad(int Kp_rule_list[7][7], int Ki_rule_list[7][7], int Kd_rule_list[7][7], int Fuzzy_rule_list[7][7]) {
	for (int i = 0; i <= fuzzy_pid.num_area + 1; i++) {

		fuzzy_pid.KpgradSums[i] = 0;
		fuzzy_pid.KigradSums[i] = 0;
		fuzzy_pid.KdgradSums[i] = 0;
	}
	for (int i = 0; i < 2;i++) {

		if (fuzzy_pid.e_grad_index[i] == -1) {

			continue;
		}
		for (int j = 0; j < 2;j++) {

			if (fuzzy_pid.ec_grad_index[j]!=-1) {
				//int NB = -3, NM = -2, NS = -1, ZO = 0, PS = 1, PM = 2, PB = 3; //论域隶属值
				int indexkp = Kp_rule_list[fuzzy_pid.e_grad_index[i]][fuzzy_pid.ec_grad_index[j]] + 3;
				int indexki = Ki_rule_list[fuzzy_pid.e_grad_index[i]][fuzzy_pid.ec_grad_index[j]] + 3;
				int indexkd = Kd_rule_list[fuzzy_pid.e_grad_index[i]][fuzzy_pid.ec_grad_index[j]] + 3;
				//indexKp后面用作数组的下标索引，而规则表Kp_rule_list二维数组中
				// 有负数元素(最小-3最大3)，因此+3是让其从(-3,3)的索引变成(0,5)
				fuzzy_pid.KpgradSums[indexkp] = fuzzy_pid.KpgradSums[indexkp] + (fuzzy_pid.e_gradmembership[i] * fuzzy_pid.ec_gradmembership[j]);
				fuzzy_pid.KigradSums[indexki] = fuzzy_pid.KigradSums[indexki] + (fuzzy_pid.e_gradmembership[i] * fuzzy_pid.ec_gradmembership[j]);
				fuzzy_pid.KdgradSums[indexkd] = fuzzy_pid.KdgradSums[indexkd] + (fuzzy_pid.e_gradmembership[i] * fuzzy_pid.ec_gradmembership[j]);

			}
			else {
			
				continue;
			}
		}
	}

}

//计算pid增量对应的模糊论域值
void GetOUT() {

	for (int i = 0; i < fuzzy_pid.num_area + 1;i++) {

		fuzzy_pid.qdetail_kp += fuzzy_pid.kp_menbership_values[i] * fuzzy_pid.KpgradSums[i];
		fuzzy_pid.qdetail_ki += fuzzy_pid.ki_menbership_values[i] * fuzzy_pid.KigradSums[i];
		fuzzy_pid.qdetail_kd += fuzzy_pid.kd_menbership_values[i] * fuzzy_pid.KdgradSums[i];

	}
}

//将pid从模糊论域预设到基本论域中
float Inverse_quantization(float num_max,float num_min,float x) {

	float qvalues = (num_max - num_min) * (x + 3) / 6 + num_min;
	return qvalues;
}

//模糊控制器
float fuzzy_pid_control(float e_max,float e_min,float ec_max,float ec_min,
	                    float kp_max,float kp_min, float erro, float erro_c, 
	                    float ki_max, float ki_min, float kd_max, float kd_min,
	                    float erro_pre, float errp_ppre) {
	//输入变量：误差最大最小值，误差变化最大最小值，pid参数最大最小值，
	// 误差，误差变化率，上次误差，上上次误差，
	//声明规则表
	int Kp_rule_list[7][7] = { {PB,PB,PM,PM,PS,ZO,ZO},     //kp规则表
							{PB,PB,PM,PS,PS,ZO,NS},
							{PM,PM,PM,PS,ZO,NS,NS},
							{PM,PM,PS,ZO,NS,NM,NM},
							{PS,PS,ZO,NS,NS,NM,NM},
							{PS,ZO,NS,NM,NM,NM,NB},
							{ZO,ZO,NM,NM,NM,NB,NB} };

	int  Ki_rule_list[7][7] = { {NB,NB,NM,NM,NS,ZO,ZO},     //ki规则表
								{NB,NB,NM,NS,NS,ZO,ZO},
								{NB,NM,NS,NS,ZO,PS,PS},
								{NM,NM,NS,ZO,PS,PM,PM},
								{NM,NS,ZO,PS,PS,PM,PB},
								{ZO,ZO,PS,PS,PM,PB,PB},
								{ZO,ZO,PS,PM,PM,PB,PB} };

	int  Kd_rule_list[7][7] = { {PS,NS,NB,NB,NB,NM,PS},    //kd规则表
								{PS,NS,NB,NM,NM,NS,ZO},
								{ZO,NS,NM,NM,NS,NS,ZO},
								{ZO,NS,NS,NS,NS,NS,ZO},
								{ZO,ZO,ZO,ZO,ZO,ZO,ZO},
								{PB,NS,PS,PS,PS,PS,PB},
								{PB,PM,PM,PM,PS,PS,PB} };

	int  Fuzzy_rule_list[7][7] = { {PB,PB,PB,PB,PM,ZO,ZO},
								   {PB,PB,PB,PM,PM,ZO,ZO},
								   {PB,PM,PM,PS,ZO,NS,NM},
								   {PM,PM,PS,ZO,NS,NM,NM},
								   {PS,PS,ZO,NM,NM,NM,NB},
								   {ZO,ZO,ZO,NM,NB,NB,NB},
								   {ZO,NS,NB,NB,NB,NB,NB} };

	fuzzy_pid.erro_sum += erro;//累积误差
	fuzzy_pid.qerro = Quantization(e_max,e_min,erro);//将误差从基本论域映射到模糊论域中
	fuzzy_pid.qerro_c = Quantization(ec_max,ec_min,erro_c);//将误差变化率从基本论域映射到模糊论域中

	get_grad_membership(fuzzy_pid.qerro,fuzzy_pid.qerro_c);//计算模糊论域中的误差，误差变化率的隶属度
	GetSumGrad(Kp_rule_list, Ki_rule_list, Kd_rule_list, Fuzzy_rule_list);
	GetOUT();
	fuzzy_pid.detail_kp = Inverse_quantization(kp_max, kp_min,fuzzy_pid.qdetail_kp);
	fuzzy_pid.detail_ki = Inverse_quantization(ki_max, ki_min, fuzzy_pid.qdetail_ki);
	fuzzy_pid.detail_kd = Inverse_quantization(kd_max, kd_min, fuzzy_pid.qdetail_kd);

	fuzzy_pid.qdetail_kp = 0;
	fuzzy_pid.qdetail_ki = 0;
	fuzzy_pid.qdetail_kd = 0;

	fuzzy_pid.kp = fuzzy_pid.kp + fuzzy_pid.detail_kp;
	fuzzy_pid.ki = fuzzy_pid.ki + fuzzy_pid.detail_ki;
	fuzzy_pid.kd = fuzzy_pid.kd + fuzzy_pid.detail_kd;

	if (fuzzy_pid.kp < 0)
		fuzzy_pid.kp = 0;
	if (fuzzy_pid.ki < 0)
		fuzzy_pid.ki = 0;
	if (fuzzy_pid.kd < 0)
		fuzzy_pid.kd = 0;

	fuzzy_pid.detail_kp = 0;
	fuzzy_pid.detail_ki = 0;
	fuzzy_pid.detail_kd = 0;

	float output = fuzzy_pid.detail_kp * (erro - erro_pre) + fuzzy_pid.ki * erro + fuzzy_pid.kd * (erro-2*erro_pre+errp_ppre);

	return output;

}