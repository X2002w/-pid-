//�������˵��һ����Ϊȷ���ķ�Χ

//Ҫʵ��ģ����������Ҫ��ģ�������г�ʼ������ʼ�����������ȷ���Լ������Ⱥ�����ȷ����
#include "fuzzy_pid.h"
FuzzyPid fuzzy_pid;//����ģ��pid����


int NB = -3, NM = -2, NS = -1, ZO = 0, PS = 1, PM = 2, PB = 3; //��������ֵ



//��ʼ��ģ��pid����
void fuzzy_pid_init(void) {
	fuzzy_pid.kp = 0;			//��ʼ������kp
	fuzzy_pid.ki = 0;			//��ʼ������ki
	fuzzy_pid.kd = 0;			//��ʼ������kd

	fuzzy_pid.qdetail_kp = 0;			//��ʼ��������ģ������kp
	fuzzy_pid.qdetail_ki = 0;			//��ʼ��������ģ������ki
	fuzzy_pid.qdetail_kd = 0;			//��ʼ��������ģ������kd

	fuzzy_pid.fuzzy_output = 0;			//��ʼ��ģ��pid���
	fuzzy_pid.qfuzzy_output = 0;			//��ʼ��ģ��pid���������

	fuzzy_pid.erro_sum = 0;
	fuzzy_pid.num_area = 6; //�����������

	for (int i = 0; i < 7;i++) {
		fuzzy_pid.e_membership_values[i] = i - 3; //����e������ֵ
		fuzzy_pid.ec_membership_values[i] = i - 3;//����de/dt������ֵ
		fuzzy_pid.kp_menbership_values[i] = i - 3;//�������kp������ֵ
		fuzzy_pid.ki_menbership_values[i] = i - 3; //�������ki������ֵ
		fuzzy_pid.kd_menbership_values[i] = i - 3;  //�������kd������ֵ
		fuzzy_pid.fuzzyoutput_menbership_values[i] = i - 3;
		//e_membership_values[7] = {-3,-2,-1,0,1,2,3}; //����e������ֵ
	}


}


//����ӳ�亯����ӳ�������仯�ʵ�������
//��ģ��pid�����Ⱥ���Ϊ�̶������������Ⱥ���������̶�Ϊ[-3, 3]��
// ����ʹ���Ŵ��㷨�Ż�
float Quantization(float maximum, float minimum, float x)
{
	if (maximum == minimum)
	{
		// �������򷵻�Ĭ��ֵ����Ϊ���ܳ�����  
		return 0; // �����׳�һ���쳣  
	}
	// ��������ֵ  
	float qvalues = 6.0f * (x - minimum) / (maximum - minimum) - 3.0f;
	//����̶�Ϊ[-3, 3]���ɸ��������ı�6.0f��3.0f
	return qvalues;
}

//����ģ����������������仯�ʣ�������ߵ�������,�����Ⱥ���Ϊ�̶������������Ⱥ���
void get_grad_membership(float erro,float erro_c) {

	//�������������
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

	//�������仯��������
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

// ͨ�������仯�ʵ�����������ȡ�������kp, ki, kd���������� 

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
				//int NB = -3, NM = -2, NS = -1, ZO = 0, PS = 1, PM = 2, PB = 3; //��������ֵ
				int indexkp = Kp_rule_list[fuzzy_pid.e_grad_index[i]][fuzzy_pid.ec_grad_index[j]] + 3;
				int indexki = Ki_rule_list[fuzzy_pid.e_grad_index[i]][fuzzy_pid.ec_grad_index[j]] + 3;
				int indexkd = Kd_rule_list[fuzzy_pid.e_grad_index[i]][fuzzy_pid.ec_grad_index[j]] + 3;
				//indexKp��������������±��������������Kp_rule_list��ά������
				// �и���Ԫ��(��С-3���3)�����+3�������(-3,3)���������(0,5)
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

//����pid������Ӧ��ģ������ֵ
void GetOUT() {

	for (int i = 0; i < fuzzy_pid.num_area + 1;i++) {

		fuzzy_pid.qdetail_kp += fuzzy_pid.kp_menbership_values[i] * fuzzy_pid.KpgradSums[i];
		fuzzy_pid.qdetail_ki += fuzzy_pid.ki_menbership_values[i] * fuzzy_pid.KigradSums[i];
		fuzzy_pid.qdetail_kd += fuzzy_pid.kd_menbership_values[i] * fuzzy_pid.KdgradSums[i];

	}
}

//��pid��ģ������Ԥ�赽����������
float Inverse_quantization(float num_max,float num_min,float x) {

	float qvalues = (num_max - num_min) * (x + 3) / 6 + num_min;
	return qvalues;
}

//ģ��������
float fuzzy_pid_control(float e_max,float e_min,float ec_max,float ec_min,
	                    float kp_max,float kp_min, float erro, float erro_c, 
	                    float ki_max, float ki_min, float kd_max, float kd_min,
	                    float erro_pre, float errp_ppre) {
	//�����������������Сֵ�����仯�����Сֵ��pid���������Сֵ��
	// �����仯�ʣ��ϴ������ϴ���
	//���������
	int Kp_rule_list[7][7] = { {PB,PB,PM,PM,PS,ZO,ZO},     //kp�����
							{PB,PB,PM,PS,PS,ZO,NS},
							{PM,PM,PM,PS,ZO,NS,NS},
							{PM,PM,PS,ZO,NS,NM,NM},
							{PS,PS,ZO,NS,NS,NM,NM},
							{PS,ZO,NS,NM,NM,NM,NB},
							{ZO,ZO,NM,NM,NM,NB,NB} };

	int  Ki_rule_list[7][7] = { {NB,NB,NM,NM,NS,ZO,ZO},     //ki�����
								{NB,NB,NM,NS,NS,ZO,ZO},
								{NB,NM,NS,NS,ZO,PS,PS},
								{NM,NM,NS,ZO,PS,PM,PM},
								{NM,NS,ZO,PS,PS,PM,PB},
								{ZO,ZO,PS,PS,PM,PB,PB},
								{ZO,ZO,PS,PM,PM,PB,PB} };

	int  Kd_rule_list[7][7] = { {PS,NS,NB,NB,NB,NM,PS},    //kd�����
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

	fuzzy_pid.erro_sum += erro;//�ۻ����
	fuzzy_pid.qerro = Quantization(e_max,e_min,erro);//�����ӻ�������ӳ�䵽ģ��������
	fuzzy_pid.qerro_c = Quantization(ec_max,ec_min,erro_c);//�����仯�ʴӻ�������ӳ�䵽ģ��������

	get_grad_membership(fuzzy_pid.qerro,fuzzy_pid.qerro_c);//����ģ�������е������仯�ʵ�������
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