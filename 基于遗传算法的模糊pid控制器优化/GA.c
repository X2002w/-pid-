#include "GA.h"
#include "fuzzy_pid.h"
#include "stdlib.h"
#include "time.h"


//���������
#define Seed  srand((unsigned)time(0));
//ģ��pid������ʼ��
//������Χȡֵ���ж����Ʊ��룬������ȡ��x=1----512��
//�����Ʊ��뾫��=(max-min)/2^n-1,nΪ�����ƴ���0/1��
void binary_encoding() {

}

//��Ⱥ��ʼ������ʼ�������ģ������Χ,����ʼ��Ӧ�ȣ�
void GA_init(void) {
	for (int i = 0; i < 20;i++) {
		printf("��%d������", i);
		for (int j = 0; j < 9;j++) {

			chromosome[i][j] = rand() % 2; //����0~1�������
			printf("%d", chromosome[i][j]);
		}
	//printf("\n");
		//��ʼ���������Ӧ�ȳ�ʼ��
		chromosome[i][9] = adopt_value(chromosome,i);
	}

	find_Optimal(chromosome,0);
}

//ѡ�����

void choose(int chromosome[20][10]) {

	float adapt_sum[20] = { 0 };//�������Ӧ�ȣ������������ۼ����
	adapt_sum[0] = chromosome[0][9];//�����һ���������Ӧ��

	for (int i = 1; i < 20; i++) {
		adapt_sum[i] = adapt_sum[i - 1] + chromosome[i][9];

	}
	//ѡ���ĸ�Ⱦɫ�壬�����±괢����choose_chromosome[]������
	for (int j = 0; j < 20; j++) {

		//�������ѡ��Ⱦɫ�����Ӧ�ȣ������̶�ѡ��
		int choose_adapt = rand()% ((int)adapt_sum[9]+1);


		//��ͬ�����Ӧ��ͬ��Ⱦɫ��
		if (choose_adapt >= 0 && choose_adapt < (int)adapt_sum[0]) {
			choose_chromosome[j] = 0;
		}
		else if (choose_adapt >= (int)adapt_sum[0] && choose_adapt < (int)adapt_sum[1]) {
			choose_chromosome[j] = 1;
		}
		else if (choose_adapt >= (int)adapt_sum[1] && choose_adapt < (int)adapt_sum[2]) {
			choose_chromosome[j] = 2;
		}
		else if (choose_adapt >= (int)adapt_sum[2] && choose_adapt < (int)adapt_sum[3]) {
			choose_chromosome[j] = 3;
		}
		else if (choose_adapt >= (int)adapt_sum[3] && choose_adapt < (int)adapt_sum[4]) {
			choose_chromosome[j] = 4;
		}
		else if (choose_adapt >= (int)adapt_sum[4] && choose_adapt < (int)adapt_sum[5]) {
			choose_chromosome[j] = 5;
		}
		else if (choose_adapt >= (int)adapt_sum[5] && choose_adapt < (int)adapt_sum[6]) {
			choose_chromosome[j] = 6;
		}
		else if (choose_adapt >= (int)adapt_sum[6] && choose_adapt < (int)adapt_sum[7]) {
			choose_chromosome[j] = 7;
		}
		else if (choose_adapt >= (int)adapt_sum[7] && choose_adapt < (int)adapt_sum[8]) {
			choose_chromosome[j] = 8;
		}
		else if (choose_adapt >= (int)adapt_sum[8] && choose_adapt <= (int)adapt_sum[9]) {
			choose_chromosome[j] = 9;
		}
		else if (choose_adapt >= (int)adapt_sum[9] && choose_adapt <= (int)adapt_sum[10]) {
			choose_chromosome[j] = 10;
		}
		else if (choose_adapt >= (int)adapt_sum[10] && choose_adapt <= (int)adapt_sum[11]) {
			choose_chromosome[j] = 11;
		}
		else if (choose_adapt >= (int)adapt_sum[11] && choose_adapt <= (int)adapt_sum[12]) {
			choose_chromosome[j] = 12;
		}
		else if (choose_adapt >= (int)adapt_sum[12] && choose_adapt <= (int)adapt_sum[13]) {
			choose_chromosome[j] = 13;
		}
		else if (choose_adapt >= (int)adapt_sum[13] && choose_adapt <= (int)adapt_sum[14]) {
			choose_chromosome[j] = 14;
		}
		else if (choose_adapt >= (int)adapt_sum[14] && choose_adapt <= (int)adapt_sum[15]) {
			choose_chromosome[j] = 15;
		}
		else if (choose_adapt >= (int)adapt_sum[15] && choose_adapt <= (int)adapt_sum[16]) {
			choose_chromosome[j] = 16;
		}
		else if (choose_adapt >= (int)adapt_sum[16] && choose_adapt <= (int)adapt_sum[17]) {
			choose_chromosome[j] = 17;
		}
		else if (choose_adapt >= (int)adapt_sum[17] && choose_adapt <= (int)adapt_sum[18]) {
			choose_chromosome[j] = 18;
		}
		else if (choose_adapt >= (int)adapt_sum[18] && choose_adapt <= (int)adapt_sum[19]) {
			choose_chromosome[j] = 19;
		}

	}
}


//�����㷨
void variation(int chromosome[20][10]) {

	//ѭ���Ƚ�ÿ�������Ƿ�����
	for (int i = 0; i < 20;i++) {
		//������ɱ�����ʣ����趨ֵ��Ƚ�
		float var_probability= (rand() % 100)/100.0f;
		
		if (var_probability>= var_probability_set) {
			//���忪ʼ����
			int var_num = rand() % 5 + 1;//������������λ�����Ϊ5λ
			//int* new_var = (int*)malloc(sizeof(int)*var_num);
			//ѭ����������Ļ���λ
			for (int j = 0; j <= var_num;j++) {
				int var_index;
				var_index = rand() % 9;

			//	new_var[j] = var_index;

				chromosome[i][var_index] = 1 - chromosome[i][var_index];
			}
			//free(new_var);
		}
	}
}

//�����㷨

void cross(int chromosome_copy[20][10], int chromosome[20][10]) {
		//������ĸ�� ��ͬ����20��Ⱦɫ�壬�ɶԽ��н��� 
		
	float n = (rand() % 100) / 100.0f;
	printf("\n%f,%f\n",n,cross_probability_set);//��һ���Ƿ񽻲�
	if (n < cross_probability_set) {
		for (int i = 0; i < 10; i++) { // 20��Ⱦɫ��  
			int crossPoints[2]; // �洢һ��Ⱦɫ��Ľ����  

			// ѡ���������������ظ��Ľ����  
			for (int j = 0; j < 2; j++) {
				int point;
				do {// ���ѡ��һ��λ��  
					point = rand() % 9;
					// ����λ���Ƿ��Ѿ���ѡΪ�����  
					for (int k = 0; k < j; k++) {
						if (point == crossPoints[k]) {
							point = -1; // ����Ѿ�ѡ�����������ѡ��  
							break;
						}
					}
				} while (point == -1); // ȷ��ѡ���˲��ظ��Ľ����  
				crossPoints[j] = point;
			}

			// �������������������ĸ���ڽ�����ϵĻ���  
			for (int j = 0; j < 9; j++) {
				if (j == crossPoints[0] || j == crossPoints[1]) {
					// ��������  
					double temp = chromosome_copy[i * 2][j];
					chromosome_copy[i * 2][j] = chromosome_copy[i * 2 + 1][j];
					chromosome_copy[i * 2 + 1][j] = temp;
				}
			}

			// ��������Ⱦɫ�帴�Ƶ� chromosome  
			for (int j = 0; j < 9; j++) {
				chromosome[i * 2][j] = chromosome_copy[i * 2][j];
				chromosome[i * 2 + 1][j] = chromosome_copy[i * 2 + 1][j];
			}
		}
	}
	else {
		for (int i = 0; i < 20; i++) {
			for (int j = 0; j < 9; j++) {
				chromosome[i][j] = chromosome_copy[i][j];
			}

		}
	}

}

//�������Ž�,�������飬�Ӵ���
void find_Optimal(int chromosome[20][10],int Offspring) {

	max_chromosome[9] =0;//����ֵ����Ӧ����Ϊ0��
	//����ÿ������
	for (int i = 0; i < 20;i++) {
		//�Ƚ��Ӵ���Ӧ��
		if (chromosome[i][9]>max_chromosome[9]) {

			max_chromosome[9] = chromosome[i][9];
			max_chromosome[10] = i;//����ÿ�����ŵ�����
		}
	}

	//ȫ������

	printf("\n%d,%d\n", fin_max_chromosome[9], max_chromosome[9]);

	if (fin_max_chromosome[9] < max_chromosome[9]) {
		fin_max_chromosome[9] = max_chromosome[9];
		for (int j = 0; j < 9; j++) {

			fin_max_chromosome[j] = chromosome[max_chromosome[10]][j];
		}
	}
/*
	for (int i = 0; i < 20;i++) {
		if (fin_max_chromosome[9] < chromosome[i][9]) {

			fin_max_chromosome[9] = chromosome[i][9];//����ȫ�����ŵ���Ӧ��
			fin_max_chromosome[10] = Offspring;//����ȫ�����ŵ��Ӵ���

			//����ȫ�����ŵĶ����Ʊ���
			for (int j = 0; j < 9; j++) {

				fin_max_chromosome[j] = chromosome[i][j];
			}
		}

	}*/
	int u;
	int num;
	int i = max_chromosome[10];
	printf("\n��%d�����Ÿ���Ϊ:%d", Offspring, i);
	//������ҽ����
	u = chromosome[i][0] * 256 + chromosome[i][1] * 128 + chromosome[i][2] * 64 + chromosome[i][3] * 32 + chromosome[i][4] * 16
		+ chromosome[i][5] * 8 + chromosome[i][6] * 4 + chromosome[i][7] * 2 + chromosome[i][8] * 1;

	num = fin_max_chromosome[0] * 256 + fin_max_chromosome[1] * 128 + fin_max_chromosome[2] * 64 + fin_max_chromosome[3] * 32 + fin_max_chromosome[4] * 16
		+ fin_max_chromosome[5] * 8 + fin_max_chromosome[6] * 4 + fin_max_chromosome[7] * 2 + fin_max_chromosome[8] * 1;
	
	printf("  ����ֵΪ: %d   ��Ӧ��Ϊ:%d\n",u,1000- max_chromosome[9]);
	printf("ȫ������ֵΪ:%d,��Ӧ��Ϊ��%d\n",num, 1000-fin_max_chromosome[9]);
	printf("\n");


}


//��������Ӧ��
//��ÿ���������һ��ģ��pid���ƣ�����ʵ��ֵ��Ŀ��ֵ��ӽ���(�����С��)pid����������
// �������������壨�������ֵ������Ӧ��
float adopt_value(int chromosome[20][10], int i) {


	for (int i = 0; i < 20; i++) {
		for (int j = 0; j < 9; j++) {

			
				//printf("%d", chromosome[i][j]);
		}
			//printf("\n");

	}
	//�������
	domain_max = chromosome[i][0]*256+ chromosome[i][1]*128+ chromosome[i][2]*64+ chromosome[i][3]*32+ chromosome[i][4]*16
		+ chromosome[i][5]*8+ chromosome[i][6]*4+ chromosome[i][7]*2+ chromosome[i][8]*1;

	if (domain_max==0) {
		domain_max = 1;
	}
	printf("   ��%d�����˽��룺%d\n",i, domain_max);

	float target = 50;
	float actual = 0;
	float e_max = 600;
	float e_min = -600;
	float ec_max = 600;
	float ec_min = -600;
	float kp_max = 0.5;
	float kp_min = 0;
	float ki_max = 0.02;
	float ki_min = 0;
	float kd_max = 0.05;
	float kd_min = 0;
	int adaption = 1000;
	float adaption_num = 50;
	
	float erro;
	float erro_c;
	float erro_pre = 0;
	float erro_ppre = 0;
	erro = target - actual;
	erro_c = erro - erro_pre;
	for (int i = 0; i < 1000; i++) {

		float s;
		s = fuzzy_pid_control(e_max, e_min, ec_max, ec_min, kp_max, kp_min, erro, erro_c,
			ki_max, ki_min, kd_max, kd_min, erro_pre, erro_ppre);


		actual += s;//transfer_fac(s);

		erro_ppre = erro_pre;
		erro_pre = erro;
		erro = target - actual;
		erro_c = erro - erro_pre;

		if (erro < adaption_num ) {
			adaption_num = erro;
			adaption = i;
			//printf("���Ϊ��%f\n", erro);
			//printf("�������Ӧ��Ϊ��%d\n", 1000 - adaption);
			//printf("\n");
		}
		
	}

	return 1000-adaption;//������Сֵ��ת��Ϊ�����ֵ�����������̶ĵĽ���
}


//�Ŵ��Ż�

void heredity_optimize(void) {
	Seed
	GA_init();

	//ѭ������
	for (int reproduc = 1; reproduc <= reproduc_num; reproduc++) {
		//ѡ�����
		choose(chromosome);

		//����ѡ���ĸ���
		for (int i = 0; i < 20;i++) {
			//printf("��%d������", i);
			for (int j = 0; j < 9;j++) {

				chromosome_copy[i][j] = chromosome[choose_chromosome[i]][j];
				//printf("%d", chromosome_copy[i][j]);
			//	printf("��%d�����壺%d", i,chromosome[i][j]);
			}
		//	printf("\n");

		}

		//�������
		cross(chromosome_copy, chromosome);

		//����ѡ���ĸ���
		for (int i = 0; i < 20; i++) {
			printf("��%d������:", i);
			for (int j = 0; j < 9; j++) {

				//chromosome_copy[i][j] = chromosome[choose_chromosome[i]][j];
				printf("%d", chromosome_copy[i][j]);
				
				//	printf("��%d�����壺%d", i,chromosome[i][j]);
			}

			printf("    ");
			
		//�������
		variation(chromosome);
		for (int j = 0; j < 9; j++) {

			//chromosome_copy[i][j] = chromosome[choose_chromosome[i]][j];
			//printf("%d", chromosome_copy[i][j]);
			printf("%d", chromosome[i][j]);
			//	printf("��%d�����壺%d", i,chromosome[i][j]);
		}


		printf("\n");

		}


	/*	printf("\n");
		printf("\n");
		printf("\n");*/
		//������Ӧ��
		for (int i = 0; i < 20;i++) {
			chromosome[i][9]= adopt_value(chromosome, i);

			printf("��%d���������Ӧ��Ϊ��%d",i,1000-chromosome[i][9]);
		//	printf("\n");
		}
		//�������Ž�

		find_Optimal(chromosome, reproduc);
	}

	//���Ž�
	float x;

	x = fin_max_chromosome[0] * 256 + fin_max_chromosome[1] * 128 + fin_max_chromosome[2] * 64 + fin_max_chromosome[3] * 32 + fin_max_chromosome[4] * 16
		+ fin_max_chromosome[5] * 8 + fin_max_chromosome[6] * 4 + fin_max_chromosome[7] * 2 + fin_max_chromosome[8] * 1;
	domain_max = x;
	if (domain_max <= 0) {
		domain_max = 3;
	}
	printf("���ҵ�������ģ������Ϊ��%d\n", domain_max);
}



