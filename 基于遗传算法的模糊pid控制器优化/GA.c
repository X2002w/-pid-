#include "GA.h"
#include "fuzzy_pid.h"
#include "stdlib.h"
#include "time.h"


//随机数种子
#define Seed  srand((unsigned)time(0));
//模糊pid参数初始化
//对论域范围取值进行二级制编码，表现型取（x=1----512）
//二进制编码精度=(max-min)/2^n-1,n为二进制串的0/1数
void binary_encoding() {

}

//种群初始化（初始化最初的模糊论域范围,及初始适应度）
void GA_init(void) {
	for (int i = 0; i < 20;i++) {
		printf("第%d个个体", i);
		for (int j = 0; j < 9;j++) {

			chromosome[i][j] = rand() % 2; //产生0~1的随机数
			printf("%d", chromosome[i][j]);
		}
	//printf("\n");
		//初始化个体的适应度初始化
		chromosome[i][9] = adopt_value(chromosome,i);
	}

	find_Optimal(chromosome,0);
}

//选择操作

void choose(int chromosome[20][10]) {

	float adapt_sum[20] = { 0 };//个体的适应度，按照索引数累加求和
	adapt_sum[0] = chromosome[0][9];//储存第一个个体的适应度

	for (int i = 1; i < 20; i++) {
		adapt_sum[i] = adapt_sum[i - 1] + chromosome[i][9];

	}
	//选择哪个染色体，将其下标储存在choose_chromosome[]数组中
	for (int j = 0; j < 20; j++) {

		//随机生成选择染色体的适应度，即轮盘赌选择
		int choose_adapt = rand()% ((int)adapt_sum[9]+1);


		//不同区域对应不同的染色体
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


//变异算法
void variation(int chromosome[20][10]) {

	//循环比较每个个体是否会变异
	for (int i = 0; i < 20;i++) {
		//随机生成变异概率，与设定值相比较
		float var_probability= (rand() % 100)/100.0f;
		
		if (var_probability>= var_probability_set) {
			//个体开始变异
			int var_num = rand() % 5 + 1;//随机发生变异的位，最高为5位
			//int* new_var = (int*)malloc(sizeof(int)*var_num);
			//循环遍历个体的基因位
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

//交叉算法

void cross(int chromosome_copy[20][10], int chromosome[20][10]) {
		//父本，母本 相同，共20对染色体，成对进行交叉 
		
	float n = (rand() % 100) / 100.0f;
	printf("\n%f,%f\n",n,cross_probability_set);//这一代是否交叉
	if (n < cross_probability_set) {
		for (int i = 0; i < 10; i++) { // 20对染色体  
			int crossPoints[2]; // 存储一对染色体的交叉点  

			// 选择多个（两个）不重复的交叉点  
			for (int j = 0; j < 2; j++) {
				int point;
				do {// 随机选择一个位置  
					point = rand() % 9;
					// 检查该位置是否已经被选为交叉点  
					for (int k = 0; k < j; k++) {
						if (point == crossPoints[k]) {
							point = -1; // 如果已经选择过，则重新选择  
							break;
						}
					}
				} while (point == -1); // 确保选择了不重复的交叉点  
				crossPoints[j] = point;
			}

			// 交叉操作：交换父本和母本在交叉点上的基因  
			for (int j = 0; j < 9; j++) {
				if (j == crossPoints[0] || j == crossPoints[1]) {
					// 交换基因  
					double temp = chromosome_copy[i * 2][j];
					chromosome_copy[i * 2][j] = chromosome_copy[i * 2 + 1][j];
					chromosome_copy[i * 2 + 1][j] = temp;
				}
			}

			// 将交叉后的染色体复制到 chromosome  
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

//查找最优解,个体数组，子代数
void find_Optimal(int chromosome[20][10],int Offspring) {

	max_chromosome[9] =0;//最优值的适应度置为0；
	//查找每代最优
	for (int i = 0; i < 20;i++) {
		//比较子代适应度
		if (chromosome[i][9]>max_chromosome[9]) {

			max_chromosome[9] = chromosome[i][9];
			max_chromosome[10] = i;//保存每代最优的索引
		}
	}

	//全局最优

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

			fin_max_chromosome[9] = chromosome[i][9];//保存全局最优的适应度
			fin_max_chromosome[10] = Offspring;//保存全局最优的子代数

			//储存全局最优的二进制编码
			for (int j = 0; j < 9; j++) {

				fin_max_chromosome[j] = chromosome[i][j];
			}
		}

	}*/
	int u;
	int num;
	int i = max_chromosome[10];
	printf("\n第%d代最优个体为:%d", Offspring, i);
	//输出查找结果：
	u = chromosome[i][0] * 256 + chromosome[i][1] * 128 + chromosome[i][2] * 64 + chromosome[i][3] * 32 + chromosome[i][4] * 16
		+ chromosome[i][5] * 8 + chromosome[i][6] * 4 + chromosome[i][7] * 2 + chromosome[i][8] * 1;

	num = fin_max_chromosome[0] * 256 + fin_max_chromosome[1] * 128 + fin_max_chromosome[2] * 64 + fin_max_chromosome[3] * 32 + fin_max_chromosome[4] * 16
		+ fin_max_chromosome[5] * 8 + fin_max_chromosome[6] * 4 + fin_max_chromosome[7] * 2 + fin_max_chromosome[8] * 1;
	
	printf("  最优值为: %d   适应度为:%d\n",u,1000- max_chromosome[9]);
	printf("全局最优值为:%d,适应度为：%d\n",num, 1000-fin_max_chromosome[9]);
	printf("\n");


}


//求解个体适应度
//得每个个体进行一次模糊pid控制，查找实际值与目标值最接近的(误差最小的)pid迭代索引，
// 此索引当作个体（论域最大值）的适应度
float adopt_value(int chromosome[20][10], int i) {


	for (int i = 0; i < 20; i++) {
		for (int j = 0; j < 9; j++) {

			
				//printf("%d", chromosome[i][j]);
		}
			//printf("\n");

	}
	//解码个体
	domain_max = chromosome[i][0]*256+ chromosome[i][1]*128+ chromosome[i][2]*64+ chromosome[i][3]*32+ chromosome[i][4]*16
		+ chromosome[i][5]*8+ chromosome[i][6]*4+ chromosome[i][7]*2+ chromosome[i][8]*1;

	if (domain_max==0) {
		domain_max = 1;
	}
	printf("   第%d个个人解码：%d\n",i, domain_max);

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
			//printf("误差为：%f\n", erro);
			//printf("个体的适应度为：%d\n", 1000 - adaption);
			//printf("\n");
		}
		
	}

	return 1000-adaption;//将求最小值，转化为求最大值，再用于轮盘赌的进行
}


//遗传优化

void heredity_optimize(void) {
	Seed
	GA_init();

	//循环迭代
	for (int reproduc = 1; reproduc <= reproduc_num; reproduc++) {
		//选择操作
		choose(chromosome);

		//储存选出的个体
		for (int i = 0; i < 20;i++) {
			//printf("第%d个个体", i);
			for (int j = 0; j < 9;j++) {

				chromosome_copy[i][j] = chromosome[choose_chromosome[i]][j];
				//printf("%d", chromosome_copy[i][j]);
			//	printf("第%d个个体：%d", i,chromosome[i][j]);
			}
		//	printf("\n");

		}

		//交叉操作
		cross(chromosome_copy, chromosome);

		//储存选出的个体
		for (int i = 0; i < 20; i++) {
			printf("第%d个个体:", i);
			for (int j = 0; j < 9; j++) {

				//chromosome_copy[i][j] = chromosome[choose_chromosome[i]][j];
				printf("%d", chromosome_copy[i][j]);
				
				//	printf("第%d个个体：%d", i,chromosome[i][j]);
			}

			printf("    ");
			
		//变异操作
		variation(chromosome);
		for (int j = 0; j < 9; j++) {

			//chromosome_copy[i][j] = chromosome[choose_chromosome[i]][j];
			//printf("%d", chromosome_copy[i][j]);
			printf("%d", chromosome[i][j]);
			//	printf("第%d个个体：%d", i,chromosome[i][j]);
		}


		printf("\n");

		}


	/*	printf("\n");
		printf("\n");
		printf("\n");*/
		//储存适应度
		for (int i = 0; i < 20;i++) {
			chromosome[i][9]= adopt_value(chromosome, i);

			printf("第%d个个体的适应度为：%d",i,1000-chromosome[i][9]);
		//	printf("\n");
		}
		//查找最优解

		find_Optimal(chromosome, reproduc);
	}

	//最优解
	float x;

	x = fin_max_chromosome[0] * 256 + fin_max_chromosome[1] * 128 + fin_max_chromosome[2] * 64 + fin_max_chromosome[3] * 32 + fin_max_chromosome[4] * 16
		+ fin_max_chromosome[5] * 8 + fin_max_chromosome[6] * 4 + fin_max_chromosome[7] * 2 + fin_max_chromosome[8] * 1;
	domain_max = x;
	if (domain_max <= 0) {
		domain_max = 3;
	}
	printf("查找到的最优模糊论域为：%d\n", domain_max);
}



