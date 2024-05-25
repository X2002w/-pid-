#pragma once





//遗传算法

//定义种群大小（论域范围最大值的个数）
#define population_num  20
//定义迭代次数
#define reproduc_num   100
//定义交叉概率
#define cross_probability_set 0.56

//定义变异概率
#define var_probability_set 0.05
 
//定义种群二维数组中数组没行表示一个个体(染色体，论域范围最大值,1<x<301）

int chromosome[20][10];//20行，即20个个体，每行/每个个体9位编码，10为适应度
int chromosome_copy[20][10];
int max_chromosome[11];//每代最优染色体
int fin_max_chromosome[11];//全局最优染色体

int choose_chromosome[20];


void find_Optimal(float chromosome[20][10], int Offspring);
void heredity_optimize(void);
//定义基因

//定义染色体（个体唯一指定），一条染色体代表问题求解的一个答案

//定义种群（染色体数目），答案的集合



//在种群即系列答案中择优，即遗传算法


//适应度，即答案的合理/优良程度，用公式表达


//选择，适应度越大，被选择的概率越高
// 交叉，扩大答案的范围
// 变异，