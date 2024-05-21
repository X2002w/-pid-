#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include "fuzzy_pid.h"


/*
基于遗传算法的模糊控制器优化设计的详细步骤如下：

确定模糊控制器结构：
	+定义模糊控制器的输入和输出变量，并确定它们的模糊集数量。
	为每个模糊集定义隶属度函数（membership functions）。
	设计模糊控制规则，这些规则定义了如何从输入变量的模糊集映射到输出变量的模糊集。

编码模糊控制器参数：
	将模糊控制器的参数（如隶属度函数的参数、模糊控制规则等）编码为遗传算法中的染色体（个体）。每个参数可以是一个基因，而整个参数集则构成一个染色体。

定义适应度函数：
	设计一个适应度函数来评估每个染色体（即模糊控制器参数集）的性能。这个函数通常基于模糊控制器在控制系统上的表现，如误差平方和（SSE）、最大误差（ME）或其他性能指标。

初始化种群：
	随机生成一组染色体（即模糊控制器参数集）作为遗传算法的初始种群。种群中的每个个体都代表一个可能的模糊控制器参数集。

遗传算法过程：
	选择（Selection）：基于适应度函数值，从当前种群中选择一部分个体作为父代。通常使用轮盘赌选择、锦标赛选择等方法。
	交叉（Crossover）：随机选择两个父代个体，并按照一定的交叉概率和交叉方式（如单点交叉、多点交叉等）交换部分基因，生成新的子代个体。
	变异（Mutation）：对新生成的子代个体，按照一定的变异概率和变异方式（如随机变异、非均匀变异等）改变部分基因，以增加种群的多样性。

评估适应度：计算新生成的子代个体的适应度函数值。
	种群更新：将新生成的子代个体与父代个体合并，形成新的种群。根据适应度函数值对种群中的个体进行排序，选择适应度值较高的个体作为下一代的父代。

迭代与终止条件：
	重复执行遗传算法的选择、交叉、变异和评估操作，直到满足终止条件。终止条件可以是达到预设的迭代次数、适应度值满足一定的收敛要求等。

输出最优解：
	当遗传算法终止时，选择适应度函数值最高的染色体（即模糊控制器参数集）作为最优解输出。

验证与优化：
	将得到的最优模糊控制器参数集应用到实际控制系统中，进行验证和优化。根据系统的实际表现调整适应度函数、遗传算法参数等，以进一步提高模糊控制器的性能。
	以上步骤基于遗传算法对模糊控制器进行优化设计，结合了模糊控制理论和遗传算法的全局搜索能力。在实际应用中，可能还需要根据具体问题和需求进行适当的调整和扩展。

*/



/*思路
	定义模糊控制pid，给一个目标变量，通过pid算法使得结果从初始状态逐步达到目标变量，其中初始状态较大时，
	变化快，当趋近目标变量时，变化较慢，即需要根据误差改变pid的三个参数，至于如何改变kp，ki，kd，即采用
	模糊控制器，定义模糊控制集，根据误差，模糊，解模糊，得到具体的pid参数，在其中，模糊控制集如何得到
	最优状态，可采用遗传算法优化，每一个模糊控制集为一条染色体，通过系列算法，择优出最终控制集，代入
	模糊控制器中，进而再得到具体pid参数，使得响应达到快速，稳定，精确的结果。
*/




typedef struct PID {
	float err_last;
	float err_last_last;
	float sum_err;
}PID;

float target_num=0;
float actual = 500;
float pid_num[3] = {0.9,0,0};
float err ,duty;
int index=1;
PID num_pid;
float pid_realize(PID* sprt,float target_num,float actual_num) {

	err = target_num - actual_num;
	//printf("%f\n",err);
	/*if (abs(err) > 200)
	{
		index = 0;
	}
	else {
		index = 1;*/
		sprt->sum_err += err;
	//}
	duty = pid_num[0] * err +index*pid_num[1]*sprt->sum_err + pid_num[2] * (err - sprt->err_last);
	sprt->err_last = err;
//	printf("%f\n", duty);
	actual = duty;
	return actual;

}

int count = 0;
float speed=0;




/**********************模糊pid******************/

//此模型误差指计算过程的误差，且此模型与初始条件无关，关注过程及达到稳态（目标值）的结果，其中的误差、
// ，误差变化率，随误差变化的pid参数即为模糊理论与遗传算法所优化部分

/*
* 模糊控制器流程
* 确定模糊控制器结构：
	+定义模糊控制器的输入和输出变量，并确定它们的模糊集数量。
	为每个模糊集定义隶属度函数（membership functions）。
	设计模糊控制规则，这些规则定义了如何从输入变量的模糊集映射到输出变量的模糊集。

1. 模糊化

2. 模糊推理

3. 去模糊化
*/

int main() {


	float target = 600;
	float actual = 0;
	float e_max = 600;
	float e_min = -600;
	float ec_max = 600;
	float ec_min = -600;
	float kp_max = 0;
	float kp_min = 0;
	float ki_max = 1;
	float ki_min = -1;
	float kd_max = 0;
	float kd_min = 0;
	float erro;
	float erro_c;
	float erro_pre = 0;
	float erro_ppre = 0;
	erro = target - actual;
	erro_c = erro - erro_pre;
	for (int i = 0; i < 100;i++) {

		float u;
		u = fuzzy_pid_control(e_max,e_min,ec_max,ec_min,kp_max,kp_min,erro,erro_c,
			ki_max,ki_min,kd_max,kd_min,erro_pre,erro_ppre);

		actual += u;
		erro_ppre = erro_pre;
		erro_pre = erro;
		erro = target - actual;
		erro_c = erro - erro_pre;
		printf("i:%d         target:%f          actual:%f\n",i,target,actual);
		printf("\n");
		printf("\n");


	}
	
}
