基于ABA正动力学的关节空间力矩MPC  
基于珞石SR4六轴协作机器人的MPC模型预测控制  
各个关节采用力矩控制，即控制量为 $\tau$  
在控制过程中，需要用到机器人完整动力学方程，即

$$
M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=\tau
$$

状态变量：

$$
q,\dot{q}
$$

ABA正动力学作用：  

$$
(q,\dot{q},\tau)\Longrightarrow \ddot q
$$

在力矩控制中， $\ddot q$ 不是独立的状态，而是由ABA动力学方程计算出来的结果  
中间过程：  
首先需要轨迹规划器用来规划轨迹，得到对应的 $q$ 和 $\dot{q}$  ，MPC需要求解最优的 $\tau$ ，为了知道这个 $\tau$ 作用后下一步机器人的状态 $q_{k+1},\dot{q}_{k+1}$ ,需要用到机械臂的动力学方程 $M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=\tau$ ，所以在给定上一时刻的力矩 $\tau_k$ 后，必须通过ABA算法先计算 $\ddot{q}_k$ ，然后根据离散积分来推测下一时刻的机器人状态 $q_{k+1},\dot{q}_{k+1}$   
类比倒立摆，也就是在定义状态变量X后，还要对其求导，得到非线性状态空间方程；也就相当于线性状态空间方程中的 $\dot{x}(t)=Ax(t)+Bu(t)$ 公式一个道理  
同时，在控制频率很高时，可以使用上一时刻的控制指令 $u_{k-1}$ ,来输入一阶泰勒展开公式，来进行非线性方程的线性化，即：

$$
\dot{X}\approx f(X_k,u_{k-1})+A_k(X-X_k)+B_k(u-u_{k-1})
$$

然后通过这个函数求解出来   
注意：该代码前期在Windows编程，后期将迁移到Linux中  
## 文件定义
* scr：源代码文件  
    src\dynamics_matrix.cpp:用来生成机器人动力学矩阵，包括惯性矩阵 $M(q)$ ，科里奥利力与离心力矩阵 $C(q,\dot q)$ ,重力矢量 $G(q)$
* test：测试文件  
    test\test_dynamics.cpp:用来测试Pinocchio是否配置正确
* python：python代码文件
    
************
在利用雅可比矩阵求解矩阵 $A_k、B_k$ 时，使用C++库Pinocchio和CasADi库，其中，利用Pinocchio库来进行矩阵的获取，因为在理想情况下有下面公式：  

$$
\dot X=\begin{bmatrix}
 \dot q\\
\ddot q
\end{bmatrix}=\begin{bmatrix}
\dot q \\
ABA(q,\dot q,\tau)
\end{bmatrix}
$$

但是在求解矩阵的时候，我希望用现成的库函数进行计算，即Pinocchio机器人动力学库，然后再使用CasADi求解出雅可比解，此时公式可以写为：  

$$
\dot X=\begin{bmatrix}
 \dot q\\
\ddot q
\end{bmatrix}=\begin{bmatrix}
\dot q \\
M^{-1}(q)(\tau-C(q,\dot q)\dot q -g(q))
\end{bmatrix}
$$

其中，矩阵 $M^{-1}(q),C(q,\dot q),g(q)$ 可以通过Pinocchio求解得到  
因此，首先来进行这三个动力学矩阵的求解  
## 1、Pinocchio库动力学矩阵求解  
首先在windows里面安装Pinocchio，这里可以使用Conda，可以用下面的指令来检查是否安装了Pinocchio，即：`conda list pinocchio`

![alt text](fig/fig_1/conda_list.png)

如果出现上面图的情况，说明没有安装Pinocchio，可以利用pip下载，具体终端命令为:`conda install pinocchio -c conda-forge `,安装过程如下  
首先需要新建一个环境，使用命令`conda create -n mpc_rokae python=3.10 -y`

![alt text](fig\fig_1\new_env.png)

一定要在新的环境下载

![alt text](fig\fig_1\install_pinocchio_1.png)

注意：千万不要在base环境里面直接`conda install pinocchio -c conda-forge `，因为这样会检索base环境里面所有的包，速度非常慢，一直卡在solving里面，一定要新建一个环境，并且千万不能使用pip安装，pip安装的是不对的Pinocchio库

在下载完成之后，可以打开对应环境的文件夹查看是否安装到这个环境上面，同时这个文件夹的地址也会用到CMake文件中

<div align="center">
    <img src="fig\fig_1\pinocchio_cmake.png">
    <br>
    图 ：对应环境的文件夹
</div>

相关的CMake configure如下图所示：

<div align="center">
    <img src="fig\fig_1\Cmake_configure.png">
    <br>
    图 ：对应的CMake Configure
</div>

在这一步完成后，可以用代码`test\test_dynamics.cpp`进行验证，该代码为AI生成

为了让最终输出的公式为解析式，最终代码如`src\dynamics_matrix.cpp`所示，在其中需要用到CasADi库，因为在python中这两个库是更加容易使用的，因此该项目的矩阵构建方法是通过python生成相关的库，然后在C++中进行调用计算

首先是在conda环境中安装CasADo库，如下图所示：

<div align="center">
    <img src="fig\fig_1\install_casadi.png">
    <br>
    图 ：下载CasADi
</div>


