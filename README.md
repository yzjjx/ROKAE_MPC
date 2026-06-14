# 基于ABA正动力学的关节空间力矩MPC  
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
## 1 Pinocchio库动力学矩阵求解  
### 1.1 前置库安装
首先在windows里面安装Pinocchio，这里可以使用Conda，可以用下面的指令来检查是否安装了Pinocchio，即：`conda list pinocchio`

![alt text](fig/fig_1/conda_list.png)

如果出现上面图的情况，说明没有安装Pinocchio，可以利用pip下载，具体终端命令为:`conda install pinocchio -c conda-forge `,安装过程如下  
首先需要新建一个环境，使用命令`conda create -n mpc_rokae python=3.10 -y`

<div align="center">
    <img src="fig\fig_1\new_env.png">
    <br>
    图 ：对应环境的文件夹
</div>

一定要在新的环境下载

<div align="center">
    <img src="fig\fig_1\install_pinocchio_1.png">
    <br>
    图 ：对应环境的文件夹
</div>

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
### 1.2 动力学解析矩阵生成
为了让最终输出的公式为解析式，最终代码如`src\dynamics_matrix.cpp`所示，在其中需要用到CasADi库（CasADi是一个符号计算和自动代码生成工具），因为在python中这两个库是更加容易使用的，因此该项目的矩阵构建方法是通过python生成相关的库，然后在C++中进行调用计算

首先是在conda环境中安装CasADi库，如下图所示：

<div align="center">
    <img src="fig\fig_1\install_casadi.png">
    <br>
    图 ：下载CasADi
</div>

之后，需要运行代码`python\matrices_generate.py`生成动力学矩阵的函数，生成之后的结果见：

<div align="center">
    <img src="fig\fig_1\generate_C++.png">
    <br>
    图 ：生成C代码与对应的头文件
</div>
 
然后将对应的代码复制到src和include文件夹里面。  
在获得相应的矩阵代码后，可以使用两个程序进行验证，分别是：python\pinocchio_com_tau.py，使用该程序可以利用Pinocchio动力学库的RNEA函数计算最后的关节输出力矩 $\tau$ ；src\dynamics_com_tau.cpp，使用该程序可以利用生成的动力学矩阵函数，来验证最后计算的关节输出力矩 $\tau$ ；验证过程如下两图

<div align="center">
    <img src="fig\fig_1\tau_from_python.png">
    <br>
    图 ：Python+Pinocchio利用RNEA函数计算&tau;
</div>
 
<div align="center">
    <img src="fig\fig_1\tau_from_c.png">
    <br>
    图 ：C++利用CasADi生成的矩阵函数来计算&tau;
</div>
 
可以看到计算结果基本上保持一致，证明通过CasADi生成的矩阵函数是正确的
### 1.3 矩阵A和矩阵B的生成
因为生成解析代码的过程还是Python比较方便，因此在这个雅可比矩阵生成的过程，还是使用python来生成  
首先来尝试生成矩阵dotX的函数，即：

$$
\dot X = \begin{bmatrix}
\dot q \\
M^{-1}(q)[\tau-C(q,\dot q)\dot q-G(q)]
\end{bmatrix}
$$

代码可以参见：`python\generate_dotX.py`  
下面直接进入矩阵A和矩阵B的生成，已知矩阵A和B分别为矩阵dotX对状态变量X的偏导和对控制输出u的偏导，即下面公式：  

$$
A=\frac{\partial \dot X}{\partial X} ,B = \frac{\partial \dot X}{\partial u} 
$$

利用雅可比函数：
```python
A_sym = casadi.jacobian(dotX_sym, X_sym)
```
详细代码可以参见：`python\generate_A_B.py`  
之后将产生的代码文件分别复制到include和src文件夹，自此矩阵需要的矩阵A和矩阵B的求解已经全部完成  
## 2 机器人MPC动力学MATLAB验证
既然已经实现了A矩阵和B矩阵的求解，那么就可以使用MATLAB进行前期验证，这里还是根据之前的代码进行验证，详细代码的解析可以参见仓库：https://github.com/yzjjx/LTV-MPC_MATLAB ，在这里，仅仅需要修改Cartpole_MPC.m的离线预编译部分即可  
### 2.1 MATLAB安装CasADi
首先需要在MATLAB中安装CasADi，安装过程如下  
需要在CasADi的GitHub Releases页面找到最新的稳定版本，链接为：https://github.com/casadi/casadi/releases ，如图

 
<div align="center">
    <img src="fig\fig_2\MATLAB_casadi_install.png">
    <br>
    图 ：下载适用于MATLAB的CasADi
</div>

解压文件夹，然后在MATLAB里面添加路径，如图 所示

<div align="center">
    <img src="fig\fig_2\Casadi_MATLAB_install.png">
    <br>
    图 ：下载适用于MATLAB的CasADi
</div>

最后，运行测试代码进行测试，看输出是否正确即可，如图 

<div align="center">
    <img src="fig\fig_2\test_code.png">
    <br>
    图 ：下载适用于MATLAB的CasADi
</div>

最后测试没加入重力补偿的代码效果如图 

<div align="center">
    <img src="fig\fig_2\test1.jpg">
    <br>
    图 ：代码运行结果(不加重力补偿)
</div>

使用CasADi在MATLAB里面加载生成的C代码