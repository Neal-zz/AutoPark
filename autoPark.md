
<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->

<!-- code_chunk_output -->

- [运动学算法](#运动学算法)
  - [正向运动学](#正向运动学)
  - [逆向运动学](#逆向运动学)
- [轨迹生成](#轨迹生成)
- [在控制模型中引入积分器](#在控制模型中引入积分器)

<!-- /code_chunk_output -->

# 运动学算法

## 正向运动学

![参数定义](/img/kinematic1.png)

正向运动学关系：

$$\begin{bmatrix}
\dot{x}\\
\dot{y}\\
\dot{\theta}
\end{bmatrix} = 
\begin{bmatrix}
{\cos\theta} & 0 \\
{\sin\theta} & 0 \\
0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
{v_c}  \\
{\dot{\theta}} \\
\end{bmatrix} = 
\begin{bmatrix}
{\cos\theta} & 0 \\
{\sin\theta} & 0 \\
0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
{1/2} & {1/2} \\
{1/l} & {-1/l} \\
\end{bmatrix}
\begin{bmatrix}
{v_r}  \\
{v_l} \\
\end{bmatrix}$$

## 逆向运动学

![逆向运动学](/img/kinematic2.png)

逆向运动学流程：

每一步计算 linErr 和 angErr；比例控制 linVel 和 angVel；再逆向运动学求解 vr 和 vl 控制运动。

> 存在问题：位置到位之后，姿态不一定到位。

> 解决方法：生成一条与目标姿态相切的轨迹，让小车跟随轨迹运动，这样一来当小车到达目标位置后，姿态也基本准确。

## 参数设置

* wheelsRadius = 50; // r = 50 mm
* wheelsDistance = 560; // l = 560 mm
* linVelLim = 50; // vlim = 50 mm/s
* linAccLim = 20; // alim = 20 mm/s^2
* angVelLim = 0.1; // $\dot{\theta}$lim = 0.1 rad/s
* angAccLim = 0.04; // $\ddot{\theta}$lim = 0.04 rad/s^2
* communFreq = 10; // 10 Hz

# 轨迹生成

整体思路：原始位姿 $iniPose=[x^i,y^i,\theta^i]$，此处设为 $[-1,1.5,0]$；目标位姿 $finalPose=[x^f,y^f,\theta^f]$，此处设为 $[0,0,-80 \pi / 180]$。不要求轨迹与原始姿态相切，因此总共有两个位置约束，和一个角度约束。可以解出两条抛物线方程（对称轴分别平行 x 轴和 y 轴），和多条三次样条曲线。

| 抛物线轨迹 | 三次样条曲线 |
|---|---|
|![抛物线](/img/parabolic.png)|![三次样条](/img/cubic.png)|

以上生成的轨迹称为 desired path，由于轨迹的初始方向与台车的初始姿态不一致，因此台车需要一段路程的调整才能跟随上 desired path。

对每条 desired path 选取 20 个路标点 signpost。台车连续跟踪这 20 个 signpost，直至抵达目标位置。台车在仿真过程中走过的真实轨迹称为 factual path。最后从仿真结果中选取一条长度合理，姿态准确的路径作为最终的路径。

|生成的 factual path|||
|---|---|---|
|![案例1](/img/pathFollow1.png)|![案例2](/img/pathFollow2.png)|![案例3](/img/pathFollow3.png)|

> 如果初始时刻先让台车原地旋转，使其姿态与轨迹初始方向相切，那么往往能得到比较好的跟随效果

# 在控制模型中引入积分器

令 $z=[x+s\cos\theta, y+s\sin\theta]^T$，$u=[v, \omega]^T$，假设存在一个常量扰动，记为 d，有，

$$\begin{bmatrix}
\dot{z}\\
\dot{\theta}
\end{bmatrix}=
\begin{bmatrix}
\cos\theta & -s\sin\theta\\
\sin\theta & s\cos\theta\\
0 & 1
\end{bmatrix}
\begin{bmatrix}
u+d
\end{bmatrix}=
A
\begin{bmatrix}
u+d
\end{bmatrix}
$$

记 $\tilde{z}(t):=z(t)-z^d(t)$，其中上标 d 代表 desired。引入积分器 $\rho$ 来估计扰动 d，即令，

$$u=A^{-1}(-k_p\tilde{z}+\dot{z}^d)-\rho$$

$$\dot{\rho}=k_iA^T\tilde{z}$$

由此可以推出，

$$\begin{bmatrix}
\dot{\tilde{z}}\\
\dot{\rho}
\end{bmatrix}=
\begin{bmatrix}
-k_p I_2 & -k_i A\\
k_i A^T & 0_2
\end{bmatrix}
\begin{bmatrix}
\tilde{z} \\
k_i^{-1} \tilde{\rho}
\end{bmatrix}$$

其中 $\tilde{\rho}=\rho-d$。
