### 单应矩阵

描述共面对应像素点的对应关系
$$
P_2 \ = H \ * \ P_1
$$
opencv已经实现书籍Deeper understanding of the homography decomposition for vision-based control的单应矩阵分解，输出四种情况

matlab代码https://www.cnblogs.com/yongjieShi/p/12202850.html

单应矩阵重投影误差计算：https://blog.csdn.net/a7691176/article/details/101353206

### 本佂矩阵

公式推导：

设空间一点在左相机空间坐标系下坐标为X,由相机模型有，


$$
Z_1*\ u1 =KP_1= K\ * (R_1\ X\ +\ T_1)\\
Z_2*\ u2 =KP_2=K\ * (R_2\ X\ +\ T_2)\\
$$
u1,u2为像素坐标，P1,P2为左右片相机空间坐标系下坐标，R1,T1是世界坐标系到左片相机空间坐标系的转换关系，R2,T2是世界坐标系到右片相机空间坐标系的转换，将左片相机空间坐标转换到右片相机空间坐标系下

左乘K得到：
$$
Z_1* K^{-1}\ u1 =P_1 =  R_1\ X\ +\ T_1\\
Z_2*K^{-1}\ u2 = P_2 =  R_2\ X\ +\ T_2\\
$$
即将像素坐标变换到相机空间坐标系，又以左片相机坐标系为基准则R1=I,T1=0，R2,T2为将左片变换到右片坐标转换矩阵,上式化为：
$$
P_1 =   X\\
P_2=  R_2\ X\ +\ T_2\\
$$
将１代入２得到
$$
P_2 =  R_2\ P_1+\ T_2\\
$$
该式子表达了左右相机空间坐标系下的坐标转换关系，利用这个式子，我们可以得到右片相机透视中心O在左片下的坐标，也就是相机２在全局坐标系下的坐标（摄影测量是先平移再旋转？）：
$$
O=
\begin{pmatrix}
0\\0\\0
\end{pmatrix}
=R_2P_1+T_2\\
P_1=-{R_2}^{-1}T_2=-{R_2}^TT_2
$$
更一般的情况，反解得到：
$$
P_1=  {R_2}^{-1}\ (P_2-\ T_2)= {R_2}^{-1}\ P_2- {R_2}^{-1}\ \ T_2
$$
该式子表示的是假设P1,P2两个坐标系重合，对P2旋转Ｒ｀，再平移Ｔ｀得到其在左片相机空间坐标系下的坐标，所以相机的位姿为：
$$
T=[{R_2}^{-1}\ \ \ \ -{R_2}^{-1}T_2]
$$


引入外积向量:
$$
T_2×P_2=
\begin{bmatrix}
i\ \ \  j \ \  k\\
t1 \ t2\ t3 \\
u1 \ u2\ u3 \\
\end{bmatrix}
=
\begin{bmatrix}
0\  -t3 \  t2\\
-t3 \ 0\ t1 \\
-t2 \ t1\ 0 \\
\end{bmatrix}
\begin{bmatrix}
u1 \\ u2\\ u3 \\
\end{bmatrix}
=\widehat {T_2}P_2
$$
该向量与u1和Ｔ都正交，用该向量与上式做内积得到
$$
0= {(\widehat {T_2}P_2)}^TR_2\ P_1
$$

$$
P_2^T{\widehat {T_2}}^TR_2P_1=0
$$

$$
P_2^TEP_1=0
$$



上式即为对极几何约束，描述在相机空间坐标系下同名点所满足的关系，可以得到本征矩阵为
$$
E=\widehat {T_2}^TR_2=
\begin{bmatrix}
0\  -t3 \  t2\\
-t3 \ 0\ t1 \\
-t2 \ t1\ 0 \\
\end{bmatrix}
^T
* R_2
$$
可以证明
$$
det(\widehat {T_2}) = t1t2t3-t1t2t3=0
$$
而R阵为满秩，因此E的秩为２，E秩亏

将K-1阵代入得
$$
{(K^{-1}u2)}^T\widehat {T_2}^TR_2K^{-1}P_1=0\\
u2^TK^{-T}\widehat {T_2}^TR_2K^{-1}P_1=0\\
F=K^{-T}\widehat {T_2}^TR_2K^{-1}
$$
得到F-基础矩阵，描述给定内参矩阵下左右片同名像素点满足的对极几何约束，一般使用更简洁的本征矩阵，通过该方程求解的是求解方法有八点法和五点法．

SVD分解为



### 讨论：

点共面造成求解的矩阵秩亏？

纯旋转使得Ｔ为０，造成完全秩亏？

opencv recoverPose恢复的是points1到points2的位姿

```c++
CV_EXPORTS_W int recoverPose( InputArray E, InputArray points1, InputArray points2,
                            OutputArray R, OutputArray t,
                            double focal = 1.0, Point2d pp = Point2d(0, 0),
                            InputOutputArray mask = noArray() );
Parameters
E	The input essential matrix.
points1	Array of N 2D points from the first image. The point coordinates should be floating-point (single or double precision).
points2	Array of the second image points of the same size and format as points1 .
cameraMatrix	Camera intrinsic matrix A=⎡⎣⎢fx000fy0cxcy1⎤⎦⎥ . Note that this function assumes that points1 and points2 are feature points from cameras with the same camera intrinsic matrix.
R	Output rotation matrix. Together with the translation vector, this matrix makes up a tuple that performs a change of basis from the first camera's coordinate system to the second camera's coordinate system. Note that, in general, t can not be used for this tuple, see the parameter described below.
t	Output translation vector. This vector is obtained by decomposeEssentialMat and therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit length.
mask	Input/output mask for inliers in points1 and points2. If it is not empty, then it marks inliers in points1 and points2 for then given essential matrix E. Only these inliers will be used to recover pose. In the output mask only inliers which pass the cheirality check.
This function decomposes an essential matrix using decomposeEssentialMat and then verifies possible pose hypotheses by doing cheirality check. The cheirality check means that the triangulated 3D points should have positive depth. Some details can be found in [186].

This function can be used to process the output E and mask from findEssentialMat. In this scenario, points1 and points2 are the same input for
```





景深检核：

cheirality是多视图几何中代表着3D点的正景深约束

相机坐标系以投影方向为ｚ正值，所以ｚ值大于０

https://blog.csdn.net/weixin_39461878/article/details/106366558

Werner T, Pajdla T. Cheirality in epipolar geometry[C]//Proceedings Eighth IEEE International Conference on Computer Vision. ICCV 2001. IEEE, 2001, 1: 548-553.
Hartley R, Zisserman A. Multiple view geometry in computer vision[M]. Cambridge university press, 2003.

### 问题

https://qa.1r1g.com/sf/ask/2259835581/

关于退化情况的讨论

https://blog.csdn.net/qqh19910525/article/details/52240521

https://www.itdaan.com/tw/9dabce4cae05
