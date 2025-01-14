
### 4.6
Especially for the joint whose axes is aligned with $\hat{y_s}$:
$$
v = -\omega \times q
$$
$$
v_2 = - \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix} \times \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix},
v_4 = - \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix} \times \begin{bmatrix} W_1 \\ 0 \\ L_1 \end{bmatrix} = \begin{bmatrix} -L_1 \\ 0 \\ -W_1 \end{bmatrix},
v_6 = - \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix} \times \begin{bmatrix} 0 \\ 0 \\ L_1+L_2 \end{bmatrix} = \begin{bmatrix} -(L_1+L_2) \\ 0 \\ 0 \end{bmatrix}
$$
And for other joint:
$$
v_1 = v_3 = v_5 = v_7 = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix}
$$
$$
\cal{S_1} = \begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
0 \\
0 \\
\end{bmatrix}
\cal{S_2} = \begin{bmatrix}
0 \\
1 \\
0 \\
0 \\
0 \\
0 \\
\end{bmatrix}
\cal{S_3} = \begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
0 \\
0 \\
\end{bmatrix}
$$
$$
\cal{S_4} = \begin{bmatrix}
0 \\
1 \\
0 \\
-L_1 \\
0 \\
-W_1 \\
\end{bmatrix}
\cal{S_5} = \begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
0 \\
0 \\
\end{bmatrix}
\cal{S_6} = \begin{bmatrix}
0 \\
1 \\
0 \\
-(L_1+L_2) \\
0 \\
0 \\
\end{bmatrix}
\cal{S_7} = \begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
0 \\
0 \\
\end{bmatrix}
$$

### 4.9
$$
M = \begin{bmatrix}
0 & 1 & 0 & 0 \\
1 & 0 & 0 & 3L \\
0 & 0 & -1 & -2L \\
0 & 0 & 0 & 1
\end{bmatrix}
$$
From left to right, from up to down, we define the joint 1-6.
In $\{0\}$:
$$
v = -\omega \times q
$$
$$
v_1 = - \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \times \begin{bmatrix} 0 \\ 0 \\ -L \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix}
,
\cal{S_1} = \begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
0 \\
0 \\
\end{bmatrix}
$$
$$
v_2 = - \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix} \times \begin{bmatrix} 0 \\ 0 \\ -2L \end{bmatrix} = \begin{bmatrix} 0 \\ -2L \\ 0 \end{bmatrix}
,
\cal{S_2} = \begin{bmatrix}
1 \\
0 \\
0 \\
0 \\
-2L \\
0 \\
\end{bmatrix}
$$
$$
v_3 = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}
,
\cal{S_3} = \begin{bmatrix}
0 \\
0 \\
0 \\
0 \\
1 \\
0 \\
\end{bmatrix}
$$
$$
v_4 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}
,
\cal{S_4} = \begin{bmatrix}
0 \\
0 \\
0 \\
0 \\
0 \\
1 \\
\end{bmatrix}
$$
$$
v_5 = - \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix} \times \begin{bmatrix} 0 \\ L \\ -L \end{bmatrix} = \begin{bmatrix} L \\ 0 \\ 0 \end{bmatrix}
,
\cal{S_5} = \begin{bmatrix}
0 \\
1 \\
0 \\
L \\
0 \\
0 \\
\end{bmatrix}
$$
$$
v_6 = - \begin{bmatrix} 0 \\ 0 \\ -1 \end{bmatrix} \times \begin{bmatrix} 0 \\ 3L \\ -L \end{bmatrix} = \begin{bmatrix} -3L \\ 0 \\ 0 \end{bmatrix}
,
\cal{S_6} = \begin{bmatrix}
0 \\
0 \\
-1 \\
-3L \\
0 \\
0 \\
\end{bmatrix}
$$
In $\{b\}$:
$$
v_1 = - \begin{bmatrix} 0 \\ 0 \\ -1 \end{bmatrix} \times \begin{bmatrix} -3L \\ 0 \\ -L \end{bmatrix} = \begin{bmatrix} 0 \\ -3L \\ 0 \end{bmatrix}
,
\cal{B_1} = \begin{bmatrix}
0 \\
0 \\
-1 \\
0 \\
-3L \\
0 \\
\end{bmatrix}
$$
$$
v_2 = - \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix} \times \begin{bmatrix} -3L \\ 0 \\ 0 \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ -3L \end{bmatrix}
,
\cal{B_2} = \begin{bmatrix}
0 \\
1 \\
0 \\
0 \\
0 \\
-3L \\
\end{bmatrix}
$$
$$
v_3 = \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix}
,
\cal{B_3} = \begin{bmatrix}
0 \\
0 \\
0 \\
1 \\
0 \\
0 \\
\end{bmatrix}
$$
$$
v_4 = \begin{bmatrix} 0 \\ 0 \\ -1 \end{bmatrix}
,
\cal{B_4} = \begin{bmatrix}
0 \\
0 \\
0 \\
0 \\
0 \\
-1 \\
\end{bmatrix}
$$
$$
v_5 = - \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix} \times \begin{bmatrix} -L \\ 0 \\ -L \end{bmatrix} = \begin{bmatrix} 0 \\ -L \\ 0 \end{bmatrix}
,
\cal{B_5} = \begin{bmatrix}
1 \\
0 \\
0 \\
0 \\
-L \\
0 \\
\end{bmatrix}
$$
$$
v_6 = - \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \times \begin{bmatrix} 0 \\ 0 \\ -L \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix}
,
\cal{B_6} = \begin{bmatrix}
0 \\
0 \\
1 \\
0 \\
0 \\
0 \\
\end{bmatrix}
$$
### 5.8
**(a)**
In $\{s\}$:
$$
v = -\omega \times q
$$
$$
v_1 = - \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix} \times \begin{bmatrix} 0 \\ L \\ 0 \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix}
,
\cal{S_1} = \begin{bmatrix}
0 \\
1 \\
0 \\
0 \\
0 \\
0 \\
\end{bmatrix}
$$
$$
v_2 = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}
，
\cal{S_2} = \begin{bmatrix}
0 \\
0 \\
0 \\
0 \\
1 \\
0 \\
\end{bmatrix}
$$
$$
v_3 = - \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \times \begin{bmatrix} 0 \\ 2L \\ 0 \end{bmatrix} = \begin{bmatrix} 2L \\ 0 \\ 0 \end{bmatrix}
，
\cal{S_3} = \begin{bmatrix}
0 \\
0 \\
1 \\
2L \\
0 \\
0 \\
\end{bmatrix}
$$
$$
J_s(\theta)=[\cal{S_1}, [Ad_{\hat{T_1}}]\cal{S_2}, [Ad_{\hat{T_2}}]\cal{S_3}]
$$
$$
\hat{T_1} = e^{[\cal{S_1}]\theta_1},
\hat{T_2} = e^{[\cal{S_1}]\theta_1}e^{[\cal{S_2}]\theta_2}
$$
Calculate by MATLAB code:
```
function A = skew_symmetric(v)
    A = [0, -v(3), v(2);
         v(3), 0, -v(1);
         -v(2), v(1), 0];
end
syms theta1 theta2 L

w_1 = [0; 1; 0];
v_1 = [0; 0; 0];
S_1 = [w_1; v_1];

w_2 = [0; 0; 0];
v_2 = [0; 1; 0];
S_2 = [w_2; v_2];

w_3 = [0; 0; 1];
v_3 = [2*L; 0; 0];
S_3 = [w_3; v_3];

W_1 = skew_symmetric(w_1);
R_1 = eye(3,3) + W_1*sin(theta1) + W_1^2*(1-cos(theta1));
G_1 = eye(3,3)*theta1 + (1-cos(theta1))*W_1 + (theta1 - sin(theta1))*W_1^2;
p_1 = G_1*v_1;
[R_1, p_1]
E_1 = [R_1,p_1;zeros(1,3),1];
AdT_1 = [R_1,zeros(3,3);skew_symmetric(p_1)*R_1,R_1];

W_2 = skew_symmetric(w_2);
R_2 = eye(3,3) + W_2*sin(theta2) + W_2^2*(1-cos(theta2));
G_2 = eye(3,3)*theta2 + (1-cos(theta2))*W_2 + (theta2 - sin(theta2))*W_2^2;
p_2 = G_2*v_2;
E_2 = [R_2,p_2;zeros(1,3),1];
T2 = E_1*E_2;
AdT_2 = [T2(1:3,1:3),zeros(3,3);skew_symmetric(T2(1:3,4))*T2(1:3,1:3),T2(1:3,1:3)];

J = [S_1, AdT_1*S_2, AdT_2*S_3];
J
```
Result:
```
J =
 
[0, 0,                            sin(theta1)]
[1, 0,                                      0]
[0, 0,                            cos(theta1)]
[0, 0,   2*L*cos(theta1) + theta2*cos(theta1)]
[0, 1,                                      0]
[0, 0, - 2*L*sin(theta1) - theta2*sin(theta1)]
```

$$
J_s(\theta) = \begin{bmatrix}
0 & 0 & sin(\theta_1)\\
1 & 0 & 0\\
0 & 0 & cos(\theta_1)\\
0 & 0 & (2L+\theta_2)cos(\theta_1)\\
0 & 1 & 0\\
0 & 0 & -(2L+\theta_2)sin(\theta_1)\\
\end{bmatrix}
$$

**(b)**

$$
v_3 = - \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \times \begin{bmatrix} 0 \\ -L \\ 0 \end{bmatrix} = \begin{bmatrix} -L \\ 0 \\ 0 \end{bmatrix}
,
\cal{B_3} = \begin{bmatrix}
0 \\
0 \\
1 \\
-L \\
0 \\
0 \\
\end{bmatrix}
$$
$$
v_2 = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}
，
\cal{B_2} = \begin{bmatrix}
0 \\
0 \\
0 \\
0 \\
1 \\
0 \\
\end{bmatrix}
$$
$$
v_1 = - \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix} \times \begin{bmatrix} 0 \\ -3L \\ 0 \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix}
，
\cal{B_1} = \begin{bmatrix}
0 \\
1 \\
0 \\
0 \\
0 \\
0 \\
\end{bmatrix}
$$
$$
J_b(\theta)=[[Ad_{\hat{T_2}}]\cal{B_1}, [Ad_{\hat{T_3}}]\cal{B_2}, \cal{B_3}]
$$
$$
\hat{T_3} = e^{-[\cal{B_3}]\theta_3},
\hat{T_2} = e^{-[\cal{B_3}]\theta_3}e^{-[\cal{B_2}]\theta_2}
$$
Or
$$
T_{bs} = \begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 1 & 0 & 3L\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1\\
\end{bmatrix}
$$
$$
J_b(\theta)  = [Ad_{T_{bs}}]J_s(\theta)
$$
$$
J_b(\theta) = 
\begin{bmatrix}
0 & 0 & 0 \\
1 & 0 & 0 \\
0 & 0 & 1 \\
0 & 0 & -L \\
0 & 1 & 0 \\
0 & 0 & 0 \\
\end{bmatrix}
$$
Suppose an external force 
$$
f = 
\begin{bmatrix}
f_x \\
f_y \\
f_z
\end{bmatrix} 
\in \mathbb{R}^3,
$$
which is applied to the $\{b\}$ frame origin. The set of joint torques $\tau$ should satisfy:
$$
\tau = J_b^T(\theta) F_b =
\begin{bmatrix}
0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 1 & 0 & 0 & -L \\
0 & 1 & 0 & 0 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
0 \\
0 \\
0 \\
f_x \\
f_y \\
f_z
\end{bmatrix} =
\begin{bmatrix}
0 \\
f_y \\
-L f_x
\end{bmatrix}.
$$
In this case, there are zero torques from the manipulator, which means
$$
f_y = f_x = 0
$$
The external force can be obtained as
$$
f = 
\begin{bmatrix}
0 \\
0 \\
f_z
\end{bmatrix}
$$
