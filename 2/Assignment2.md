
## 1

**(a)**
The linear velocity of point C is:
$$[0, 0, 0]^T$$

**(b)**
The linear velocity of point A is:
$$[2v, 0, 0]^T$$

**(c)**
Assume that the center of this cylinder is $D$,The velocity of the body fixed point coincides with $C$ is:
$$
^{O}v_C = ^{O}v_D + ^{O}\omega \times \overrightarrow{DC}
=
\begin{bmatrix} 
v\\ 
0\\ 
0
\end{bmatrix}
+
\begin{bmatrix} 
0 \\ 
\frac{v}{r} \\ 
0
\end{bmatrix}
\times
\begin{bmatrix} 
0 \\ 
0\\ 
-r
\end{bmatrix}
=
\begin{bmatrix} 
0 \\ 
0\\ 
0
\end{bmatrix}
$$
**(d)**
The velocity of the body fixed point coincides with A is:
$$
^{O}v_A = ^{O}v_D + ^{O}\omega \times \overrightarrow{DA}
=
\begin{bmatrix} 
v\\ 
0\\ 
0
\end{bmatrix}
+
\begin{bmatrix} 
0 \\ 
\frac{v}{r} \\ 
0
\end{bmatrix}
\times
\begin{bmatrix} 
0 \\ 
0\\ 
r
\end{bmatrix}
=
\begin{bmatrix} 
2v \\ 
0\\ 
0
\end{bmatrix}
$$

**(e)**
Choose O as the reference point:
$$
\omega = [0,\frac{v}{r},0]^T
$$
$$
^{O}v_O = ^{O}v_D + ^{O}\omega \times \overrightarrow{DO}
=
\begin{bmatrix} 
v\\ 
0\\ 
0
\end{bmatrix}
+
\begin{bmatrix} 
0 \\ 
\frac{v}{r} \\ 
0
\end{bmatrix}
\times
\begin{bmatrix} 
-C_x(t) \\ 
0 \\ 
-r
\end{bmatrix}
=
\begin{bmatrix} 
0 \\ 
0 \\ 
\frac{C_x(t)v}{r}
\end{bmatrix}
$$
$$
\mathcal{V} = [0,\frac{v}{r} ,0, 0, 0, \frac{C_x(t)v}{r}]^T
$$
**(f)**
$$
^{C}X_O=
\begin{bmatrix} 
^{C}R_O & O\\ 
[p]^{C}R_O & ^{C}R_O
\end{bmatrix}
$$
$$
^{C}\mathcal{V} = ^{C}X_O^{O}\mathcal{V}=
\begin{bmatrix} 
1 & 0 & 0 & 0 & 0 & 0\\ 
0 & 1 & 0 & 0 & 0 & 0\\ 
0 & 0 & 1 & 0 & 0 & 0\\
0 & 0 & 0 & 1 & 0 & 0\\
0 & 0 & -C_x(t)  & 0 & 1 & 0\\
0 & C_x(t) & 0 & 0 & 0 & 1\\
\end{bmatrix}
\begin{bmatrix} 
0 \\ 
\frac{v}{r} \\ 
0 \\
0 \\
0 \\
\frac{C_x(t)v}{r}
\end{bmatrix}
=
\begin{bmatrix} 
0 \\ 
\frac{v}{r} \\ 
0 \\
0 \\
0 \\
\frac{2C_x(t)v}{r}
\end{bmatrix}
$$
## 3.21
**(a)**
From the $^{a}T_{b}$:
$$
^{a}p_c=
\begin{bmatrix} 
0 \\ 
800 \\ 
0
\end{bmatrix}
,
^{a}p_b=
\begin{bmatrix} 
-100 \\ 
300 \\ 
500 
\end{bmatrix}
$$

$$
^{a}R_b=
\begin{bmatrix} 
0 & -1 & 0 \\ 
1 & 0 & 0 \\ 
0 & 0 & 1 
\end{bmatrix}
$$
The vector $r$ is:
$$
^{a}r=\overrightarrow{OC}-\overrightarrow{OB}=
^{a}p_c-^{a}p_b=
\begin{bmatrix} 
100 \\ 
500 \\ 
-500 
\end{bmatrix}
$$
$$
^{b}r=^{b}R_a^{a}r= 
\begin{bmatrix} 
500 \\ 
-100 \\ 
-500 
\end{bmatrix}
$$
**(b)**
$^{a}R_c$ can get from the rotation of frame $\{c\}$:
$$
^{a}R_c = 
\begin{bmatrix} 
1 & 0 & 0 \\ 
0 & \frac{\sqrt{3}}{2} & -\frac{1}{2} \\ 
0 & \frac{1}{2} & \frac{\sqrt{3}}{2} 
\end{bmatrix}
$$
$$
^{b}R_c=^{b}R_a\ ^{a}R_c=^{a}R_b^{T}\ ^{a}R_c = 
\begin{bmatrix} 
0 & \frac{\sqrt{3}}{2} & -\frac{1}{2} \\ 
-1 & 0 & 0\\ 
0 & \frac{1}{2} & \frac{\sqrt{3}}{2} 
\end{bmatrix}
$$

Then we can get $^bT_{c}$:
$$
^bT_{c}=
\begin{bmatrix} 
0 & \frac{\sqrt{3}}{2} & -\frac{1}{2} & 500  \\ 
-1 & 0 & 0 & -100 \\ 
0 & \frac{1}{2} & \frac{\sqrt{3}}{2} & -500 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$
## 3.28
The body's angular velocity:
$$
\omega_b = ^{b}R_s^T \omega_s=
\begin{bmatrix} 
3 \\ 
-1 \\ 
-2
\end{bmatrix}
$$ 
## 5.5
**(a)**
$$
^{s}P = \begin{bmatrix} 
L+dsin\theta \\ 
L-dcos\theta \\ 
0
\end{bmatrix}
$$
**(b)**
$$
v_p = \begin{bmatrix} 
d\dot{\theta}cos\theta \\ 
d\dot{\theta}sin\theta \\ 
0
\end{bmatrix}
$$
**(c)**
$$
^{s}T_b=
\begin{bmatrix} 
1 & 0 & 0 & L+dsin\theta\\ 
0 & cos\theta & -sin\theta & L-dcos\theta\\ 
0 & sin\theta & cos\theta & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
$$
**(d)**
$$
\dot{T}_{sb} = \begin{bmatrix} 
-\dot{\theta} \sin \theta & -\dot{\theta} \cos \theta & 0 & d \dot{\theta} \cos \theta \\ 
\dot{\theta} \cos \theta & -\dot{\theta} \sin \theta & 0 & d \dot{\theta} \sin \theta \\ 
0 & 0 & 0 & 0 \\ 
0 & 0 & 0 & 0 
\end{bmatrix}, \quad 
T_{sb}^{-1} = \begin{bmatrix} 
\cos \theta & \sin \theta & 0 & -L (\cos \theta + \sin \theta) \\ 
-\sin \theta & \cos \theta & 0 & -L (\cos \theta - \sin \theta) + d \\ 
0 & 0 & 1 & 0 \\ 
0 & 0 & 0 & 1 
\end{bmatrix}.
$$

$$
[\mathcal{V}_b] = T_{sb}^{-1} \dot{T}_{sb} = \begin{bmatrix} 
0 & -\dot{\theta} & 0 & d \dot{\theta} \\ 
\dot{\theta} & 0 & 0 & 0 \\ 
0 & 0 & 0 & 0 \\ 
0 & 0 & 0 & 0 
\end{bmatrix} \Rightarrow \mathcal{V}_b = \begin{bmatrix} 
0 \\ 
0 \\ 
\dot{\theta} \\ 
d \dot{\theta} \\ 
0 \\ 
0 
\end{bmatrix} = \begin{bmatrix} 
\omega_b \\ 
v_b 
\end{bmatrix}.
$$

**(e)**

$$
[\mathcal{V}_s] = \dot{T}_{sb} T_{sb}^{-1} = \begin{bmatrix} 
0 & -\dot{\theta} & 0 & L \dot{\theta} \\ 
\dot{\theta} & 0 & 0 & -L \dot{\theta} \\ 
0 & 0 & 0 & 0 \\ 
0 & 0 & 0 & 0 
\end{bmatrix} \Rightarrow \mathcal{V}_s = \begin{bmatrix} 
0 \\ 
0 \\ 
\dot{\theta} \\ 
L \dot{\theta} \\ 
-L \dot{\theta} \\ 
0 
\end{bmatrix} = \begin{bmatrix} 
\omega_s \\ 
v_s 
\end{bmatrix}.
$$

**(f)**

$$
[\mathcal{V}_s] = T_{sb} [\mathcal{V}_b] T_{sb}^{-1}.
$$

**(g)** Because:

$$
\dot{p}_P = \dot{p}_{sb}, \quad R_{sb}^{-1} \dot{p}_P = v_b.
$$

**(h)** From:

$$
\dot{R}_{sb} R_{sb}^{-1} = \omega_s, \quad -\omega_s p_P + \dot{p}_P = v_s.
$$

## 5.6
**(a)** Given $$ \theta_1 = t,~\theta_2 = t,~\dot{\theta}_1 = \dot{\theta}_2 = 1 $$

$$
\mathcal{V}_b = J_b(\theta)\dot{\theta}
$$

$$
J_b(\theta) = \begin{bmatrix} \mathcal{V}_{b1}(\theta) & \mathcal{V}_{b2}(\theta) \end{bmatrix}
$$

where:

$$
\mathcal{V}_{b1}(\theta) = \begin{bmatrix} \sin \theta_2 \\ \cos \theta_2 \\ 0 \\ -20 \cos \theta_2 \\ 20 \sin \theta_2 \\ -10 \cos \theta_2 \end{bmatrix}, \quad \mathcal{V}_{b2}(\theta) = \begin{bmatrix} 0 \\ 0 \\ 1 \\ 0 \\ 10 \\ 0 \end{bmatrix}
$$

Thus,

$$
\mathcal{V}_b = J_b(\theta)\dot{\theta} = \begin{bmatrix} \sin t & 0 \\ \cos t & 0 \\ 0 & 1 \\ -20 \cos t & 0 \\ 20 \sin t & 10 \\ -10 \cos t & 0 \end{bmatrix} \begin{bmatrix} 1 \\ 1 \end{bmatrix} = \begin{bmatrix} \sin t \\ \cos t \\ 1 \\ -20 \cos t \\ 20 \sin t + 10 \\ -10 \cos t \end{bmatrix} = \begin{bmatrix} \omega_b \\ v_b \end{bmatrix}
$$

Therefore,

$$
\omega_b = \begin{bmatrix} \sin t \\ \cos t \\ 1 \end{bmatrix}, \quad v_b = \begin{bmatrix} -20 \cos t \\ 20 \sin t + 10 \\ -10 \cos t \end{bmatrix}
$$

**(b)** The linear velocity of the rider in the fixed frame {s} coordinates is $ \dot{p} $:

$$
\dot{p} = R_{sb} v_b
$$

Expanding this:

$$
\dot{p}(t) = \begin{bmatrix} \cos t & 0 & \sin t \\ 0 & 1 & 0 \\ -\sin t & 0 & \cos t \end{bmatrix} \begin{bmatrix} \cos t & -\sin t & 0 \\ \sin t & \cos t & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} -20 \cos t \\ 20 \sin t + 10 \\ -10 \cos t \end{bmatrix}
$$

$$
= \begin{bmatrix} -20 \cos t - 20 \cos t \sin t \\ 10 \cos t \\ 20 \sin t + 10 \sin^2 t - 10 \cos^2 t \end{bmatrix}
$$
