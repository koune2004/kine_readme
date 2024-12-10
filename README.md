## Calculation

### Setup
เราทำการศึกษาเบื้องต้นว่าจริงๆแล้วหุ่นยนต์หมานี้มีท่าทางการเดินเป็นอย่างไร จึงพบว่าจะมีรูปแบบการเดินของสัตว์ โดยเราเน้นไปในรูปแบบที่เป็น Crawl (Walk) และ Trot ดังภาพ

![four_leg_move](pic\four_leg_move.png)

จากนั้นทำการตั้งเฟรมอ้างอิงเบื้องต้นขึ้นมาเพื่อกำหนดเฟรมหลักของหุ่นยนต์และเฟรมของขาแต่ละข้างของหุ่นยนต์ `เนื่องจาก simulate ใน unity ซึ่งใช้แกนด้วยกฎมือซ้าย`
![dog with left hand law axis](pic\dog_with_axis_left_hand_law.png)

### Forward Kinematics
กำหนด

$l_1$ คือ ความยาวของข้อต่อจาก Hip joint ไป Knee joint<br>
$q_1$ คือ คือ มุมข้อต่อของ Hip Joint<br>
$l_2$ คือ ความยาวของข้อต่อจาก Knee joint ไป Foot<br>
$q_2$ คือ คือ มุมข้อต่อของ Knee Joint<br>
FR คือ ขาขวาด้านหน้า<br>
FL คือ ขาซ้ายหน้า<br>
RR คือ ขาขวาด้านหลัง<br>
RL คือ ขาซ้ายด้านหลัง<br>

จากรูป<br>
![FW_simple](pic\FW_simple.png)<br>
จะได้ว่า
```math
x = l_{1}cos(q_{1}) + l_{2}cos(q_{2}+q_{3})\\
z = l_{1}sin(q_{1}) + l_{2}sin(q_{2}+q_{3})
```

เมื่อทำการแปลงเฟรมให้ออกมาอยู่ในรูปแบบของขาจะได้ดังรูป
![FW2leg](pic\FW_leg_frame.png)<br>
```math
x = l_{1}cos(\theta_1+135^\circ) + l_{2}cos(\theta_2+\theta_3+90^\circ)\\
z = l_{1}sin(\theta_1+135^\circ) + l_{2}sin(\theta_2+\theta_3+90^\circ)
```

ทำให้ได้ Transformation Matrix ของสมการ Forward Kinematics ในทุกๆขาของหุ่นยนต์ออกมาดังนี้

```math
T_{[]foot}^{[]} = 
\begin{bmatrix} 1 & 0 & 0 & l_{1}cos(\theta_1+135^\circ) + l_{2}cos(\theta_2+\theta_3+90^\circ)\\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & l_{1}sin(\theta_1+135^\circ) + l_{2}sin(\theta_2+\theta_3+90^\circ)\\
0 & 0 & 0 & 1
\end{bmatrix}
```

กำหนดให้<br>
$S_{FR}^0$ คือ Translation จากจุดกลางของหุ่นไปยังเข่าของขาข้างขวาด้านหน้า <br>
$S_{RR}^0$ คือ Translation จากจุดกลางของหุ่นไปยังเข่าของขาข้างขวาด้านหลัง <br>
$S_{FL}^0$ คือ Translation จากจุดกลางของหุ่นไปยังเข่าของขาข้างซ้ายด้านหน้า <br>
$S_{RL}^0$ คือ Translation จากจุดกลางของหุ่นไปยังเข่าของขาข้างซ้ายด้านหลัง <br>

![di_robot](pic\di_of_robot.png)<br>
จะได้ว่า
```math
S_{FR}^0 = \begin{bmatrix} x_F \\ -y_R \\ 0 \end{bmatrix} \\
S_{RR}^0 = \begin{bmatrix} -x_R \\ -y_R \\ 0 \end{bmatrix} \\
S_{FL}^0 = \begin{bmatrix} x_F \\ y_L \\ 0 \end{bmatrix} \\
S_{RL}^0 = \begin{bmatrix} -x_R \\ y_L \\ 0 \end{bmatrix}
```
สมการสุดท้ายของ Transformation matrix ได้เป็นรูปแบบดังนี้
```math
T_{[]foot}^0 = S_{[]}^0 \times T_{[]foot}^{[]}
```

### Inverse Kinematics
![IK_cal](pic\IK_cal.png)<br>
จะได้ว่า
```math
r = \sqrt{x^2 + z^2} \\[1em]
cos(\theta_2) = \frac{r^2 - l_1^2 - l_2^2}{2l_1l_2} \\[1em]
sin(\theta_2) = \gamma\sqrt{1-cos^2(\theta_2)} \\[1em]
\beta = Atan2(l_2s_2 , l_2c_2 + l_1) \\[1em]
\alpha = Atan2(z,x) \\[1em]
\therefore \quad \theta_1 = \alpha - \beta \quad\quad \theta_2 = Atan2(s_2,c_2)
```

### Control speed
ในส่วนของการควบคุมความเร็วในการเดิน โดยจะรับ input เป็นความเร็ว v 
<span style="color:red;">อาจต้องให้พี่เตงใส่ต่อว่าหลัง input เข้าไปแล้วมันไปยังไงต่อนะฮะ</span>


โดยการสั่งการเดินจะเป็นการสั่งไปที่ Task Space ของเท้าหุ่นและทำการ Inverse Kinematics เพื่อสั่ง Configuration ของขา โดย
```math
\begin{bmatrix} x \\ y \\ z \end{bmatrix} =
\begin{bmatrix} x_{foot} \\ y_{foot} \\ z_{foot} \end{bmatrix}+
\begin{bmatrix} \frac{v}{2}(1-cos(\theta)) \\ 0 \\ -\frac{v}{2}(sin(\theta)) \end{bmatrix}
; 0^\circ \leq \theta \leq 180^\circ \\[1em]

\begin{bmatrix} x \\ y \\ z \end{bmatrix} =
\begin{bmatrix} x_{foot} \\ y_{foot} \\ z_{foot} \end{bmatrix}+
\begin{bmatrix} \frac{v}{2}(1-cos(\theta)) \\ 0 \\ 0 \end{bmatrix}
; 180^\circ < \theta < 360^\circ
```
จากข้างต้นจะได้ path การเคลื่อนที่ของขาดังนี้
![walkpath](pic\walkpath.png)<br>