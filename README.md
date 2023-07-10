# balancing-robot
단기 프로젝트로 만든 발란싱로봇 관련 repository입니다.

외력이 들어왔을 때 평형 상태로 제어하기 위해서 AHRS(자세방위시스템)에서 들어온 센싱값과 encoder에서 들어온 각속도 값을 complementary filter로 수정하고 값에 따라 모터의 roll, pitch값을 보상하도록 PID제어를 진행했습니다. 블루투스를 이용한 간단한 방향 제어를 통해 로봇을 이동시킬 수 있습니다.

- PID Adjustment
    
    : 로봇의 모터 상태 및 무게중심에 따라 적절한 PID Values가 다를 수 있으므로 실험값을 통해 세팅.
    
    Parameter
    
    rPID bala(130, 2040, 5, 5000);
    rPID movePID(0.0015, 0.001, 0.00, 2);
    

- improvement
 ![image](https://github.com/JinkyoJB/balancing-robot/assets/85150616/ece4c714-00d9-4ddd-9267-85eda90c98cd)

    
- control diagram
![image](https://github.com/JinkyoJB/balancing-robot/assets/85150616/2a706cb8-08f9-45cd-83e8-6864dc0aacbd)


- 관련 cumstom code
![image](https://github.com/JinkyoJB/balancing-robot/assets/85150616/1b5b26ba-b954-496f-95e2-67b1b7995269)
![image](https://github.com/JinkyoJB/balancing-robot/assets/85150616/2850c498-5913-4584-9c3e-254f83788fa5)

- 관련 영상
[![Watch the video](https://img.youtube.com/vi/4g02npqN3SM/maxresdefault.jpg)](https://youtu.be/4g02npqN3SM)

