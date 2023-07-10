# balancing-robot
단기 프로젝트로 만든 발란싱로봇 관련 repository입니다.

외력이 들어왔을 때 평형 상태로 제어하기 위해서 AHRS(자세방위시스템)에서 들어온 센싱값과 encoder에서 들어온 각속도 값을 complementary filter로 수정하고 값에 따라 모터의 roll, pitch값을 보상하도록 PID제어를 진행했습니다. 블루투스를 이용한 간단한 방향 제어를 통해 로봇을 이동시킬 수 있습니다.

- PID Adjustment
    
    : 로봇의 모터 상태 및 무게중심에 따라 적절한 PID Values가 다를 수 있으므로 실험값을 통해 세팅.
    
    Parameter
    
    rPID bala(130, 2040, 5, 5000);
    rPID movePID(0.0015, 0.001, 0.00, 2);
    

- improvement
    
    [![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/bbe0e2c7-4e82-47f1-a6ef-32b2b705898f/Untitled.png)](https://file.notion.so/f/s/bbe0e2c7-4e82-47f1-a6ef-32b2b705898f/Untitled.png?id=691f10bd-2b57-4264-b20b-37325f6f8eda&table=block&spaceId=686aa7f9-b24d-4f38-b71b-5bd28ff9cd96&expirationTimestamp=1689120000000&signature=lu5oQjoKTERrM4tv-3DIYSlkF8qHsFxCCFGsi7ukPkM&downloadName=Untitled.png)
    
- control diagram

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/69ed52b0-177e-477d-9ea3-87a2717e4e26/Untitled.png)

- 관련 cumstom code

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/f39ef765-5758-4606-b104-2351360390d9/Untitled.png)

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/89c33ce6-e965-42cf-adb6-b31466f4842e/Untitled.png)

- 관련 영상
  https://youtu.be/4g02npqN3SM
