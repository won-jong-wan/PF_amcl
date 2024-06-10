---
Title: "pf_amcl"
Author: "Muros"
Content: "amcl을 이용한 위치 찾기 && pf를 이용한 길찾기"
---

# pf_amcl

## 설치

### 설치를 위한 디렉토리 만들기

```bash
mkdir -p ~/pf_amcl_ws/src && cd ~/pf_amcl_ws/src 
```

### 패키지 다운로드

```bash
git clone https://github.com/won-jong-wan/pf_amcl.git
```

### 빌드하기

```bash
cd ~/pf_amcl_ws
colcon build --symlink-install
```

빌드 성공 시 아래와 같이 출력됨
```bash
Starting >>> pf_amcl 
Finished <<< pf_amcl [0.18s]                  

Summary: 1 package finished [0.44s]
```

빌드에 성공했다면 이제 아래 명령어만 입력하면 패키지를 사용할 수 있음
```bash
echo >> "source ~/pf_amcl_ws/install/setup.bash" >> ~/.bashrc
echo >> "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```
<span style="color:red">
설치 완료
</span>

## 사용 방법

### Gazebo 환경 켜기
```bash
ros2 launch pf_amcl muros_world.launch.py
```

### AMCL 실행하기
```bash
ros2 launch pf_amcl amcl.launch.py use_sim_time:=true
```

### pf 실행하기 
```bash
ros2 run pf_amcl pf_node 0 0 #0,0으로 이동 명령
```

## pf 실행 시 parameter 변경 방법

```bash
#실행 예시
ros2 run pf_amcl pf_node 3 -3 --ros-args -p Q_attraction:=1.0 -p "Q_repulsion:=1.0" -p "max_scan:=100.0"
```

위와 같이 기존 pf 실행 코드에 추가적인 명령어를 덧붙여 parameter를 변경할 수 있음\
크게 3가지 parameter가 존재

----
![pf_image](/images/pf_rm.png)

### Q_attraction

목적지가 로봇을 당기는 인력을 조절할 수 있는 parameter\
Q_attraction이 증가하면 그만큼 인력 역시 강해짐

### Q_repulsion

로봇 주변의 라이다 센서를 통해 감지된 장애물들이 로봇을 미는 힘을 조절할 수 있는 parameter\
Q_repulsion이 증가하면 그만큼 척력 역시 강해짐

### max_scan

단위는 m로 로봇 주변 몇 m까지의 장애물을 장애물로 인식할 것인지 조절할 수 있는 parameter\
max_scan이 5면 로봇 주변 5m 안쪽의 장애물부터의 척력만 받음

#### 주의! 세 parameter 모두 float형으로 소수점을 찍어야 정상입력 가능

## mapping시 주의사항
```bash
ros2 run nav2_map_server map_saver_cli -f ~/pf_amcl_ws/src/pf_amcl/map
```
기존의 코드가 아닌 위의 코드로 map을 저장해야 AMCL시 열리는 map이 달라짐