---
Title: "pf_amcl"
Author: "Muros"
Content: "amcl을 이용한 위치 찾기 && pf를 이용한 길찾기"
---

# pf_amcl

### 에러 발생 시 체크 리스트

1. 실습시간에 사용했던 패키지가 전부 설치되어 있는가?
2. turtlebot3 모델명을 export 해주었는가?

## 설치

### 설치를 위한 디렉토리 만들기

```bash
mkdir -p ~/pf_amcl_ws/src && cd ~/pf_amcl_ws/src 
```

### 패키지 다운로드

```bash
git clone https://github.com/won-jong-wan/pf_amcl.git
```

### 빌드

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
source ~/.bashrc
```

## 사용 방법

### Gazebo 환경 켜기
```bash
export TURTLEBOT3_MODEL=burger #타 모델 선택 가능 #waffle, waffle_pi
ros2 launch pf_amcl go_back_home.launch.py
```

### AMCL 실행하기
```bash
export TURTLEBOT3_MODEL=burger #타 모델 선택 가능 #waffle, waffle_pi
ros2 launch pf_amcl amcl.launch.py
```

### pf 실행하기 
```bash
export TURTLEBOT3_MODEL=burger #타 모델 선택 가능 #waffle, waffle_pi
ros2 run pf_amcl pf_node 0 0 #0,0으로 이동 명령
```

## pf 수정 관련
### potentialF.cpp 열람 방법
```bash
gedit ~/pf_amcl_ws/src/pf_amcl/src/potentialF.cpp #text editer 사용시
code ~/pf_amcl_ws/src/pf_amcl/src/potentialF.cpp #vscode 사용시
``` 
potentialF.cpp 수정 이후 아래 코드를 입력해야 수정 사항이 적용됨
```bash
cd ~/pf_amcl_ws && colcon build --symlink-install
```
### potentialF.cpp 설명

