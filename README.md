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
 echo -e "source ~/pf_amcl_ws/install/setup.bash\n" >> ~/.bashrc
 echo -e "export TURTLEBOT3_MODEL=burger\n" >> ~/.bashrc
 source ~/.bashrc
 ```

## 사용 방법

### Gazebo 환경 켜기
 ```bash
 ros2 launch pf_amcl muros_world.launch.py
 ```
 #### 에러 발생 시 대처 
 시뮬레이션 환경에 이상이 있거나 로봇이 시뮬레이션 되지 않을때\
 ubuntu 어플리케이션 메뉴에서 System Monitor를 켜 gz 검색 후 gzserver, gzclient 종료 후 재시도

 ![gz_kill](/images/gz_kill.png)\
 spawn entity 관련 오류시 아래 명령어 실행 후 재시도
 ```bash
 . /usr/share/gazebo/setup.sh
 ```

---

### AMCL 실행하기
 ```bash
 ros2 launch pf_amcl amcl.launch.py use_sim_time:=true
 ```

 #### 주의사항
 map을 amcl에 적용시키기 위해서는 지도를 만들때 이름을 map으로 저장해야하며\
 mapping 후 해당 맵에 해당하는 pgm, yaml 파일을 아래 사진 속의 ~/pf_amcl_ws/src/PF_amcl/map로 이동시켜야함(기존에 해당 위치에 있던 map 파일은 제거)\
 경우에 따라 ~/pf_amcl_ws/src/pf_amcl/map일 수도 있음

 ![map_path](/images/map_path.png)
---

### pf 실행하기 
 ```bash
 ros2 run pf_amcl pf_node 0 0 #0,0으로 이동 명령
 ```

## pf 실행 시 parameter 변경 방법

 ```bash
 #실행 예시
 ros2 run pf_amcl pf_node 3 -3 --ros-args -p Q_attraction:=1.0 -p "Q_repulsion:=1.0" -p "max_scan:=100.0"
 ```

 pf 실행 코드에 추가적인 명령어를 덧붙여 parameter를 변경할 수 있음\
 크게 3가지 parameter가 존재

 ### Q_attraction
 목적지가 로봇을 당기는 인력을 조절할 수 있는 parameter\
 Q_attraction이 증가하면 그만큼 인력 역시 강해짐

 ### Q_repulsion
 로봇 주변의 라이다 센서를 통해 감지된 장애물들이 로봇을 미는 힘을 조절할 수 있는 parameter\
 Q_repulsion이 증가하면 그만큼 척력 역시 강해짐

 ### max_scanV
 단위는 m로 로봇 주변 몇 m까지의 장애물을 장애물로 인식할 것인지 조절할 수 있는 parameter\
 max_scan이 5면 로봇 주변 5m 안쪽의 장애물부터의 척력만 받음

 #### 주의! 세 parameter 모두 float형으로 소수점을 찍어야 정상입력 가능

## Q_attraction 그리고 Q_repulsion 관련 식 변경 방법
 관련 식은 모두 ~/pf_amcl_ws/src/pf_amcl/src의 potentialF.cpp 안에 코딩되어 있음\
 총 6개의 함수로 이루어져 있으며 우리는 그 중 2개의 함수만을 수정할 것임
 ### ComputeAttraction_amcl
 Line 183 ~ Line 212
 ```C++
void ComputeAttraction_amcl(float x_a, float y_a)
    {
      //RCLCPP_INFO(this->get_logger(), "GOAL | x : %f | y : %f",x_a,y_a);
      // Compute distance between the attraction and the current position
      float distance =  sqrt(pow(x_a - x_amcl,2) + pow(y_a - y_amcl,2));
      // Compute the point to reach relative to the current position
      x_a = x_a - x_amcl;
      y_a = y_a - y_amcl;

      float F_attraction = 0; 
      /**************************************************/
      // Create the Module of the force to simulate
      if(distance < 5){
        F_attraction = (Q_attraction*100 )/(4 * PI * pow(distance,2));
      }else{
        F_attraction = Q_attraction;
      }
      /**************************************************/

      // Create the position of the force to simulate
      V_attraction = {F_attraction * x_a , F_attraction * y_a};

      //RCLCPP_INFO(this->get_logger(), "x : %f | y : %f",x_a,y_a);
      //RCLCPP_INFO(this->get_logger(), "Force: %f",F_attraction);
      //RCLCPP_INFO(this->get_logger(), "angle attraction :%f°",atan(V_attraction[1]/V_attraction[0])*180/PI);
      //RCLCPP_INFO(this->get_logger(), "v_attraction is : x = %f ; y = %f",x,y);

      geometry_msgs::msg::PoseStamped attraction = PublishVector(V_attraction[0],V_attraction[1]);
      att_pub->publish(attraction);
    }
 ```
 #### 참고사항
 /***/로 감싸인 부분이 목적지를 기준으로 하는 attraction의 크기에 해당하는 식임\
 F_attraction이 벡터의 크기에 해당하고\
 방향은 현재 위치와 목적지를 잇는 방향으로 자동 결정됨

 ### scan_callback
 Line 247 ~ Line 299
 ```C++
void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr _msg)
    {
      float angle_min = _msg->angle_min;
      //float angle_max = _msg->angle_max;
      float step      = _msg->angle_increment; 
      auto scan       = _msg->ranges;
      auto len        = int(float(scan.size()));

      int counter = 0;

      float x_r = 0;
      float y_r = 0;

      float max_d = 3;

      for(int i = 0 ; i < len ; i++)
      {
        // If the value of the scan is < 100m it's not tacking into account
        if(scan[i] < max_d and scan[i] > 0.1)
        { 
          /**************************************************/
          // Create the Module of the force to simulate
          float Current_Q = (Q_repulsion) / (4 * PI * pow(scan[i],2));
          /**************************************************/
          // Projection of the vectors in the x , y coordinates
          x_r -= Current_Q * cos(angle_min+theta_amcl+step*i);
          y_r -= Current_Q * sin(angle_min+theta_amcl+step*i);
        }
        else
        {
          counter += 1;
        }
        
      }
      //RCLCPP_INFO(this->get_logger(), "Counter : %d  ",counter);
      if(counter == 360)
      {
        V_repulsion = {0.0001,0.000000000001};
      }
      else
      {
        //RCLCPP_INFO(this->get_logger(), "x: %f | y: %f",x_r,y_r);
        V_repulsion = {x_r, y_r};
      }
      //RCLCPP_INFO(this->get_logger(), "\n angle repulsion : %f°",atan(V_repulsion[1]/V_repulsion[0])*180/PI);

      // Create the vector for Rviz
      geometry_msgs::msg::PoseStamped repulsion = PublishVector(V_repulsion[0],V_repulsion[1]);
      // Publish the vector
      rep_pub->publish(repulsion);
      // Controller
      controller();
    }
 ```
 #### 참고사항
 /***/로 감싸인 부분이 장애물을 기준으로 발생하는 repulsion에 해당하는 식임\
 Current_Q는 벡터의 크기에 해당함\
 Current_Q는 라이다로 인식된 장애물 중 단 한 점에 의한 repulsion의 크기임\
 <br/> 
 모든 장애물 점들로 인한 Current_Q를 통합한 값은 V_repulsion이라는 이름으로 외부 함수로 전달됨\
 그 후 V_repulsion과 V_attraction는 더해져 로봇을 움직이는 벡터가 결정됨\
 통합되는 과정에서 -가 붙기 때문에 Current_Q는 양의 값이어야함
 
 #### 하단 사이트 이용하여 수식 구성 후 수정하는 것을 권장
 https://www.desmos.com/calculator?lang=ko

 #### parameter를 코드 속에 직접 변경해도 작동함
 아래와 같이 변수로 선언되어 있음
 ```C++
float scan_max = 100;
float Q_repulsion = 1;
float Q_attraction = 1;
 ```

 ### 코드 수정 후 수정 내역 패키지에 적용허는 방법
 아래 코드에 따라 패키지를 다시 빌드해야 변경된 코드가 적용됨
 ```bash
 cd ~/pf_amcl_ws #패키지가 실치된 경로로 이동
 colcon build --symlink-install  #패키지 다시 빌드
 ```

