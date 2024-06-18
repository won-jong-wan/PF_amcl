#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <memory>

#define PI 3.14159265

using std::placeholders::_1;

class PotentialField : public rclcpp::Node
{
  public:
    PotentialField(char* x_goal, char* y_goal)
    : Node("pf_node")

    {
      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
      // Create Subscriber to AMCL pose
      sub_amcl = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 10, std::bind(&PotentialField::amcl_callback, this, _1));
      // Create Subscriber to LiDAR
      sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", default_qos, std::bind(&PotentialField::scan_callback, this, _1));
      // Create Publisher in command control
      cmd_pub  = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
      // Create Publihser of the attraction vector
      att_pub  = this->create_publisher<geometry_msgs::msg::PoseStamped>("attraction_vector",1);
      // Create Publihser of the Replusion vector
      rep_pub  = this->create_publisher<geometry_msgs::msg::PoseStamped>("repulsion_vector",1);
      // Create Publihser of the Final vector
      fin_pub  = this->create_publisher<geometry_msgs::msg::PoseStamped>("Final_Vector",1);

      paramT = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&PotentialField::param_callback, this));

      this->declare_parameter<float>("scan_max", 100.0);
      this->declare_parameter<float>("Q_repulsion", 1.0);
      this->declare_parameter<float>("Q_attraction", 1.0);

      scan_max = this->get_parameter("scan_max").get_value<float>();
      Q_repulsion = this->get_parameter("Q_repulsion").get_value<float>();
      Q_attraction = this->get_parameter("Q_attraction").get_value<float>();

      RCLCPP_INFO(this->get_logger(), "scan_max = %f, Q_repulsion = %f, Q_attraction = %f", scan_max, Q_repulsion, Q_attraction);
      RCLCPP_INFO(this->get_logger(), "x = %s and y = %s", x_goal,y_goal);

      goal_x = atof(x_goal);
      goal_y = atof(y_goal);

    }

    void param_callback()
    {
      s = scan_max, r = Q_repulsion, a=Q_attraction;

      scan_max = this->get_parameter("scan_max").get_value<float>();
      Q_repulsion = this->get_parameter("Q_repulsion").get_value<float>();
      Q_attraction = this->get_parameter("Q_attraction").get_value<float>();

      if(s != scan_max || r != Q_repulsion || a !=Q_attraction){
        RCLCPP_INFO(this->get_logger(), "fixed scan_max = %f, Q_repulsion = %f, Q_attraction = %f", scan_max, Q_repulsion, Q_attraction);
      }

    }

    int controller()
    {


      double x_final = V_attraction[0] + V_repulsion[0];
      double y_final = V_attraction[1] + V_repulsion[1];

      geometry_msgs::msg::PoseStamped finalvector = PublishVector(x_final,y_final);
      
      fin_pub->publish(finalvector);

      geometry_msgs::msg::Twist direction;

      double tolerance = 0.5;
      double angle = atan(y_final/x_final);
      double delta;

      if (x_final < 0)
      {
        angle = PI + atan(y_final/x_final);
      }
      else
      {
        angle = atan(y_final/x_final);
      }
      


      delta = PI - fabs(fmod(fabs(angle - theta_amcl), 2*PI) - PI);

      //RCLCPP_INFO(this->get_logger(), "angle to goal: %f", delta);
 

      RCLCPP_INFO(this->get_logger(), "delta=%f", delta);

      // if(delta > 1.57){
      //    delta = -(PI-delta);
      // }

      double speed = sqrt(pow(x_final, 2)+pow(y_final, 2));
      double distance =  sqrt(pow(goal_x - x_amcl,2) + pow(goal_y - y_amcl,2));

      if(delta> delta_temp && delta < tolerance){
        vel_temp = -vel_temp;
      }

      if(delta > tolerance)
      {
        direction.angular.z = vel_temp*-sqrt(delta);
        direction.linear.x  = 0;
        // v angle +
      }
      else
      {
        // v forward +
        if(distance < 5){
          direction.linear.x  = sqrt(distance/10);
        }
        else if(speed/100 <= 0.4){
          direction.linear.x  = speed/10;
        }else{
          direction.linear.x  = 0.4;
        }
        direction.angular.z = 0;
      }

      cmd_pub->publish(direction);
      delta_temp = delta;
      return 0;
    }

    geometry_msgs::msg::PoseStamped PublishVector(float x, float y)
    {

      // Create the attraction vector to show in RVIZ
      geometry_msgs::msg::PoseStamped vector;
      // Set the frame id
      std::string id_frame = "map";
      vector.header.frame_id = id_frame;
      // Set the time stamp (current time)
      vector.header.stamp = this->get_clock()->now();
      // set the position (it's always (0,0,0) as the reference frame is odom)
      vector.pose.position.x = x_amcl ;
      vector.pose.position.y = y_amcl ;
      vector.pose.position.z = 0 ;
      // Compute the theta angle
      float angle;
      if (x < 0)
      {
        angle = PI + atan(y/x);
      }
      else
      {
        angle = atan(y/x);
      }
      
      // Init variable of quaterion 
      tf2::Quaternion q;
      // Set the quaterion using euler angles
      q.setRPY(0,0,angle);
      // Convert the geometry::quaternion message into pose::quaterion message
      vector.pose.orientation = tf2::toMsg(q);
      // Publish it

      return vector;

    }

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
    

  private:
    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
      // set current x position
      x_amcl = msg->pose.pose.position.x;
      // set current y position
      y_amcl = msg->pose.pose.position.y;

      // Retrive the rotations using quaterions
      tf2::Quaternion q(
              msg->pose.pose.orientation.x,
              msg->pose.pose.orientation.y,
              msg->pose.pose.orientation.z,
              msg->pose.pose.orientation.w);

      // Convert it into matrice of 3x3
      tf2::Matrix3x3 m(q);

      // Init angle variables
      double roll, pitch, yaw;
      // Transform quaterion into Euler
      m.getRPY(roll, pitch, yaw);
      // Define theta = yaw
      theta_amcl = yaw;

      //RCLCPP_INFO(this->get_logger(), "amcl : x = %f | y = %f | theta_amcl = %f" , x_amcl , y_amcl, theta_amcl);
      
      ComputeAttraction_amcl(goal_x,goal_y);
      
    }


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

     

      for(int i = 0 ; i < len ; i++)
      {
        // If the value of the scan is < 100m it's not tacking into account
        if(scan[i] < scan_max and scan[i] > 0.1)
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
    // amcl_pose subsriber variable declaration
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr       sub_amcl;
    // scan subsriber variable declaration
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr   sub_scan;
    // robot control publisher variable declaration
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        cmd_pub;
    // attraction vector publisher variable declaration
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  att_pub;
    // repulsion vector publisher variable declaration
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  rep_pub;
    // repulsion vector publisher variable declaration
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  fin_pub;
    // Declare position

    rclcpp::TimerBase::SharedPtr paramT;

    // x position of odometry
    double x_amcl = 0;
    // y position of odoemtry
    double y_amcl = 0;
    // Angle of the robot(amcl)
    double theta_amcl = 0;
    // Attraction vector
    std::vector<float> V_attraction = {0,0};
    // Replusion vector
    std::vector<float> V_repulsion ;
    //
    float goal_x = 0;
    float goal_y = 0;

    float scan_max = 100;
    float Q_repulsion = 1;
    float Q_attraction = 1;

    float delta_temp = 0;
    float vel_temp = 1;
    float s= 0 , r= 0 , a= 0 ;
};

int main(int argc, char * argv[])
{  
  // init node
  rclcpp::init(argc, argv);
  // init class
  auto node = std::make_shared<PotentialField>(argv[1],argv[2]);
  rclcpp::spin(node);
  // shutdown once finished
  rclcpp::shutdown();
  // end
  return 0;
}
