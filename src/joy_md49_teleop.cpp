#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

class TeleopMD49
{
public:
  TeleopMD49();

  int spin();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& jointState);

  ros::NodeHandle nh_;

  int linear_, angular_, accel_, decel_, centerHead_;
  double l_scale_, a_scale_, ac_scale_, d_scale_, ac_offs_, d_offs_;
  int m_headTiltAxis, m_headPanAxis;
  double m_headTiltScale, m_headPanScale;
  double m_currentHeadTilt, m_currentHeadPan;
  bool m_centerHeadPending;
  bool m_gotFirstAccel, m_gotFirstDecel;
  sensor_msgs::Joy mCurrentJoystickState;
  ros::Publisher vel_pub_;
  ros::Publisher headpt_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber mJointStateSubscriber;
 

};


TeleopMD49::TeleopMD49():
  nh_("teleop"),
  linear_(-1),
  angular_(2),
  accel_(4),
  decel_(3),
  centerHead_(11),
  l_scale_(1.0),
  a_scale_(1.0),
  ac_scale_(-0.5),
  d_scale_(0.5),
  ac_offs_(-1.0),
  d_offs_(-1.0),
  m_headTiltAxis(4),
  m_headPanAxis(3),
  m_headTiltScale(1.57),
  m_headPanScale(1.57),
  m_currentHeadTilt(0),
  m_currentHeadPan(0),
  m_centerHeadPending(false),
  m_gotFirstAccel(false),
  m_gotFirstDecel(false)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_accel", accel_, accel_);
  nh_.param("axis_decel", decel_, decel_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_accel", ac_scale_, ac_scale_);
  nh_.param("scale_decel", d_scale_, d_scale_);
  nh_.param("offset_accel", ac_offs_, ac_offs_);
  nh_.param("offset_decel", d_offs_, d_offs_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  nh_.param("axis_head_pan", m_headPanAxis, m_headPanAxis);
  nh_.param("axis_head_tilt", m_headTiltAxis, m_headTiltAxis);
  nh_.param("scale_head_pan", m_headPanScale, m_headPanScale);
  nh_.param("scale_head_tilt", m_headTiltScale, m_headTiltScale);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  headpt_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &TeleopMD49::joyCallback, this);

  mJointStateSubscriber = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 20, &TeleopMD49::jointStateCallback, this);
}

void TeleopMD49::jointStateCallback(const sensor_msgs::JointState::ConstPtr& jointState)
{
  int c = jointState->name.size();
  for (int i=0;i<c;i++) {
    const std::string &name = jointState->name[i];
    double position = jointState->position[i];
    if (name == "head_pan_joint") {
      m_currentHeadPan = position;
    } else if (name == "head_tilt_joint") {
      m_currentHeadTilt = position;
    }
  }
}

void TeleopMD49::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  mCurrentJoystickState = *joy;
  m_gotFirstAccel = m_gotFirstAccel || ((accel_ != -1) && (mCurrentJoystickState.axes[accel_] != 0));
  m_gotFirstDecel = m_gotFirstDecel || ((decel_ != -1) && (mCurrentJoystickState.axes[decel_] != 0));
  if (mCurrentJoystickState.buttons[centerHead_]) {
    m_centerHeadPending = true;
  }
}


int TeleopMD49::spin() {
  ros::Rate loop_rate(20);

  while(ros::ok()) {
    ros::spinOnce();

    if (mCurrentJoystickState.axes.size() > 0) {

      geometry_msgs::Twist twist;
      twist.angular.z = a_scale_*mCurrentJoystickState.axes[angular_];
      twist.linear.x = 0;
      
      if (linear_ != -1) {
        twist.linear.x += l_scale_*mCurrentJoystickState.axes[linear_];
      }

      if (m_gotFirstAccel) {
        twist.linear.x += ac_scale_*(mCurrentJoystickState.axes[accel_] + ac_offs_);
      }

      if (m_gotFirstDecel) {
        twist.linear.x += d_scale_*(mCurrentJoystickState.axes[decel_] + d_offs_);
      }

      trajectory_msgs::JointTrajectory headJoints;
      trajectory_msgs::JointTrajectoryPoint p;

      double headPanV = mCurrentJoystickState.axes[m_headPanAxis] * m_headPanScale;
      double headTiltV = mCurrentJoystickState.axes[m_headTiltAxis] * m_headTiltScale;

      if ((fabs(headPanV) > 0.01) || (fabs(headTiltV) > 0.01)) m_centerHeadPending = false;

      if (m_centerHeadPending) {
        headJoints.joint_names.push_back("head_pan_joint");
        p.positions.push_back(0);
        headJoints.joint_names.push_back("head_tilt_joint");
        p.positions.push_back(0);
      } else {
        if (fabs(headPanV) < 0.00001) headPanV = 0;
        double headPanP = m_centerHeadPending ? 0 : (m_currentHeadPan + headPanV * loop_rate.expectedCycleTime().toSec());
        headJoints.joint_names.push_back("head_pan_joint");
        p.positions.push_back(headPanP);
        p.velocities.push_back(headPanV*0.6);

        if (fabs(headTiltV) < 0.01) headTiltV = 0;
        double headTiltP = m_centerHeadPending ? 0 : (m_currentHeadTilt + headTiltV * loop_rate.expectedCycleTime().toSec());
        headJoints.joint_names.push_back("head_tilt_joint");
        p.positions.push_back(headTiltP);
        p.velocities.push_back(headTiltV*1.5);
      }

      if (p.positions.size() > 0) {
        p.time_from_start = loop_rate.expectedCycleTime();
        headJoints.points.push_back(p);
        headpt_pub_.publish(headJoints);
      }

      vel_pub_.publish(twist);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }  

  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sodabot_teleop");
  TeleopMD49 teleop_md49;
  teleop_md49.spin();
}
