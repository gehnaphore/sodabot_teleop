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

  int linear_, angular_;
  double l_scale_, a_scale_;
  int m_headTiltAxis, m_headPanAxis;
  double m_headTiltScale, m_headPanScale;
  double m_currentHeadTilt, m_currentHeadPan;
  bool m_centerHeadPending;
  sensor_msgs::Joy mCurrentJoystickState;
  ros::Publisher vel_pub_;
  ros::Publisher headpt_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber mJointStateSubscriber;
 

};


TeleopMD49::TeleopMD49():
  linear_(1),
  angular_(2),
  l_scale_(1.0),
  a_scale_(1.0),
  m_headTiltAxis(4),
  m_headPanAxis(3),
  m_headTiltScale(1.57),
  m_headPanScale(1.57),
  m_currentHeadTilt(0),
  m_currentHeadPan(0),
  m_centerHeadPending(false)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  nh_.param("axis_head_pan", m_headPanAxis, m_headPanAxis);
  nh_.param("axis_head_tilt", m_headTiltAxis, m_headTiltAxis);
  nh_.param("scale_head_pan", m_headPanScale, m_headPanScale);
  nh_.param("scale_head_tilt", m_headTiltScale, m_headTiltScale);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  headpt_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("head_controller/command", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopMD49::joyCallback, this);

  mJointStateSubscriber = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, &TeleopMD49::jointStateCallback, this);
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
  if (mCurrentJoystickState.buttons[10]) {
    m_centerHeadPending = true;
  }
}


int TeleopMD49::spin() {
  ros::Rate loop_rate(10);

  while(ros::ok()) {
    ros::spinOnce();

    if (mCurrentJoystickState.axes.size() > 0) {
      geometry_msgs::Twist twist;
      twist.angular.z = a_scale_*mCurrentJoystickState.axes[angular_];
      twist.linear.x = l_scale_*mCurrentJoystickState.axes[linear_];

      trajectory_msgs::JointTrajectory headJoints;
      trajectory_msgs::JointTrajectoryPoint p;

      double headPanV = mCurrentJoystickState.axes[m_headPanAxis] * m_headPanScale;
      double headTiltV = mCurrentJoystickState.axes[m_headTiltAxis] * m_headTiltScale;

      if ((fabs(headPanV) > 0.001) || (fabs(headTiltV) > 0.001) || m_centerHeadPending) {
        double headPanP = m_centerHeadPending ? 0 : (m_currentHeadPan + headPanV * loop_rate.expectedCycleTime().toSec());
        headJoints.joint_names.push_back("head_pan_joint");
        p.positions.push_back(headPanP);

        double headTiltP = m_centerHeadPending ? 0 : (m_currentHeadTilt + headTiltV * loop_rate.expectedCycleTime().toSec());
        headJoints.joint_names.push_back("head_tilt_joint");
        p.positions.push_back(headTiltP);

        m_centerHeadPending = false;
      }

      if (p.positions.size() > 0) {
        p.time_from_start = loop_rate.expectedCycleTime();
        headJoints.points.push_back(p);
        headpt_pub_.publish(headJoints);
      }

      printf("az=%lf, lx=%lf\n",twist.angular.z,twist.linear.x);
      vel_pub_.publish(twist);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }  

  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy_md49");
  TeleopMD49 teleop_md49;
  teleop_md49.spin();
}
