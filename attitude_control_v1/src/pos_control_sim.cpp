#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "math.h"
#include "tf/transform_datatypes.h"
#include "aruco_msgs/MarkerArray.h"

double freq = 100;
ros::Publisher guide_pub;
ros::Publisher pub;//�ȤäƤʤ�
/*current_position*/
float marker_pos_x;
float marker_pos_y;
float marker_t;//�ޡ����ǰ��֤�����׻�����
float imu_pos_x;
float imu_pos_y;
float imu_t;//imu�ǰ��֤�����׻�����
double theta_c;
double phi_c;
double yaw_c;
/*position��buffer*/
float diff_x[2];
float diff_y[2];
/*��ʬ��*/
double integral_x, integral_y;
/*���ʰ��ָ��л���*/
/*Ϣ³����1s����Ǹ��Ф����marker��pos��Ȥ�*/
ros::Time marker_1stTime;//����Ū�Ǥʤ����Ȥ����
ros::Time marker_2ndTime;
ros::Time marker_timer1;
ros::Time marker_timer2;
double wait_time;
double marker_del_t;
double del_t;//DELTA T
/*attitude�λ�����*/
mavros_msgs::AttitudeTarget guide_msg;//mavros��publish�������
/*marker�򸡽Ф��Ƥ��뤫�Υե饰*/
int marker_succeeded;

/*int id;
float pos_x, pos_y, pos_z;
float quat_x, quat_y, quat_z, quat_w;
float eul_x, eul_y, eul_z;
int cnt = 0;
float vel_x, vel_y;
float pi = 3.1415;
float theta_x, theta_y;*/
/*global����������ѿ�*/
/*target*/
float target_pos_x, target_pos_y, target_pos_z;
/*current*/
float curr_pos_x, curr_pos_y, curr_pos_z;
//float curr_quat_x, curr_quat_y, curr_quat_z, curr_quat_w;
//float curr_eul_x, curr_eul_y, curr_eul_z;

/*******************************�ؿ����*********************************
 *Subscribe�����оݤΥȥԥå����������줿��ƤӽФ���륳����Хå��ؿ�(Cb)
 *�����ˤϥȥԥå���Publish������å������η���Ʊ������������뤳��
 ***********************************************************************/
/*======================================================================
  marker�򸡽Ф����Ȥ����߰��֤���marker���Ȥ���ɸ���֤ذ�ư����position��
  ����
 =======================================================================*/
void marker_Callback(const aruco_msgs::MarkerArray::Ptr& msg){
  int id;
  /*target*/
  //float target_pos_x, target_pos_y, target_pos_z;
  /*current*/
  //float curr_pos_x, curr_pos_y, curr_pos_z;
  float curr_quat_x, curr_quat_y, curr_quat_z, curr_quat_w;
  float curr_eul_x, curr_eul_y, curr_eul_z;
  /*���ʰ��ּ��������򥰥��Х��ѿ��˳�Ǽ*/
  marker_2ndTime = marker_1stTime;
  marker_1stTime = ros::Time::now();
  /*��¬���ϻ���*/
  marker_timer1 = ros::Time::now();
  /*�����*/
  ros::NodeHandle n;
  //MarkerID���ͤ�id�˳�Ǽ����
  id = msg->markers[0].id;
  //MarkerPosition���ͤ�pos_x,pos_y,pos_z�˳�Ǽ����
  curr_pos_x = msg->markers[0].pose.pose.position.x;
  curr_pos_y = msg->markers[0].pose.pose.position.y;
  curr_pos_z = msg->markers[0].pose.pose.position.z;
  //MarkerPoseQuaternion���ͤ�quaternion��x,y,z�˳�Ǽ����
  curr_quat_x = msg->markers[0].pose.pose.orientation.x;
  curr_quat_y = msg->markers[0].pose.pose.orientation.y;
  curr_quat_z = msg->markers[0].pose.pose.orientation.z;
  curr_quat_w = msg->markers[0].pose.pose.orientation.w;
  //MarkerPoseEuler���ͤ�euler��x,y,z�˳�Ǽ����
  //curr_eul_x = msg->epose.x;
  //curr_eul_y = msg->epose.y;
  //curr_eul_z = msg->epose.z;
  std::cout << "-----------------------------------" << std::endl;
  //MarkerID��ɽ��
  std::cout << "Marker        ID: " << id << std::endl;
  //MarkerPosition��ɽ��
  std::cout << "MarkerPosition x: " << curr_pos_x << std::endl;
  std::cout << "               y: " << curr_pos_y << std::endl;
  std::cout << "               z: " << curr_pos_z << std::endl;
  //MarkerPoseQuaternion��ɽ��
  //std::cout << "MarkerPoseQuaternion x: " << qposex << std::endl;
  //std::cout << "                     y: " << qposey << std::endl;
  //std::cout << "                     z: " << qposez << std::endl;
  //std::cout << "                     w: " << qposew << std::endl;
  //MarkerPoseEuler��ɽ��
  //std::cout << "MarkerPoseEuler      x: " << eposex << std::endl;
  //std::cout << "                     y: " << eposey << std::endl;
  std::cout << "             yaw: " << curr_eul_z << std::endl << std::endl;
  /*id�ˤ�äƽ������ѹ�����*/
  if(id == 1000){
    /*target position�λ���*/
    target_pos_x = 0.0;
    target_pos_y = 0.0;
  }
  /*~~~~~~~~~~marker���м����μ���~~~~~~~~~~~~~~~~~~~~~~~~*/
  ros::Duration dura1(marker_1stTime - marker_2ndTime);//ros::TimeƱ�Τκ���ros::Duration��
  marker_del_t = dura1.toSec();
  // ROS_INFO("marker_del_t: [%lf]\n\t", marker_del_t);
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~/
  /*position������*/
  /*diff_x,y[1] integral_x,y diff_x,y[1]-diff_x,y[0]/del_t */
  diff_x[0] = diff_x[1];
  diff_x[1] = target_pos_x - curr_pos_x;
  integral_x += ((diff_x[1] + diff_x[0]) / 2.0)* del_t;
  diff_y[0] = diff_y[1];
  diff_y[1] = target_pos_y - curr_pos_y;
  integral_y += ((diff_y[1] + diff_y[0]) / 2.0)* del_t;
  /*mavros��publish����topic������*/
  /*std::cout << "orientation x: " << guide_msg.orientation.x << std::endl;
  std::cout << "            y: " << guide_msg.orientation.y << std::endl;
  std::cout << "            z: " << guide_msg.orientation.z << std::endl;
  std::cout << "            w: " << guide_msg.orientation.w << std::endl;
  std::cout << "body_rate   x: " << guide_msg.body_rate.x << std::endl;
  std::cout << "            y: " << guide_msg.body_rate.y << std::endl;
  std::cout << "            z: " << guide_msg.body_rate.z << std::endl;
  std::cout << "thrust       : " << guide_msg.thrust << std::endl << std::endl;*/
  /**/
}
/*======================================================================
 * mavros����angle&angvel&accel�������긽�߰��֤������ʾ�����ߤ������
 * �����ͤ���� 
 *=======================================================================*/
void imu_data_Callback(const sensor_msgs::Imu::ConstPtr& msg){
  float quat_x, quat_y, quat_z, quat_w;
  double imu_roll, imu_pitch, imu_yaw;
  float angvel_x, angvel_y, angvel_z;
  float accel_x, accel_y, accel_z;
  mavros_msgs::AttitudeTarget msgxx;
  ros::NodeHandle n;
  quat_x = msg->orientation.x;
  quat_y = msg->orientation.y;
  quat_z = msg->orientation.z;
  quat_w = msg->orientation.w;
  //angular_velosity���ͤ�angvelx,angvely,angvelz�˳�Ǽ����
  angvel_x = msg->angular_velocity.x;
  angvel_y = msg->angular_velocity.y;
  angvel_z = msg->angular_velocity.z;
  //linear_acceleration���ͤ�accelx,accely,accelz�˳�Ǽ����
  accel_x = msg->linear_acceleration.x;
  accel_y = msg->linear_acceleration.y;
  accel_z = msg->linear_acceleration.z;
  //����
  msgxx.orientation.x = 0.0;
  msgxx.orientation.y = 0.0;
  msgxx.orientation.z = 0.0;
  msgxx.orientation.w = 0.0;
  msgxx.body_rate.x = 0.0;
  msgxx.body_rate.y = 0.0;
  msgxx.body_rate.z = 0.0;
  msgxx.thrust = 1.0;
  /*���������˥���򥪥��顼�Ѥ��Ѵ�*/
  tf::Quaternion imu_quat(quat_x, quat_y, quat_z, quat_w);
  tf::Matrix3x3 m(imu_quat);
  m.getRPY(imu_roll, imu_pitch, imu_yaw);
  /*���߻����Ѥ򥰥��Х��ѿ��˳�Ǽ*/
  theta_c = imu_roll;
  phi_c = -imu_pitch;//����碌
  yaw_c = -imu_yaw;//����碌
  /*debag�Ѥ�������*/
  double debug_roll, debug_pitch, debug_yaw;
  debug_roll = imu_roll;
  debug_pitch = imu_pitch;
  debug_yaw = imu_yaw;
  tf::Matrix3x3 debug;
  debug.setEulerYPR(debug_yaw,debug_pitch,debug_roll);
  tf::Quaternion debug_q;
  debug.getRotation(debug_q);
  double qx,qy,qz,qw;
  qx = debug_q.getX();
  qy = debug_q.getY();
  qz = debug_q.getZ();
  qw = debug_q.getW();
  
  ROS_INFO("qx: [%lf] qy: [%lf] qz: [%lf] qw: [%lf]\n", qx, qy, qz, qw);
  /*debag�Ѥ����ޤ�*/
  //orientation��ɽ��
  //ROS_INFO("Imu Orientation\n\t x: [%f]\n\t y: [%f]\n\t z: [%f]\n\t w: [%f]", quat_x, quat_y, quat_z, quat_w);
  //angular_velocity��ɽ��
  //ROS_INFO("Imu Angular_velocity\n\t x: [%f]\n\t y: [%f]\n\t z: [%f]", angvel_x, angvel_y, angvel_z);
  //linear_acceleration��ɽ��
  //ROS_INFO("Imu linear_acceleration\n\t x: [%f]\n\t y: [%f]\n\t z: [%f]\n\t",accel_x, accel_y, accel_z);
  //debug��printf
  //ROS_INFO("%f : %f : %f\n", imu_roll, imu_pitch, imu_yaw);
  /*marker���Хե饰��Ƚ��*/
  /*msgxx���ͤ򥰥��Х��guide_msgs�˳�Ǽ����*/
  /*guide_msg.orientation.x = 0.0;
  guide_msg.orientation.y = 0.0;
  guide_msg.orientation.z = 0.0;
  guide_msg.orientation.w = 0.0;
  guide_msg.body_rate.x = 0.0;
  guide_msg.body_rate.y = 0.0;
  guide_msg.body_rate.z = 0.0;
  guide_msg.thrust = 1.0;*/
}/***************************�ؿ�����Ϥ����ޤ�****************************/

int main(int argc,char **argv){
  int cnt=0;
  /*del_t�ν����(�Ϥ����marker�ʤ�)*/
  del_t = 1.0;
  /*ROS�ν����*/
  ros::init(argc, argv, "attitude_control_v1");
  /*�Ρ��ɤν�����ȥ꥽�����Υ��ꥢ*/
  ros::NodeHandle n;
  guide_pub = n.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",2000);
  /*�¹Լ��������*/
  ros::Rate loop_rate(freq);
  /*ʣ���Υ���åɤ��饳����Хå���ƤӽФ�spinner�����*/
  ros::AsyncSpinner spinner(2);
  /*�������������ǡ��������*/
  //mavros_msgs::AttitudeTarget guide_msg;
  /*/mavros/setpoint_raw/attitude�ν����*/
  guide_msg.orientation.x = 0.0;
  guide_msg.orientation.y = 0.0;
  guide_msg.orientation.z = 0.0;
  guide_msg.orientation.w = 1.0;
  guide_msg.body_rate.x = 0.0;
  guide_msg.body_rate.y = 0.0;
  guide_msg.body_rate.z = 0.0;
  guide_msg.thrust = 0.5;
  /*�ѿ����*/
  /*��ɸ����Ƴ�Х�����*/
  double p = 0.02;
  double i = 0.0;
  double d = 0.0;
  double kp = 1.0;
  double ki = 0.0;
  double kd = 0.0;
  double roll;
  double pitch;
  double yaw;
  double theta_d;
  double phi_d;
  double yaw_d;
  double r_ang_diff[2];
  double r_ang_integral = 0.0;
  double p_ang_diff[2];
  double p_ang_integral = 0.0;
  double y_ang_diff[2];
  double y_ang_integral = 0.0;  
  ros::Subscriber marker_sub = n.subscribe("/aruco_marker_publisher/markers", 2000, marker_Callback);   
  ros::Subscriber imu_sub = n.subscribe("/mavros/imu/data", 2000, imu_data_Callback);
  /*�¥롼��*/
  while(ros::ok()){
    /*delta_t�μ���*/
    del_t = marker_del_t;
    /*���֤�����Ǥ���ޤǤ��Ԥ�����*/
    marker_timer2 = ros::Time::now();
    ros::Duration dura2(marker_timer2 - marker_timer1);
    wait_time = dura2.toSec();
    /*marker���л��֤�������Ѥ������Τʤ�Ȥ�*/
    /*Ƚ����ϼ���marker��Xsec�δ֤˸��ФǤ������*/
    if(ros::Duration(wait_time) >= ros::Duration(0.2)){
      /*������ˡ*/
      roll = 0.0;
      pitch = 0.0;
      yaw = 0.0;
      ROS_INFO("no_marker");
    }else{
      /****************************��ɸ����������********************************/
      /*roll*/
      theta_d = p * diff_x[1] + i * integral_x + d * ( diff_x[1] - diff_x[0]) / del_t;//���֤ˤ�äƷ����������碌��
      if(theta_d > 0.1745)
	theta_d = 0.1745;//+10deg�ʾ�Ϸ����ʤ�
      if(theta_d < -0.1745)
	theta_d = 0.1745;//-10deg�ʾ�Ϸ����ʤ�
      /*pitch*/
      phi_d   = p * diff_y[1] + i * integral_y + d * ( diff_y[1] - diff_y[0]) / del_t;
      if(phi_d > 0.1745)
	phi_d = 0.1745;//10deg�ʾ�Ϸ����ʤ�
      if(phi_d < -0.1745)
	phi_d = -0.1745;//10deg�ʾ�Ϸ����ʤ�
      /*yaw head direction����ɸ���̤�����*/
      yaw_d   = 0.0;
      /**************************��ɸ�����������Ϥ����ޤ�************************/   
      /*****************************��������ȥ���****************************/
      /*roll*/
      r_ang_integral = 0.0;
      r_ang_diff[0] = r_ang_diff[1];
      r_ang_diff[1] = theta_d - theta_c;
      r_ang_integral = r_ang_integral + ((r_ang_diff[1] + r_ang_diff[0]) / 2.0 ) * 1/freq;
	  
      roll =kp * r_ang_diff[1] + ki * r_ang_integral + kd * (r_ang_diff[1] - r_ang_diff[0]) / freq;//����碌
      /*pitch*/
      p_ang_integral = 0.0;
      p_ang_diff[0] = p_ang_diff[1];
      p_ang_diff[1] = phi_d - phi_c;
      p_ang_integral += (p_ang_diff[1] + p_ang_diff[0]) / 2.0 * freq;
      
      pitch =-( kp * p_ang_diff[1] + ki * p_ang_integral + kd * (p_ang_diff[1] - p_ang_diff[0]) / freq);//����碌
      /*yaw*/
      y_ang_integral = 0.0;
      y_ang_diff[0] = y_ang_diff[1];
      y_ang_diff[1] = yaw_d - yaw_c;
      y_ang_integral += (y_ang_diff[1] + y_ang_diff[0]) / 2.0 * freq;
      
      yaw =-( kp * y_ang_diff[1] + ki * y_ang_integral + kd * (y_ang_diff[1] - y_ang_diff[0]) / freq);
      //ROS_INFO("roll_d: [%lf]  pitch_d: [%lf]  yaw_d: [%lf]\n", theta_d, phi_d, yaw_d);//debug
      //ROS_INFO("roll: [%lf]  pitch: [%lf]  yaw: [%lf]\n", roll, pitch, yaw);//debug
    }/*************************��������ȥ���Ϥ����ޤ�*************************/
    /*Euler����Quaternion���Ѵ�����*/
    //tf::Quaternion guide_quat_msg=tf::createQuaternionFromRPY(roll, pitch, yaw);//Euler->Quaternion���Ѵ�
    tf::Matrix3x3 obt_mat;
    obt_mat.setEulerYPR(yaw,-pitch,roll);
    tf::Quaternion q_tf;
    obt_mat.getRotation(q_tf);
    /*����������å������˳�Ǽ*/
    guide_msg.orientation.x = q_tf.getX();//���Τ�xy����
    guide_msg.orientation.y = q_tf.getY();
    guide_msg.orientation.z = q_tf.getZ();
    guide_msg.orientation.w = q_tf.getW();
    guide_msg.body_rate.x = 0.0;
    guide_msg.body_rate.y = 0.0;
    guide_msg.body_rate.z = 0.0;
    guide_msg.thrust = 0.5;
    /*�ºݤ�publish���Ƥ���Ȥ���*/
    guide_pub.publish(guide_msg); 
    //ROS_INFO("%d",cnt);
    //printf("%d\n",cnt);
    //cnt = cnt+1;
    /*ROS��������Ϥ�*/
    /*spin()�ϥΡ��ɤΥ���åȥ�����ν�������λ�����Ȥ��Τ���äƤ���*/
    /*while�롼�פ��򤱤��������硼�ȥ��åȤǤ���*/
    /*����ˤ�귫���֤���������*/
    //ros::spinOnce();
    marker_del_t = 10.0;//marker�����Ф��ʤ��Ȥ���10s�ˤʤ�
    spinner.start();
    loop_rate.sleep();
    }
  ros::waitForShutdown();
}

  
