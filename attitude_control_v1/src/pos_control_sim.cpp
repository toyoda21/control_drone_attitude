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
ros::Publisher pub;//使ってない
/*current_position*/
float marker_pos_x;
float marker_pos_y;
float marker_t;//マーカで位置を得る計算周期
float imu_pos_x;
float imu_pos_y;
float imu_t;//imuで位置を得る計算周期
double theta_c;
double phi_c;
double yaw_c;
/*positionのbuffer*/
float diff_x[2];
float diff_y[2];
/*積分値*/
double integral_x, integral_y;
/*自己位置検出時間*/
/*連続して1s以内で検出すればmarkerのposを使う*/
ros::Time marker_1stTime;//周期的でないことに注意
ros::Time marker_2ndTime;
ros::Time marker_timer1;
ros::Time marker_timer2;
double wait_time;
double marker_del_t;
double del_t;//DELTA T
/*attitudeの指令値*/
mavros_msgs::AttitudeTarget guide_msg;//mavrosにpublishする姿勢
/*markerを検出しているかのフラグ*/
int marker_succeeded;

/*int id;
float pos_x, pos_y, pos_z;
float quat_x, quat_y, quat_z, quat_w;
float eul_x, eul_y, eul_z;
int cnt = 0;
float vel_x, vel_y;
float pi = 3.1415;
float theta_x, theta_y;*/
/*globalに定義する変数*/
/*target*/
float target_pos_x, target_pos_y, target_pos_z;
/*current*/
float curr_pos_x, curr_pos_y, curr_pos_z;
//float curr_quat_x, curr_quat_y, curr_quat_z, curr_quat_w;
//float curr_eul_x, curr_eul_y, curr_eul_z;

/*******************************関数定義*********************************
 *Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数(Cb)
 *引数にはトピックにPublishされるメッセージの型と同じ型を定義すること
 ***********************************************************************/
/*======================================================================
  markerを検出したとき現在位置からmarkerごとの目標位置へ移動するpositionを
  出力
 =======================================================================*/
void marker_Callback(const aruco_msgs::MarkerArray::Ptr& msg){
  int id;
  /*target*/
  //float target_pos_x, target_pos_y, target_pos_z;
  /*current*/
  //float curr_pos_x, curr_pos_y, curr_pos_z;
  float curr_quat_x, curr_quat_y, curr_quat_z, curr_quat_w;
  float curr_eul_x, curr_eul_y, curr_eul_z;
  /*自己位置取得周期をグローバル変数に格納*/
  marker_2ndTime = marker_1stTime;
  marker_1stTime = ros::Time::now();
  /*計測開始時間*/
  marker_timer1 = ros::Time::now();
  /*初期化*/
  ros::NodeHandle n;
  //MarkerIDの値をidに格納する
  id = msg->markers[0].id;
  //MarkerPositionの値をpos_x,pos_y,pos_zに格納する
  curr_pos_x = msg->markers[0].pose.pose.position.x;
  curr_pos_y = msg->markers[0].pose.pose.position.y;
  curr_pos_z = msg->markers[0].pose.pose.position.z;
  //MarkerPoseQuaternionの値をquaternionのx,y,zに格納する
  curr_quat_x = msg->markers[0].pose.pose.orientation.x;
  curr_quat_y = msg->markers[0].pose.pose.orientation.y;
  curr_quat_z = msg->markers[0].pose.pose.orientation.z;
  curr_quat_w = msg->markers[0].pose.pose.orientation.w;
  //MarkerPoseEulerの値をeulerのx,y,zに格納する
  //curr_eul_x = msg->epose.x;
  //curr_eul_y = msg->epose.y;
  //curr_eul_z = msg->epose.z;
  std::cout << "-----------------------------------" << std::endl;
  //MarkerIDの表示
  std::cout << "Marker        ID: " << id << std::endl;
  //MarkerPositionの表示
  std::cout << "MarkerPosition x: " << curr_pos_x << std::endl;
  std::cout << "               y: " << curr_pos_y << std::endl;
  std::cout << "               z: " << curr_pos_z << std::endl;
  //MarkerPoseQuaternionの表示
  //std::cout << "MarkerPoseQuaternion x: " << qposex << std::endl;
  //std::cout << "                     y: " << qposey << std::endl;
  //std::cout << "                     z: " << qposez << std::endl;
  //std::cout << "                     w: " << qposew << std::endl;
  //MarkerPoseEulerの表示
  //std::cout << "MarkerPoseEuler      x: " << eposex << std::endl;
  //std::cout << "                     y: " << eposey << std::endl;
  std::cout << "             yaw: " << curr_eul_z << std::endl << std::endl;
  /*idによって処理を変更する*/
  if(id == 1000){
    /*target positionの指定*/
    target_pos_x = 0.0;
    target_pos_y = 0.0;
  }
  /*~~~~~~~~~~marker検出周期の取得~~~~~~~~~~~~~~~~~~~~~~~~*/
  ros::Duration dura1(marker_1stTime - marker_2ndTime);//ros::Time同士の差はros::Duration型
  marker_del_t = dura1.toSec();
  // ROS_INFO("marker_del_t: [%lf]\n\t", marker_del_t);
  /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~/
  /*positionの微積*/
  /*diff_x,y[1] integral_x,y diff_x,y[1]-diff_x,y[0]/del_t */
  diff_x[0] = diff_x[1];
  diff_x[1] = target_pos_x - curr_pos_x;
  integral_x += ((diff_x[1] + diff_x[0]) / 2.0)* del_t;
  diff_y[0] = diff_y[1];
  diff_y[1] = target_pos_y - curr_pos_y;
  integral_y += ((diff_y[1] + diff_y[0]) / 2.0)* del_t;
  /*mavrosへpublishするtopicの内容*/
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
 * mavrosからangle&angvel&accelを受け取り現在位置が不明な場合は停止する姿勢
 * 指令値を出力 
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
  //angular_velosityの値をangvelx,angvely,angvelzに格納する
  angvel_x = msg->angular_velocity.x;
  angvel_y = msg->angular_velocity.y;
  angvel_z = msg->angular_velocity.z;
  //linear_accelerationの値をaccelx,accely,accelzに格納する
  accel_x = msg->linear_acceleration.x;
  accel_y = msg->linear_acceleration.y;
  accel_z = msg->linear_acceleration.z;
  //処理
  msgxx.orientation.x = 0.0;
  msgxx.orientation.y = 0.0;
  msgxx.orientation.z = 0.0;
  msgxx.orientation.w = 0.0;
  msgxx.body_rate.x = 0.0;
  msgxx.body_rate.y = 0.0;
  msgxx.body_rate.z = 0.0;
  msgxx.thrust = 1.0;
  /*クォータニオンをオイラー角に変換*/
  tf::Quaternion imu_quat(quat_x, quat_y, quat_z, quat_w);
  tf::Matrix3x3 m(imu_quat);
  m.getRPY(imu_roll, imu_pitch, imu_yaw);
  /*現在姿勢角をグローバル変数に格納*/
  theta_c = imu_roll;
  phi_c = -imu_pitch;//符号合わせ
  yaw_c = -imu_yaw;//符号合わせ
  /*debag用ここから*/
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
  /*debag用ここまで*/
  //orientationの表示
  //ROS_INFO("Imu Orientation\n\t x: [%f]\n\t y: [%f]\n\t z: [%f]\n\t w: [%f]", quat_x, quat_y, quat_z, quat_w);
  //angular_velocityの表示
  //ROS_INFO("Imu Angular_velocity\n\t x: [%f]\n\t y: [%f]\n\t z: [%f]", angvel_x, angvel_y, angvel_z);
  //linear_accelerationの表示
  //ROS_INFO("Imu linear_acceleration\n\t x: [%f]\n\t y: [%f]\n\t z: [%f]\n\t",accel_x, accel_y, accel_z);
  //debug用printf
  //ROS_INFO("%f : %f : %f\n", imu_roll, imu_pitch, imu_yaw);
  /*marker検出フラグの判定*/
  /*msgxxの値をグローバルなguide_msgsに格納する*/
  /*guide_msg.orientation.x = 0.0;
  guide_msg.orientation.y = 0.0;
  guide_msg.orientation.z = 0.0;
  guide_msg.orientation.w = 0.0;
  guide_msg.body_rate.x = 0.0;
  guide_msg.body_rate.y = 0.0;
  guide_msg.body_rate.z = 0.0;
  guide_msg.thrust = 1.0;*/
}/***************************関数定義はここまで****************************/

int main(int argc,char **argv){
  int cnt=0;
  /*del_tの初期化(はじめはmarkerなし)*/
  del_t = 1.0;
  /*ROSの初期化*/
  ros::init(argc, argv, "attitude_control_v1");
  /*ノードの初期化とリソースのクリア*/
  ros::NodeHandle n;
  guide_pub = n.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",2000);
  /*実行周期の定義*/
  ros::Rate loop_rate(freq);
  /*複数のスレッドからコールバックを呼び出すspinnerの定義*/
  ros::AsyncSpinner spinner(2);
  /*姿勢指令を送るデータの定義*/
  //mavros_msgs::AttitudeTarget guide_msg;
  /*/mavros/setpoint_raw/attitudeの初期化*/
  guide_msg.orientation.x = 0.0;
  guide_msg.orientation.y = 0.0;
  guide_msg.orientation.z = 0.0;
  guide_msg.orientation.w = 1.0;
  guide_msg.body_rate.x = 0.0;
  guide_msg.body_rate.y = 0.0;
  guide_msg.body_rate.z = 0.0;
  guide_msg.thrust = 0.5;
  /*変数宣言*/
  /*目標姿勢導出ゲイン*/
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
  /*実ループ*/
  while(ros::ok()){
    /*delta_tの取得*/
    del_t = marker_del_t;
    /*位置が情報できるまでの待ち時間*/
    marker_timer2 = ros::Time::now();
    ros::Duration dura2(marker_timer2 - marker_timer1);
    wait_time = dura2.toSec();
    /*marker検出時間が制御に耐えられるものなら使う*/
    /*判定条件は次のmarkerがXsecの間に検出できる時間*/
    if(ros::Duration(wait_time) >= ros::Duration(0.2)){
      /*慣性航法*/
      roll = 0.0;
      pitch = 0.0;
      yaw = 0.0;
      ROS_INFO("no_marker");
    }else{
      /****************************目標姿勢角生成********************************/
      /*roll*/
      theta_d = p * diff_x[1] + i * integral_x + d * ( diff_x[1] - diff_x[0]) / del_t;//位置によって傾く向きを合わせる
      if(theta_d > 0.1745)
	theta_d = 0.1745;//+10deg以上は傾けない
      if(theta_d < -0.1745)
	theta_d = 0.1745;//-10deg以上は傾けない
      /*pitch*/
      phi_d   = p * diff_y[1] + i * integral_y + d * ( diff_y[1] - diff_y[0]) / del_t;
      if(phi_d > 0.1745)
	phi_d = 0.1745;//10deg以上は傾けない
      if(phi_d < -0.1745)
	phi_d = -0.1745;//10deg以上は傾けない
      /*yaw head directionの目標方位の生成*/
      yaw_d   = 0.0;
      /**************************目標姿勢角生成はここまで************************/   
      /*****************************姿勢コントローラ****************************/
      /*roll*/
      r_ang_integral = 0.0;
      r_ang_diff[0] = r_ang_diff[1];
      r_ang_diff[1] = theta_d - theta_c;
      r_ang_integral = r_ang_integral + ((r_ang_diff[1] + r_ang_diff[0]) / 2.0 ) * 1/freq;
	  
      roll =kp * r_ang_diff[1] + ki * r_ang_integral + kd * (r_ang_diff[1] - r_ang_diff[0]) / freq;//符号合わせ
      /*pitch*/
      p_ang_integral = 0.0;
      p_ang_diff[0] = p_ang_diff[1];
      p_ang_diff[1] = phi_d - phi_c;
      p_ang_integral += (p_ang_diff[1] + p_ang_diff[0]) / 2.0 * freq;
      
      pitch =-( kp * p_ang_diff[1] + ki * p_ang_integral + kd * (p_ang_diff[1] - p_ang_diff[0]) / freq);//符号合わせ
      /*yaw*/
      y_ang_integral = 0.0;
      y_ang_diff[0] = y_ang_diff[1];
      y_ang_diff[1] = yaw_d - yaw_c;
      y_ang_integral += (y_ang_diff[1] + y_ang_diff[0]) / 2.0 * freq;
      
      yaw =-( kp * y_ang_diff[1] + ki * y_ang_integral + kd * (y_ang_diff[1] - y_ang_diff[0]) / freq);
      //ROS_INFO("roll_d: [%lf]  pitch_d: [%lf]  yaw_d: [%lf]\n", theta_d, phi_d, yaw_d);//debug
      //ROS_INFO("roll: [%lf]  pitch: [%lf]  yaw: [%lf]\n", roll, pitch, yaw);//debug
    }/*************************姿勢コントローラはここまで*************************/
    /*EulerからQuaternionに変換する*/
    //tf::Quaternion guide_quat_msg=tf::createQuaternionFromRPY(roll, pitch, yaw);//Euler->Quaternionに変換
    tf::Matrix3x3 obt_mat;
    obt_mat.setEulerYPR(yaw,-pitch,roll);
    tf::Quaternion q_tf;
    obt_mat.getRotation(q_tf);
    /*姿勢指令をメッセージに格納*/
    guide_msg.orientation.x = q_tf.getX();//何故かxyが逆
    guide_msg.orientation.y = q_tf.getY();
    guide_msg.orientation.z = q_tf.getZ();
    guide_msg.orientation.w = q_tf.getW();
    guide_msg.body_rate.x = 0.0;
    guide_msg.body_rate.y = 0.0;
    guide_msg.body_rate.z = 0.0;
    guide_msg.thrust = 0.5;
    /*実際にpublishしているところ*/
    guide_pub.publish(guide_msg); 
    //ROS_INFO("%d",cnt);
    //printf("%d\n",cnt);
    //cnt = cnt+1;
    /*ROSに制御を渡す*/
    /*spin()はノードのシャットダウンの準備が完了したときのみ戻ってくる*/
    /*whileループを避ける便利ショートカットである*/
    /*これにより繰り返しが起きる*/
    //ros::spinOnce();
    marker_del_t = 10.0;//markerが検出がないときは10sになる
    spinner.start();
    loop_rate.sleep();
    }
  ros::waitForShutdown();
}

  
