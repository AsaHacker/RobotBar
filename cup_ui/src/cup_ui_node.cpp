#include "ros/ros.h"
#include "kduino/KduinoImu.h"
#include "kduino/imu_visualization.h"
#include "cup_ui/CupUi.h"


class CupUi
{
public:
  CupUi(ros::NodeHandle nh, ros::NodeHandle nh_private)
 :nh_(nh), nh_private_(nh_private)
  {
    cup_ui_pub_ = nh_.advertise<cup_ui::CupUi>("cup_ui",2);
    imu_sub_= nh_.subscribe("kduino/imu", 1, &CupUi::kduinoImuCallback, this, ros::TransportHints().tcpNoDelay());

      if (!nh.getParam ("tilt_angle_thre", tilt_angle_thre_))
        tilt_angle_thre_ = 0.5; //~ 30 degree

  }
    ~CupUi(){};

  const static float G = 9.797;
  const static int CALIB_COUNT = 30;
          

  private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber imu_sub_;
  ros::Publisher cup_ui_pub_;

  float accXb,accYb, accZb, accXc,accYc, accZc;
  float accXBias, accYBias, accZBias;
  float accXcNonBias,accYcNonBias, accZcNonBias;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;
  float height;

  float pitch;  //pitch angle
  float roll;    //roll angle
  float yaw;    //yaw angle

  double tilt_angle_thre_;

  ros::Time imuTimeStamp;

  void kduinoImuCallback(const kduino::KduinoImuConstPtr& imu_msg)
  {
    imuTimeStamp = imu_msg->header.stamp;
    roll  = M_PI * imu_msg->angle[0]  / 180.0;
    pitch = M_PI * imu_msg->angle[1] / 180.0;
    yaw   = M_PI * imu_msg->angle[2] / 180.0;

    accXb = imu_msg->accData[0] * ACC_SCALE;
    accYb = imu_msg->accData[1] * ACC_SCALE;
    accZb = imu_msg->accData[2] * ACC_SCALE;
    gyroX = imu_msg->gyroData[0] * GYRO_SCALE;
    gyroY = imu_msg->gyroData[1] * GYRO_SCALE;
    gyroZ = imu_msg->gyroData[2] * GYRO_SCALE;
    magX = imu_msg->magData[0] * MAG_SCALE;
    magY = imu_msg->magData[1] * MAG_SCALE;
    magZ = imu_msg->magData[2] * MAG_SCALE;
    height = imu_msg->altitude / 100.0;  //cm


    imuDataConverter();

    cup_control();
  }



  void imuDataConverter()
  {
    static int biasCalib = 0;


    accXc = (accXb) * cos(pitch) + 
      (accYb) * sin(pitch) * sin(roll) + 
      (accZb) * sin(pitch) * cos(roll);
    accYc = (accYb) * cos(roll) - (accZb) * sin(roll);
    accZc = (accXb) * (-sin(pitch)) + 
      (accYb) * cos(pitch) * sin(roll) + 
      (accZb) * cos(pitch) * cos(roll) + (-G);


    //bais calibration
    if(biasCalib < CALIB_COUNT)
      {
        biasCalib ++;

        //acc bias
        accXBias += accXc;
        accYBias += accYc;
        accZBias += accZc;

        if(biasCalib == CALIB_COUNT)
          {
            accXBias /= CALIB_COUNT;
            accYBias /= CALIB_COUNT;
            accZBias /= CALIB_COUNT;
            ROS_WARN("accX bias is %f, accY bias is %f, accZ bias is %f", accXBias, accYBias, accZBias);
          }
      }
    if(biasCalib == CALIB_COUNT)
      {
        accXcNonBias = accXc - accXBias;
        accYcNonBias = accYc - accYBias;
        accZcNonBias = accZc - accZBias;
      }

  }

  void cup_control()
  {
    cup_ui::CupUi cup_ui_msg;
    if(pitch > tilt_angle_thre_ || pitch < -tilt_angle_thre_ ) 
      cup_ui_msg.horizontal_flag = 1; 
    cup_ui_msg.horizontal_rate = pitch / (M_PI / 2);

    if(roll > tilt_angle_thre_ || roll < -tilt_angle_thre_ ) //forward
      cup_ui_msg.vertical_flag = 1; 
    cup_ui_msg.vertical_rate = roll / (M_PI / 2);

    cup_ui_pub_.publish(cup_ui_msg);

  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cup_ui");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  CupUi*  cupUiNode = new CupUi(nh, nh_private);
  ros::spin ();
  delete cupUiNode;


}
