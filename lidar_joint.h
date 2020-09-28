 
#include <DynamixelWorkbench.h>

#define BAUDRATE                        1000000 // baurd rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR
#define JOINT_CNT                        1
#define JOINT_ID_1                       1
#define DEBUG_SERIAL                    SerialBT2

typedef struct
{
  uint8_t id[20];
  uint8_t cnt;
} Dynamixel;


//Dynamixel joint_;

class LiDAR_joint_Driver
{
  public:
    LiDAR_joint_Driver();
    ~LiDAR_joint_Driver();

    bool init(uint8_t *joint_id, uint8_t joint_cnt);
    void closeDynamixel(void);
    bool setTorque(bool onoff);
    bool getTorqueState(void);
    bool syncReadDynamixelInfo(void);
    bool getPosition(double *get_data);
    bool getVelocity(double *get_data);
    bool getCurrent(double *get_data);
    bool writeJointPosition(double *set_data);
    bool writeJointProfileControlParam(double set_time, double acc = 0.0f);

  private:
    DynamixelWorkbench dxl_wb_;

    Dynamixel joint_;
  
    bool torque_state_;
};
