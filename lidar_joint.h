 

#include <DynamixelWorkbench.h>


/*******************************************************************************
* encoder
*******************************************************************************/

#define WHEEL_NUM                        2

#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14      // 180 degree

/*******************************************************************************
* servo
*******************************************************************************/
#define DEVICE_NAME ""
#define BAUDRATE  1000000


#define LEFT            0
#define RIGHT           1
#define FORWARDS true
#define BACKWARDS false

#define PWM_MIN 110
#define PWMRANGE 255

/*******************************************************************************
* Joint servo
*******************************************************************************/

void jointTrajectoryPointCallback(const std_msgs::Int32& joint_trajectory_point_msg);
ros::Subscriber<std_msgs::Int32> joint_position_sub("joint_trajectory_point", jointTrajectoryPointCallback);

DynamixelWorkbench dxl_wb;

bool is_moving        = false;
int joint_trajectory_point;
uint8_t DXL_ID = 1;

/*******************************************************************************
* encoder
*******************************************************************************/

// add in the next 3 lines to fix min max bug
#undef min
inline int min(int a, int b) { return ((a)<(b) ? (a) : (b)); }
inline double min(double a, double b) { return ((a)<(b) ? (a) : (b)); }

#undef max
inline int max(int a, int b) { return ((a)>(b) ? (a) : (b)); }
inline double max(double a, double b) { return ((a)>(b) ? (a) : (b)); }

uint16_t lPwm;
uint16_t rPwm;
float l;
float r;

const uint8_t L_PWM = 9;
const uint8_t L_BACK = 4;
const uint8_t L_FORW = 5;
const uint8_t R_BACK = 6;
const uint8_t R_FORW = 7;
const uint8_t R_PWM = 10;

volatile long encoderLeft = 0L;
volatile long encoderRight = 0L;
const byte LencoderPinA = 2;
const byte LencoderPinB = 14;
const byte RencoderPinA = 3;
const byte RencoderPinB = 15;
volatile bool Lfired;
volatile bool Lup;
volatile bool Rfired;
volatile bool Rup;



/***************API memo***************
bool init(const char* device_name = "/dev/ttyUSB0",
        uint32_t baud_rate = 57600,
        const char **log = NULL);

bool begin(const char* device_name = "/dev/ttyUSB0",
        uint32_t baud_rate = 57600,
        const char **log = NULL);

bool setPortHandler(const char *device_name, const char **log = NULL);
bool setBaudrate(uint32_t baud_rate, const char **log = NULL);
bool setPacketHandler(float protocol_version, const char **log = NULL);

float getProtocolVersion(void);
uint32_t getBaudrate(void);

const char * getModelName(uint8_t id, const char **log = NULL);
uint16_t getModelNumber(uint8_t id, const char **log = NULL);
const ControlItem *getControlTable(uint8_t id, const char **log = NULL);
const ControlItem *getItemInfo(uint8_t id, const char *item_name, const char **log = NULL);
uint8_t getTheNumberOfControlItem(uint8_t id, const char **log = NULL);
const ModelInfo* getModelInfo(uint8_t id, const char **log = NULL);

uint8_t getTheNumberOfSyncWriteHandler(void);
uint8_t getTheNumberOfSyncReadHandler(void);
uint8_t getTheNumberOfBulkReadParam(void);

bool scan(uint8_t *get_id,
        uint8_t *get_the_number_of_id,
        uint8_t range = 253,
        const char **log = NULL);

bool scan(uint8_t *get_id,
        uint8_t *get_the_number_of_id,
        uint8_t start_number,
        uint8_t end_number,
        const char **log = NULL);

bool ping(uint8_t id,
        uint16_t *get_model_number,
        const char **log = NULL);

bool ping(uint8_t id,
        const char **log = NULL);

bool clearMultiTurn(uint8_t id, const char **log = NULL);

bool reboot(uint8_t id, const char **log = NULL);
bool reset(uint8_t id, const char **log = NULL);

bool writeRegister(uint8_t id, uint16_t address, uint16_t length, uint8_t* data, const char **log = NULL);
bool writeRegister(uint8_t id, const char *item_name, int32_t data, const char **log = NULL);

bool writeOnlyRegister(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, const char **log = NULL);
bool writeOnlyRegister(uint8_t id, const char *item_name, int32_t data, const char **log = NULL);

bool readRegister(uint8_t id, uint16_t address, uint16_t length, uint32_t *data, const char **log = NULL);
bool readRegister(uint8_t id, const char *item_name, int32_t *data, const char **log = NULL);

void getParam(int32_t data, uint8_t *param);

bool addSyncWriteHandler(uint16_t address, uint16_t length, const char **log = NULL);
bool addSyncWriteHandler(uint8_t id, const char *item_name, const char **log = NULL);

bool syncWrite(uint8_t index, int32_t *data, const char **log = NULL);
bool syncWrite(uint8_t index, uint8_t *id, uint8_t id_num, int32_t *data, uint8_t data_num_for_each_id, const char **log = NULL);

bool addSyncReadHandler(uint16_t address, uint16_t length, const char **log = NULL);
bool addSyncReadHandler(uint8_t id, const char *item_name, const char **log = NULL);

bool syncRead(uint8_t index, const char **log = NULL);
bool syncRead(uint8_t index, uint8_t *id, uint8_t id_num, const char **log = NULL);

bool getSyncReadData(uint8_t index, int32_t *data, const char **log = NULL);
bool getSyncReadData(uint8_t index, uint8_t *id, uint8_t id_num, int32_t *data, const char **log = NULL);
bool getSyncReadData(uint8_t index, uint8_t *id, uint8_t id_num, uint16_t address, uint16_t length, int32_t *data, const char **log = NULL);

bool initBulkWrite(const char **log = NULL);

bool addBulkWriteParam(uint8_t id, uint16_t address, uint16_t length, int32_t data, const char **log = NULL);
bool addBulkWriteParam(uint8_t id, const char *item_name, int32_t data, const char **log = NULL);

bool bulkWrite(const char **log = NULL);

bool initBulkRead(const char **log = NULL);

bool addBulkReadParam(uint8_t id, uint16_t address, uint16_t length, const char **log = NULL);
bool addBulkReadParam(uint8_t id, const char *item_name, const char **log = NULL);

bool bulkRead(const char **log = NULL);

bool getBulkReadData(int32_t *data, const char **log = NULL);
bool getBulkReadData(uint8_t *id, uint8_t id_num, uint16_t *address, uint16_t *length, int32_t *data, const char **log = NULL);

bool clearBulkReadParam(void);
bool torque(uint8_t id, bool onoff, const char **log = NULL);
bool torqueOn(uint8_t id, const char **log = NULL);
bool torqueOff(uint8_t id, const char **log = NULL);

bool changeID(uint8_t id, uint8_t new_id, const char **log = NULL);
bool changeBaudrate(uint8_t id, uint32_t new_baudrate, const char **log = NULL);
bool changeProtocolVersion(uint8_t id, uint8_t version, const char **log = NULL);

bool itemWrite(uint8_t id, const char *item_name, int32_t data, const char **log = NULL);
bool itemRead(uint8_t id, const char *item_name, int32_t *data, const char **log = NULL);

bool led(uint8_t id, bool onoff, const char **log = NULL);
bool ledOn(uint8_t id, const char **log = NULL);
bool ledOff(uint8_t id, const char **log = NULL);

bool setNormalDirection(uint8_t id, const char **log = NULL);
bool setReverseDirection(uint8_t id, const char **log = NULL);

bool setVelocityBasedProfile(uint8_t id, const char **log = NULL);
bool setTimeBasedProfile(uint8_t id, const char **log = NULL);

bool setSecondaryID(uint8_t id, uint8_t secondary_id, const char **log = NULL);

bool setCurrentControlMode(uint8_t id, const char **log = NULL);
bool setTorqueControlMode(uint8_t id, const char **log = NULL);
bool setVelocityControlMode(uint8_t id, const char **log = NULL);  
bool setPositionControlMode(uint8_t id, const char **log = NULL);  
bool setExtendedPositionControlMode(uint8_t id, const char **log = NULL);
bool setMultiTurnControlMode(uint8_t id, const char **log = NULL);
bool setCurrentBasedPositionControlMode(uint8_t id, const char **log = NULL);
bool setPWMControlMode(uint8_t id, const char **log = NULL);

bool setOperatingMode(uint8_t id, uint8_t index, const char **log = NULL);

bool jointMode(uint8_t id, int32_t velocity = 0, int32_t acceleration = 0, const char **log = NULL);
bool wheelMode(uint8_t id, int32_t acceleration = 0, const char **log = NULL);
bool currentBasedPositionMode(uint8_t id, int32_t current = 0, const char **log = NULL);

bool goalPosition(uint8_t id, int32_t value, const char **log = NULL);
bool goalPosition(uint8_t id, float radian, const char **log = NULL);

bool goalVelocity(uint8_t id, int32_t value, const char **log = NULL);
bool goalVelocity(uint8_t id, float velocity, const char **log = NULL);

bool getPresentPositionData(uint8_t id, int32_t* data, const char **log = NULL);
bool getRadian(uint8_t id, float* radian, const char **log = NULL);

bool getPresentVelocityData(uint8_t id, int32_t* data, const char **log = NULL);
bool getVelocity(uint8_t id, float* velocity, const char **log = NULL);

int32_t convertRadian2Value(uint8_t id, float radian);
float convertValue2Radian(uint8_t id, int32_t value);

int32_t convertRadian2Value(float radian, int32_t max_position, int32_t min_position, float max_radian, float min_radian);
float convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position, float max_radian, float min_radian);

int32_t convertVelocity2Value(uint8_t id, float velocity);
float convertValue2Velocity(uint8_t id, int32_t value);

int16_t convertCurrent2Value(float current);
float convertValue2Current(int16_t value);

float convertValue2Load(int16_t value);
*****************************************************/
