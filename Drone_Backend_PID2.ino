/*************************************************************************************************************

   In this example, we'll process IMU data from MPU9250 sensor.

   See https://github.com/pronenewbits for more!
 ************************************************************************************************************/
#include <Wire.h>
#include <elapsedMillis.h>
#include "konfig.h"
#include "matrix.h"
#include "ekf.h"
#include "simple_mpu9250.h"


/* ================================================== The AHRS/IMU variables ================================================== */
/* Gravity vector constant (align with global Z-axis) */
#define IMU_ACC_Z0          (1)
/* Magnetic vector constant (align with local magnetic vector) */
float_prec IMU_MAG_B0_data[3] = {cos(0), sin(0), 0.000000};
Matrix IMU_MAG_B0(3, 1, IMU_MAG_B0_data);
/*
   cos(0)
   sin(0)
   0.000000
*/
/* The hard-magnet bias */
float_prec HARD_IRON_BIAS_data[3] = {8.832973, 7.243323, 23.95714};
Matrix HARD_IRON_BIAS(3, 1, HARD_IRON_BIAS_data);
/*
   8.832973
   7.243323
   23.95714
*/

/* ============================================ EKF variables/function declaration ============================================ */
/* Just example; in konfig.h에 정의 되어 있다.
  SS_X_LEN = 4
  SS_Z_LEN = 6
  SS_U_LEN = 3

  /* EKF initialization constant -------------------------------------------------------------------------------------- */
#define P_INIT      (10.)
//칼만피터의 최종값의 공분산 Kalman 이득을 계산하는데 사용됨
#define Q_INIT      (1e-6)
//예측 프로세스의 공분산
#define R_INIT_ACC  (0.0015/10.)
//측정의 공분산
#define R_INIT_MAG  (0.0015/10.)

/* P(k=0) variable -------------------------------------------------------------------------------------------------- */
float_prec EKF_PINIT_data[SS_X_LEN * SS_X_LEN] = {P_INIT, 0,      0,      0,
                                                  0,      P_INIT, 0,      0,
                                                  0,      0,      P_INIT, 0,
                                                  0,      0,      0,      P_INIT
                                                 };
Matrix EKF_PINIT(SS_X_LEN, SS_X_LEN, EKF_PINIT_data);
/*
   4x4 행렬로 만들어 준다.
*/
/* Q constant ------------------------------------------------------------------------------------------------------- */
float_prec EKF_QINIT_data[SS_X_LEN * SS_X_LEN] = {Q_INIT, 0,      0,      0,
                                                  0,      Q_INIT, 0,      0,
                                                  0,      0,      Q_INIT, 0,
                                                  0,      0,      0,      Q_INIT
                                                 };
Matrix EKF_QINIT(SS_X_LEN, SS_X_LEN, EKF_QINIT_data);
/* R constant ------------------------------------------------------------------------------------------------------- */
float_prec EKF_RINIT_data[SS_Z_LEN * SS_Z_LEN] = {R_INIT_ACC, 0,          0,          0,          0,          0,
                                                  0,          R_INIT_ACC, 0,          0,          0,          0,
                                                  0,          0,          R_INIT_ACC, 0,          0,          0,
                                                  0,          0,          0,          R_INIT_MAG, 0,          0,
                                                  0,          0,          0,          0,          R_INIT_MAG, 0,
                                                  0,          0,          0,          0,          0,          R_INIT_MAG
                                                 };
Matrix EKF_RINIT(SS_Z_LEN, SS_Z_LEN, EKF_RINIT_data);
/* Nonlinear & linearization function ------------------------------------------------------------------------------- */
bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianF(Matrix& F, const Matrix& X, const Matrix& U);
bool Main_bCalcJacobianH(Matrix& H, const Matrix& X, const Matrix& U);

/* EKF variables ---------------------------------------------------------------------------------------------------- */
Matrix quaternionData(SS_X_LEN, 1);
Matrix gainData(SS_X_LEN, SS_Z_LEN);
Matrix Y(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
/*************************************************************************************************************
    Class for Discrete Extended Kalman Filter
    The system to be estimated is defined as a discrete nonlinear dynamic dystem:
                x(k) = f[x(k-1), u(k-1)] + v(k)     ; x = Nx1,    u = Mx1
                y(k) = h[x(k)] + n(k)               ; y = Zx1
          Where:
            x(k) : State Variable at time-k                          : Nx1
            y(k) : Measured output at time-k                         : Zx1
            u(k) : System input at time-k                            : Mx1
            v(k) : Process noise, AWGN assumed, w/ covariance Qn     : Nx1
            n(k) : Measurement noise, AWGN assumed, w/ covariance Rn : Nx1
            f(..), h(..) is a nonlinear transformation of the system to be estimated.

 ***************************************************************************************************
  /* EKF system declaration ------------------------------------------------------------------------------------------- */
EKF EKF_IMU(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT,
            Main_bUpdateNonlinearX, Main_bUpdateNonlinearY, Main_bCalcJacobianF, Main_bCalcJacobianH);
//야코비안 F는 X추정용, H는 측정 모델

/* ========================================= Auxiliary variables/function declaration ========================================= */
elapsedMillis timerLed, timerEKF;
uint64_t u64compuTime;

char bufferTxSer[100];
/* A SimpleMPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68 */
SimpleMPU9250 IMU(Wire, 0x68);

/*========================================================Dual-PID===============================================================*/

//시간관련 값//
float dt;
unsigned long t_now;
unsigned long t_prev;


float filtered_angle_x = 0;
float filtered_angle_y = 0;
float filtered_angle_z = 0;

//이중루프PID에서 제어할 변수들(Roll, Pitch, Yaw)//
float roll_target_angle = 0.0;
float roll_angle_in;
float roll_rate_in;
float roll_stabilize_kp = 1;
float roll_stabilize_ki = 0;
float roll_rate_kp = 1;
float roll_rate_ki = 0.1;
float roll_rate_kd = 0.5;
float roll_stabilize_iterm;
float roll_rate_pterm=0.0f;
float roll_rate_iterm=0.0f;
float roll_rate_dterm=0.0f;



float roll_output;
float roll_prev_error = 0.0;


float pitch_target_angle = 0.0;
float pitch_angle_in;
float pitch_rate_in;
float pitch_stabilize_kp = 1;
float pitch_stabilize_ki = 0;
float pitch_rate_kp = 1;
float pitch_rate_ki = 0;
float pitch_rate_kd = 0.5;
float pitch_stabilize_iterm;
float pitch_rate_pterm=0.0f;
float pitch_rate_iterm=0.0f;
float pitch_rate_dterm=0.0f;
float pitch_prev_error = 0.0;
float pitch_output;

float yaw_target_angle = 0.0;
float yaw_angle_in;
float yaw_rate_in;
float yaw_stabilize_kp = 1;
float yaw_stabilize_ki = 0;
float yaw_rate_kp = 1;
float yaw_rate_ki = 0;
float yaw_rate_kd = 0.5;
float yaw_stabilize_iterm;
float yaw_rate_pterm=0.0f;
float yaw_rate_iterm=0.0f;
float yaw_rate_dterm=0.0f;
float yaw_prev_error = 0.0;
float yaw_output;

float base_roll_target_angle;
float base_pitch_target_angle;
float base_yaw_target_angle;

extern float roll_target_angle;
extern float pitch_target_angle;
extern float yaw_target_angle;
/*========================================================Dual-PID===============================================================*/

void setup() {
  /* Serial initialization -------------------------------------- */
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Calibrating IMU bias...");

  /* IMU initialization ----------------------------------------- */
  int status = IMU.begin();   /* start communication with IMU */
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  /* EKF initialization ----------------------------------------- */
  /* x(k=0) = [1 0 0 0]' */
  quaternionData.vSetToZero();
  quaternionData[0][0] = 1.0;
  EKF_IMU.vReset(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT);

  snprintf(bufferTxSer, sizeof(bufferTxSer) - 1, "EKF in Teensy 4.0 (%s)\r\n",
           (FPU_PRECISION == PRECISION_SINGLE) ? "Float32" : "Double64");
  Serial.print(bufferTxSer);
  Serial.println("-----------------------------------------");

}

void loop() {
  /* ================== Read the sensor data / simulate the system here ================== */
  /* Read the raw data */
  IMU.readSensor();
  float Ax = IMU.getAccelX_mss();
  float Ay = IMU.getAccelY_mss();
  float Az = IMU.getAccelZ_mss();
  float Bx = IMU.getMagX_uT();
  float By = IMU.getMagY_uT();
  float Bz = IMU.getMagZ_uT();

  float p = IMU.getGyroX_rads();
  float q = IMU.getGyroY_rads();
  float r = IMU.getGyroZ_rads();

  Serial.print("Ax:"); Serial.print(Ax); Serial.print("\t"); Serial.print("Ay:"); Serial.print(Ay); Serial.print("\t");
  Serial.print("Az:"); Serial.print(Az); Serial.print("\t");  Serial.print("Bx:"); Serial.print(Bx); Serial.print("\t");
  Serial.print("By:"); Serial.print(By); Serial.print("\t");  Serial.print("Bz:"); Serial.print(Bz); Serial.print("\t");
  Serial.print("p:"); Serial.print(p); Serial.print("\t");  Serial.print("q:"); Serial.print(q); Serial.print("\t");
  Serial.print("r:"); Serial.print(r); Serial.println();
  Serial.println("-----------------------------------------");

  /* Input 1:3 = gyroscope */
  U[0][0] = p;  U[1][0] = q;  U[2][0] = r;
  /* Output 1:3 = accelerometer */
  Y[0][0] = Ax; Y[1][0] = Ay; Y[2][0] = Az;
  /* Output 4:6 = magnetometer */
  Y[3][0] = Bx; Y[4][0] = By; Y[5][0] = Bz;

  /* Compensating Hard-Iron Bias for magnetometer */
  Y[3][0] = Y[3][0] - HARD_IRON_BIAS[0][0];
  Y[4][0] = Y[4][0] - HARD_IRON_BIAS[1][0];
  Y[5][0] = Y[5][0] - HARD_IRON_BIAS[2][0];

  /* Normalizing the output vector */
  float_prec _normG = sqrt(Y[0][0] * Y[0][0]) + (Y[1][0] * Y[1][0]) + (Y[2][0] * Y[2][0]);
  Y[0][0] = Y[0][0] / _normG;
  Y[1][0] = Y[1][0] / _normG;
  Y[2][0] = Y[2][0] / _normG;
  float_prec _normM = sqrt(Y[3][0] * Y[3][0]) + (Y[4][0] * Y[4][0]) + (Y[5][0] * Y[5][0]);
  Y[3][0] = Y[3][0] / _normM;
  Y[4][0] = Y[4][0] / _normM;
  Y[5][0] = Y[5][0] / _normM;
  /* ------------------ Read the sensor data / simulate the system here ------------------ */


  /* ============================= Update the Kalman Filter ============================== */
  u64compuTime = micros();
  bool update = EKF_IMU.bUpdate(Y, U);
  Serial.println(update);
  if (!update) {
    quaternionData.vSetToZero();
    quaternionData[0][0] = 1.0;
    EKF_IMU.vReset(quaternionData, EKF_PINIT, EKF_QINIT, EKF_RINIT);
    Serial.println("Whoop ");
  }
  u64compuTime = (micros() - u64compuTime);
  /* ----------------------------- Update the Kalman Filter ------------------------------ */


  /* The serial data is sent by responding to command from the PC running Processing scipt */
  snprintf(bufferTxSer, sizeof(bufferTxSer) - 1, "EKF in Teensy 4.0 (%s)\r\n",
           (FPU_PRECISION == PRECISION_SINGLE) ? "Float32" : "Double64");
  Serial.print(bufferTxSer);
  Serial.print('\n');

  /* =========================== Print to serial (for plotting) ========================== */
  quaternionData = EKF_IMU.GetX();
  gainData = EKF_IMU.GetGain();
  Serial.println("----------------EKF-------------------");
  Serial.print(",");
  Serial.print(quaternionData[0][0]);
  //serialFloatPrint(quaternionData[0][0]);
  Serial.print(",");
  //serialFloatPrint(quaternionData[1][0]);
  Serial.print(quaternionData[1][0]);
  Serial.print(",");
  //serialFloatPrint(quaternionData[2][0]);
  Serial.print(quaternionData[2][0]);
  Serial.print(",");
  Serial.print((float)u64compuTime);
  //serialFloatPrint((float)u64compuTime);
  Serial.println();
  Serial.print("KalmanGain roll"); Serial.print(gainData[0][0]);
  //serialFloatPrint(quaternionData[0][0]);
  Serial.print(",");
  Serial.print("KalmanGain pitch");  Serial.print(gainData[1][0]);
  //serialFloatPrint(quaternionData[0][0]);
  Serial.print(",");
  Serial.print("KalmanGain yaw"); Serial.print(gainData[2][0]);
  //serialFloatPrint(quaternionData[0][0]);
  Serial.println();
  Serial.println("----------------EKF-------------------");
  calcYPRtoDualPID();
  Serial.println();
  Serial.println("----------------PID-------------------");
  t_now = micros();
  dt = (t_now - t_prev) / 1000000.0;
  t_prev = t_now;
  const float RADIANS_TO_DEGREES = 180 / 3.14159;
  //Y[0][0] = Ax; Y[1][0] = Ay; Y[2][0] = Az;
  float accel_angle_x, accel_angle_y, accel_angle_z;
  float accel_xz, accel_yz;

  //accel_angle_y는 Roll각을 의미//
  accel_yz = sqrt(pow(Y[1][0], 2) + pow(Y[2][0], 2));
  accel_angle_y = atan(-Y[0][0] / accel_yz) * RADIANS_TO_DEGREES;

  //accel_angle_x는 Pitch값을 의미//
  accel_xz = sqrt(pow(Y[0][0], 2) + pow(Y[2][0], 2));
  accel_angle_x = atan(Y[1][0] / accel_xz) * RADIANS_TO_DEGREES;
  accel_angle_z = 0; //중력 가속도(g)의 방향과 정반대의 방향을 가리키므로 가속도 센서를 이용해서는 회전각을 계산할 수 없다.//

  const float ALPHA = 0.96;
  float tmp_angle_x, tmp_angle_y, tmp_angle_z;

  tmp_angle_x = filtered_angle_x + quaternionData[1][0] * dt;
  tmp_angle_y = filtered_angle_y + quaternionData[0][0] * dt;
  tmp_angle_z = filtered_angle_z + quaternionData[2][0] * dt;

  //상보필터 값 구하기(가속도, 자이로 센서의 절충)//
  filtered_angle_x = ALPHA * tmp_angle_x + (1.0 - ALPHA) * accel_angle_x;
  filtered_angle_y = ALPHA * tmp_angle_y + (1.0 - ALPHA) * accel_angle_y;
  filtered_angle_z = tmp_angle_z;

  roll_angle_in = filtered_angle_y;
  roll_rate_in = quaternionData[0][0];
  pitch_angle_in = filtered_angle_x;
  pitch_rate_in = quaternionData[1][0];
  yaw_angle_in = filtered_angle_z;
  yaw_rate_in = quaternionData[2][0];
  /* --------------------------- Print to serial (for plotting) -------------------------- */
  roll_target_angle = 0.0f;
  pitch_target_angle = 0.0f;
  yaw_target_angle = 0.0f;
  calcYPRtoDualPID();
  Serial.print("roll_output:"); Serial.print(roll_output); Serial.print("\t");
  Serial.print("pitch_output:"); Serial.print(pitch_output); Serial.print("\t");
  Serial.print("yaw_output:"); Serial.print(yaw_output); Serial.print("\t");
  Serial.println();
}



/* Function to interface with the Processing script in the PC */
void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for (int i = 0; i < 4; i++) {
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

    Serial.print(c1);
    Serial.print(c2);
  }
}


bool Main_bUpdateNonlinearX(Matrix & X_Next, const Matrix & X, const Matrix & U)
{
  /* Insert the nonlinear update transformation here
              x(k+1) = f[x(k), u(k)]

     The quaternion update function:
      q0_dot = 1/2. * (  0   - p*q1 - q*q2 - r*q3)
      q1_dot = 1/2. * ( p*q0 +   0  + r*q2 - q*q3)
      q2_dot = 1/2. * ( q*q0 - r*q1 +  0   + p*q3)
      q3_dot = 1/2. * ( r*q0 + q*q1 - p*q2 +  0  )

     Euler method for integration:
      q0 = q0 + q0_dot * dT;
      q1 = q1 + q1_dot * dT;
      q2 = q2 + q2_dot * dT;
      q3 = q3 + q3_dot * dT;
  */
  float_prec q0, q1, q2, q3;
  float_prec p, q, r;

  q0 = X[0][0];
  q1 = X[1][0];
  q2 = X[2][0];
  q3 = X[3][0];

  p = U[0][0];
  q = U[1][0];
  r = U[2][0];

  X_Next[0][0] = (0.5 * (+0.00 - p * q1 - q * q2 - r * q3)) * SS_DT + q0;
  X_Next[1][0] = (0.5 * (+p * q0 + 0.00 + r * q2 - q * q3)) * SS_DT + q1;
  X_Next[2][0] = (0.5 * (+q * q0 - r * q1 + 0.00 + p * q3)) * SS_DT + q2;
  X_Next[3][0] = (0.5 * (+r * q0 + q * q1 - p * q2 + 0.00)) * SS_DT + q3;


  /* ======= Additional ad-hoc quaternion normalization to make sure the quaternion is a unit vector (i.e. ||q|| = 1) ======= */
  if (!X_Next.bNormVector()) {
    /* System error, return false so we can reset the UKF */
    return false;
  }

  return true;
}

bool Main_bUpdateNonlinearY(Matrix & Y, const Matrix & X, const Matrix & U)
{
  /* Insert the nonlinear measurement transformation here
              y(k)   = h[x(k), u(k)]

     The measurement output is the gravitational and magnetic projection to the body:
         DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                    2*(q1*q2+q0*q3),                    2*(q1*q3-q0*q2)]
                   [                   2*(q1*q2-q0*q3), (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                    2*(q2*q3+q0*q1)]
                   [                   2*(q1*q3+q0*q2),                    2*(q2*q3-q0*q1), (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]

      G_proj_sens = DCM * [0 0 1]             --> Gravitational projection to the accelerometer sensor
      M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
  */
  float_prec q0, q1, q2, q3;
  float_prec q0_2, q1_2, q2_2, q3_2;

  q0 = X[0][0];
  q1 = X[1][0];
  q2 = X[2][0];
  q3 = X[3][0];

  q0_2 = q0 * q0;
  q1_2 = q1 * q1;
  q2_2 = q2 * q2;
  q3_2 = q3 * q3;

  Y[0][0] = (2 * q1 * q3 - 2 * q0 * q2) * IMU_ACC_Z0;

  Y[1][0] = (2 * q2 * q3 + 2 * q0 * q1) * IMU_ACC_Z0;

  Y[2][0] = (+(q0_2) - (q1_2) - (q2_2) + (q3_2)) * IMU_ACC_Z0;

  Y[3][0] = (+(q0_2) + (q1_2) - (q2_2) - (q3_2)) * IMU_MAG_B0[0][0]
            + (2 * (q1 * q2 + q0 * q3)) * IMU_MAG_B0[1][0]
            + (2 * (q1 * q3 - q0 * q2)) * IMU_MAG_B0[2][0];

  Y[4][0] = (2 * (q1 * q2 - q0 * q3)) * IMU_MAG_B0[0][0]
            + (+(q0_2) - (q1_2) + (q2_2) - (q3_2)) * IMU_MAG_B0[1][0]
            + (2 * (q2 * q3 + q0 * q1)) * IMU_MAG_B0[2][0];

  Y[5][0] = (2 * (q1 * q3 + q0 * q2)) * IMU_MAG_B0[0][0]
            + (2 * (q2 * q3 - q0 * q1)) * IMU_MAG_B0[1][0]
            + (+(q0_2) - (q1_2) - (q2_2) + (q3_2)) * IMU_MAG_B0[2][0];

  return true;
}

bool Main_bCalcJacobianF(Matrix & F, const Matrix & X, const Matrix & U)
{
  //X를 예측하는 것이고 F는 비선형 함수를 야코비안 행렬로 만들어 준것이고, U는 입력 받은 놈
  /* In Main_bUpdateNonlinearX():
      q0 = q0 + q0_dot * dT;
      q1 = q1 + q1_dot * dT;
      q2 = q2 + q2_dot * dT;
      q3 = q3 + q3_dot * dT;
  */
  float_prec p, q, r;

  p = U[0][0];
  q = U[1][0];
  r = U[2][0];

  F[0][0] =  1.000;
  F[1][0] =  0.5 * p * SS_DT;
  F[2][0] =  0.5 * q * SS_DT;
  F[3][0] =  0.5 * r * SS_DT;

  F[0][1] = -0.5 * p * SS_DT;
  F[1][1] =  1.000;
  F[2][1] = -0.5 * r * SS_DT;
  F[3][1] =  0.5 * q * SS_DT;

  F[0][2] = -0.5 * q * SS_DT;
  F[1][2] =  0.5 * r * SS_DT;
  F[2][2] =  1.000;
  F[3][2] = -0.5 * p * SS_DT;

  F[0][3] = -0.5 * r * SS_DT;
  F[1][3] = -0.5 * q * SS_DT;
  F[2][3] =  0.5 * p * SS_DT;
  F[3][3] =  1.000;

  return true;
}

bool Main_bCalcJacobianH(Matrix & H, const Matrix & X, const Matrix & U)
{
  /* In Main_bUpdateNonlinearY():

     The measurement output is the gravitational and magnetic projection to the body:
         DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                    2*(q1*q2+q0*q3),                    2*(q1*q3-q0*q2)]
                   [                   2*(q1*q2-q0*q3), (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                    2*(q2*q3+q0*q1)]
                   [                   2*(q1*q3+q0*q2),                    2*(q2*q3-q0*q1), (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]

      G_proj_sens = DCM * [0 0 -g]            --> Gravitational projection to the accelerometer sensor
      M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
  */
  float_prec q0, q1, q2, q3;

  q0 = X[0][0];
  q1 = X[1][0];
  q2 = X[2][0];
  q3 = X[3][0];

  H[0][0] = -2 * q2 * IMU_ACC_Z0;
  H[1][0] = +2 * q1 * IMU_ACC_Z0;
  H[2][0] = +2 * q0 * IMU_ACC_Z0;
  H[3][0] =  2 * q0 * IMU_MAG_B0[0][0] + 2 * q3 * IMU_MAG_B0[1][0] - 2 * q2 * IMU_MAG_B0[2][0];
  H[4][0] = -2 * q3 * IMU_MAG_B0[0][0] + 2 * q0 * IMU_MAG_B0[1][0] + 2 * q1 * IMU_MAG_B0[2][0];
  H[5][0] =  2 * q2 * IMU_MAG_B0[0][0] - 2 * q1 * IMU_MAG_B0[1][0] + 2 * q0 * IMU_MAG_B0[2][0];

  H[0][1] = +2 * q3 * IMU_ACC_Z0;
  H[1][1] = +2 * q0 * IMU_ACC_Z0;
  H[2][1] = -2 * q1 * IMU_ACC_Z0;
  H[3][1] =  2 * q1 * IMU_MAG_B0[0][0] + 2 * q2 * IMU_MAG_B0[1][0] + 2 * q3 * IMU_MAG_B0[2][0];
  H[4][1] =  2 * q2 * IMU_MAG_B0[0][0] - 2 * q1 * IMU_MAG_B0[1][0] + 2 * q0 * IMU_MAG_B0[2][0];
  H[5][1] =  2 * q3 * IMU_MAG_B0[0][0] - 2 * q0 * IMU_MAG_B0[1][0] - 2 * q1 * IMU_MAG_B0[2][0];

  H[0][2] = -2 * q0 * IMU_ACC_Z0;
  H[1][2] = +2 * q3 * IMU_ACC_Z0;
  H[2][2] = -2 * q2 * IMU_ACC_Z0;
  H[3][2] = -2 * q2 * IMU_MAG_B0[0][0] + 2 * q1 * IMU_MAG_B0[1][0] - 2 * q0 * IMU_MAG_B0[2][0];
  H[4][2] =  2 * q1 * IMU_MAG_B0[0][0] + 2 * q2 * IMU_MAG_B0[1][0] + 2 * q3 * IMU_MAG_B0[2][0];
  H[5][2] =  2 * q0 * IMU_MAG_B0[0][0] + 2 * q3 * IMU_MAG_B0[1][0] - 2 * q2 * IMU_MAG_B0[2][0];

  H[0][3] = +2 * q1 * IMU_ACC_Z0;
  H[1][3] = +2 * q2 * IMU_ACC_Z0;
  H[2][3] = +2 * q3 * IMU_ACC_Z0;
  H[3][3] = -2 * q3 * IMU_MAG_B0[0][0] + 2 * q0 * IMU_MAG_B0[1][0] + 2 * q1 * IMU_MAG_B0[2][0];
  H[4][3] = -2 * q0 * IMU_MAG_B0[0][0] - 2 * q3 * IMU_MAG_B0[1][0] + 2 * q2 * IMU_MAG_B0[2][0];
  H[5][3] =  2 * q1 * IMU_MAG_B0[0][0] + 2 * q2 * IMU_MAG_B0[1][0] + 2 * q3 * IMU_MAG_B0[2][0];

  return true;
}

void SPEW_THE_ERROR(char const * str)
{
#if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
  cout << (str) << endl;
#elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
  Serial.println(str);
#else
  /* Silent function */
#endif
  while (1);
}
//////////////////////
void dualPID(float target_angle,
             float angle_in,
             float rate_in,
             float stabilize_kp,
             float stabilize_ki,
             float rate_kp,
             float rate_ki,
             float rate_kd,
             float &stabilize_iterm,
             float &rate_iterm,
             float &prev_error,
             float &output) {
  float angle_error;
  float desired_rate;
  float rate_error;
  float stabilize_pterm, rate_pterm, rate_dterm;

  //이중루프PID알고리즘//
  angle_error = target_angle * 1.0f - angle_in;

  stabilize_pterm = stabilize_kp * angle_error;
  stabilize_iterm += stabilize_ki * angle_error * dt; //안정화 적분항//
  desired_rate = stabilize_pterm;
  //---------------- p와 FF까지 적용된 부분----------------------------------------
  rate_error = desired_rate - rate_in;

  rate_pterm = rate_kp * rate_error; //각속도 비례항//
  rate_iterm += rate_ki * rate_error * dt; //각속도 적분항//
  rate_dterm += rate_kd * (rate_error - prev_error) / dt;
  prev_error = rate_error;
  output = rate_pterm + rate_iterm + rate_dterm + stabilize_iterm; //최종 출력 : 각속도 비례항 + 각속도 적분항 + 안정화 적분항//
}

///////////////////////
void calcYPRtoDualPID() {
  dualPID(roll_target_angle,
          roll_angle_in,
          roll_rate_in,
          roll_stabilize_kp,
          roll_stabilize_ki,
          roll_rate_kp,
          roll_rate_ki,
          roll_rate_kd,//
          roll_stabilize_iterm,
          roll_rate_iterm,
          roll_prev_error,
          roll_output
         );

  dualPID(pitch_target_angle,
          pitch_angle_in,
          pitch_rate_in,
          pitch_stabilize_kp,
          pitch_stabilize_ki,
          pitch_rate_kp,
          pitch_rate_ki,
          pitch_rate_kd,//
          pitch_stabilize_iterm,
          pitch_rate_iterm,
          pitch_prev_error,
          pitch_output
         );

  dualPID(yaw_target_angle,
          yaw_angle_in,
          yaw_rate_in,
          yaw_stabilize_kp,
          yaw_stabilize_ki,
          yaw_rate_kp,
          yaw_rate_ki,
          yaw_rate_kd,//
          yaw_stabilize_iterm,
          yaw_rate_iterm,
          yaw_prev_error,
          yaw_output
         );
}
///////////////////////
