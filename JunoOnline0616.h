// -*- C++ -*-
/*!
 * @file  Juno.h * @brief NolanComponent * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef JUNO_H
#define JUNO_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
//add
#include <rtm/CorbaNaming.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "JunoService_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "SequencePlayerServiceStub.h"
// OpenHRP
#include "hrpModel/Body.h"
#include "hrpModel/Link.h"
#include "hrpModel/JointPath.h"
#include "hrpModel/ModelLoaderUtil.h"
#include "hrpUtil/MatrixSolvers.h"
#include "hrpUtil/uBlasCommonTypes.h"
#include "hrpModel/Sensor.h"
#include <vector>
#include <iostream>
#include <sstream>
// </rtc-template>
#include <Eigen/Dense>
#include <omp.h>
//user
#include "myfunc.h"
#include "VectorConvert.h"
#include "twoIVK.h"
#include "ZmpPlaner.h"
#include "preview_control/PreviewControl.h"
//class
#include "NMSclassQ.h" 
using namespace RTC;

class Juno  : public RTC::DataFlowComponentBase
{
 public:
  Juno(RTC::Manager* manager);
  ~Juno();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry() 
  virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  // virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  // virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);

  //function
  inline void update_mc(BodyPtr body);
  inline void main_algorithm();
  inline void invoke();
  inline void fSensorRead();
  inline void rzmp2st();
  inline void calcWholeIVK();
  inline void zmpHandler();
  inline void getInvResult();
  inline void object_operate();
  inline void calcRefLeg();
  inline void prmGenerator(bool &flagA, bool &flagB);
  int stepLength(FootType FT, ZmpPlaner *zmpP);
  void waitingMotion(Vector3 &cm_ref, PreviewControl *PC, bool &comInpo_flag,bool &comInpo_stop);
  void start2walk(BodyPtr body, ZmpPlaner *zmpP, PreviewControl *PC, bool &stopflag, Vector3 cm_ref);
  void prm2Planzmp(FootType FT, Vector3 *p_ref, Matrix33 *R_ref, Vector3 RLEG_ref_p, Vector3 LLEG_ref_p, Matrix33 LEG_ref_R, std::deque<vector2> &rfzmp, ZmpPlaner *zmpP, const char *ChIn);
  void walkingMotion(BodyPtr body, FootType FT, Vector3 &cm_ref, Vector3 &absZMP, Vector3 *p_Init, Vector3 *p_ref, Matrix33 *R_ref, std::deque<vector2> &rfzmp, PreviewControl *PC, ZmpPlaner *zmpP, int &count);  
  bool ChangeSupLeg(BodyPtr body, FootType &FT, int &count, ZmpPlaner *zmpP, PreviewControl *PC, bool &stopflag, int &CommandIn, Vector3 *p_now, Vector3 *p_Init, Matrix33 *R_now, Matrix33 *R_Init, bool &comInpo_flag);
  void IniNewStep(BodyPtr body, FootType &FT, int &count,  ZmpPlaner *zmpP, PreviewControl *PC, bool &stopflag, int &CommandIn, Vector3 *p_ref, Vector3 *p_Init, Matrix33 *R_ref, Matrix33 *R_Init, bool &comInpo_flag);
  void onlineMonitor(BodyPtr body, FootType &FT, ZmpPlaner *zmpP, int &count, Vector3 *p_Init, Matrix33 *R_Init);

  //method
  void setObjectV(double x, double y, double z, double roll, double pitch, double yaw);
  void move_hand(double x, double y, double z, double roll, double pitch, double yaw);
  void setMaker(double RARM_x_A , double RARM_y_A, double LARM_x_A, double LARM_y_A,
                double RARM_x_B , double RARM_y_B, double LARM_x_B, double LARM_y_B);
  void setgain(double kp, double kd, double ki);
  void set_hr(double hx, double hy, double hz);
  void set_max_eh(double max_eh);

  void setWalkCommand(const char* ChIn);
  void preSet();
  void start();
  void testMove();
  void fcontrol();
  void fcontrol_write();
  void exPos();
  void stop();
  void hogex();
  void clear_log();
  void save_log();
  void clip_open_A();
  void clip_open_B();
  void clip_close_A();
  void clip_close_B();
  void stepping();

  //pthread
  void threadfunc_main();
  static void* invoke_function_main(void *obj) {
    Juno *juno = reinterpret_cast<Juno *>(obj);
    juno->threadfunc_main();
    return NULL;
  }

 
 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_mc_A;
  InPort<TimedDoubleSeq> m_mc_AIn;
  TimedDoubleSeq m_q_A;
  InPort<TimedDoubleSeq> m_q_AIn;
  TimedDoubleSeq m_rhsensor_A;
  InPort<TimedDoubleSeq> m_rhsensor_AIn;
  TimedDoubleSeq m_lhsensor_A;
  InPort<TimedDoubleSeq> m_lhsensor_AIn;
  TimedDoubleSeq m_rfsensor_A;
  InPort<TimedDoubleSeq> m_rfsensor_AIn;
  TimedDoubleSeq m_lfsensor_A;
  InPort<TimedDoubleSeq> m_lfsensor_AIn;
  TimedDoubleSeq m_baseRPYInit_A;
  InPort<TimedDoubleSeq> m_baseRPYInit_AIn;
  TimedDoubleSeq m_basePOSInit_A;
  InPort<TimedDoubleSeq> m_basePOSInit_AIn;
  TimedDoubleSeq m_mc_B;
  InPort<TimedDoubleSeq> m_mc_BIn;
  TimedDoubleSeq m_q_B;
  InPort<TimedDoubleSeq> m_q_BIn;
  TimedDoubleSeq m_rhsensor_B;
  InPort<TimedDoubleSeq> m_rhsensor_BIn;
  TimedDoubleSeq m_lhsensor_B;
  InPort<TimedDoubleSeq> m_lhsensor_BIn;
  TimedDoubleSeq m_rfsensor_B;
  InPort<TimedDoubleSeq> m_rfsensor_BIn;
  TimedDoubleSeq m_lfsensor_B;
  InPort<TimedDoubleSeq> m_lfsensor_BIn;
  TimedDoubleSeq m_baseRPYInit_B;
  InPort<TimedDoubleSeq> m_baseRPYInit_BIn;
  TimedDoubleSeq m_basePOSInit_B;
  InPort<TimedDoubleSeq> m_basePOSInit_BIn;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_rzmp_A;
  OutPort<TimedDoubleSeq> m_rzmp_AOut;
  TimedDoubleSeq m_refq_A;
  OutPort<TimedDoubleSeq> m_refq_AOut;
  TimedDoubleSeq m_baseRPY_A;
  OutPort<TimedDoubleSeq> m_baseRPY_AOut;
  TimedDoubleSeq m_basePOS_A;
  OutPort<TimedDoubleSeq> m_basePOS_AOut;
  TimedDoubleSeq m_rzmp_B;
  OutPort<TimedDoubleSeq> m_rzmp_BOut;
  TimedDoubleSeq m_refq_B;
  OutPort<TimedDoubleSeq> m_refq_BOut;
  TimedDoubleSeq m_baseRPY_B;
  OutPort<TimedDoubleSeq> m_baseRPY_BOut;
  TimedDoubleSeq m_basePOS_B;
  OutPort<TimedDoubleSeq> m_basePOS_BOut;
  TimedDoubleSeq m_Fobj;
  OutPort<TimedDoubleSeq> m_FobjOut;
  TimedDoubleSeq m_wZMP_A;
  OutPort<TimedDoubleSeq> m_wZMP_AOut;
  TimedDoubleSeq m_wZMP_B;
  OutPort<TimedDoubleSeq> m_wZMP_BOut;

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_JunoServicePort;
  RTC::CorbaPort m_ToSequencePlayerServicePort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  JunoService_impl m_service0;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  RTC::CorbaConsumer<OpenHRP::SequencePlayerService> m_serviceSeq0;

  // </rtc-template>

 private:
  //class
  NonMasterSlave *NMS;
  ZmpPlaner *zmpP_A;
  PreviewControl *PC_A;
  ZmpPlaner *zmpP_B;
  PreviewControl *PC_B;
  //paramater
  Properties prop;
  hrp::BodyPtr body_A;
  hrp::BodyPtr body_B;
  int dof_A;
  int dof_B;
  vector32 temp_q_A;
  vector30 temp_q_B;

  std::vector<double> kgain;
  std::vector<double> fgain;
  FootType FT_A;
  FootType FT_B;

  Vector3 p_now_A[LINKNUM];
  Matrix33 R_now_A[LINKNUM];
  Vector3 p_ref_A[LINKNUM];
  Matrix33 R_ref_A[LINKNUM];
  Vector3 p_Init_A[LINKNUM];
  Matrix33 R_Init_A[LINKNUM];

  Vector3 p_now_B[LINKNUM];
  Matrix33 R_now_B[LINKNUM];
  Vector3 p_ref_B[LINKNUM];
  Matrix33 R_ref_B[LINKNUM];
  Vector3 p_Init_B[LINKNUM];
  Matrix33 R_Init_B[LINKNUM];

  vector6 velobj;

  Vector3 cm_ref_A;
  Vector3 absZMP_A, relZMP_A;//abs to waist coordinat
  Vector3 wZMP_A;// calc form sensor
  vector2 elbow_A;
  std::deque<vector2> clipDeque_A;

  Vector3 cm_ref_B;
  Vector3 absZMP_B, relZMP_B;//abs to waist coordinat
  Vector3 wZMP_B;// calc form sensor
  vector2 elbow_B;
  std::deque<vector2> clipDeque_B;
  //walking para
  bool playflag;
  bool stopflag_A;
  bool stopflag_B;
  int count_A;
  int count_B;
  bool flagcalczmp_A;
  bool flagcalczmp_B;
  std::deque<vector2> rfzmp_A;
  std::deque<vector2> rfzmp_B;

  bool exForceMode;
  bool forceControl;
  bool forceControl_write;
  //bool wflagRzmp;
  int CommandIn_A;
  int CommandIn_B;
  double time2Neutral;
  //NMS
  hrp::Link* stick_r_A;
  hrp::Link* stick_l_A;
  hrp::Link* stick_r_B;
  hrp::Link* stick_l_B;
  Vector3 stick_rpy_r_A_buf, stick_rpy_l_A_buf;
  Vector3 stick_rpy_r_B_buf, stick_rpy_l_B_buf;
  hrp::Link* object_ref;
  dvector eh_buf;
  //Path planning
  Vector3 p_obj2RLEG_A,p_obj2LLEG_A;
  Vector3 p_obj2RLEG_B,p_obj2LLEG_B;
  Matrix33 R_LEG_ini_A,R_LEG_ini_B;

  Vector3 RLEG_ref_p_A,LLEG_ref_p_A;
  Vector3 RLEG_ref_p_B,LLEG_ref_p_B;
  Matrix33 LEG_ref_R_A;
  Matrix33 LEG_ref_R_B;

  //Eigen::MatrixXd gh;
  //test MySequencePlayer
  bool logFlag;
  vector32 body_ref_A,body_cur_A,body_ini_A;
  vector30 body_ref_B,body_cur_B,body_ini_B;
  std::deque<vector32> bodyDeque_A;
  std::deque<vector30> bodyDeque_B;
  std::deque<vector32> bodyDeque_pA;
  std::deque<vector30> bodyDeque_pB;

  std::deque<Vector3> rzmpDeque_A;
  std::deque<Vector3> rzmpDeque_B;
  std::deque<Vector3> waist_p_Deque_A;
  std::deque<Vector3> waist_p_Deque_B;
  std::deque<Matrix33> waist_R_Deque_A;
  std::deque<Matrix33> waist_R_Deque_B;

  Vector3 crr;//for log
  std::deque<vector2> rpdeque;
  vector2 rpcrr;
  double yawTotal;
  Matrix33 rotRTemp;
  //for my log
  std::deque<TimedDoubleSeq> rhsensor_deque_A;
  std::deque<TimedDoubleSeq> lhsensor_deque_A;
  std::deque<TimedDoubleSeq> rhsensor_deque_B;
  std::deque<TimedDoubleSeq> lhsensor_deque_B;
  std::deque<vector32> bodyDeque_log_A;
  std::deque<vector30> bodyDeque_log_B;

  std::deque<Vector3> cmDeque_log_A;
  std::deque<Vector3> cmDeque_log_B;
  std::deque<Vector3> object_p_Deque_log;
  std::deque<Vector3> absZMPDeque_log_A;
  std::deque<Vector3> absZMPDeque_log_B;
  //online log
  std::deque<Vector3> wZMPDeque_log_A;
  std::deque<Vector3> wZMPDeque_log_B;
  Vector3 waist_p_ini_A,waist_p_ini_B;
  Matrix33 waist_R_ini_A,waist_R_ini_B;

  bool step;
  int MAX_TRANSITION_COUNT;
  double transition_count;
  vector32 qRef_tem_A;
  vector30 qRef_tem_B;

  bool ref_q_load_A;
  bool ref_q_load_B;
  int step_num;
  bool write_flag;
  bool comInpo_flag_A;
  bool comInpo_flag_B;
  bool comInpo_stop_A;
  bool comInpo_stop_B;

  //for optcap
  bool optcap,RH;
  Vector3 maker_r_A;
  Vector3 maker_l_A;
  Vector3 maker_r_B;
  Vector3 maker_l_B;

  //for inipos adjust
  hrp::Link* stick_r_single;
  hrp::Link* stick_l_single;
  hrp::Link* object_ref_single;

  //for count
  int cmain;
  int cA;
  int cB;

  //pthread
  pthread_t tmain;
  bool flag_t;
  pthread_mutex_t mutexA;
  pthread_mutex_t mutexB;
  pthread_mutex_t mutexA_zmp;
  pthread_mutex_t mutexB_zmp;

};


extern "C"
{
  DLL_EXPORT void JunoInit(RTC::Manager* manager);
};

#endif // JUNO_H

