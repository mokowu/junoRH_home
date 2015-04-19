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
  // virtual RTC::ReturnCode_t onFinalize();

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
  inline void rzmp2st();
  inline void calcWholeIVK();
  inline void getInvResult();
  inline void object_operate();
  inline void calcRefLeg();
  inline void prmGenerator();
  //method
  void setObjectV(double x, double y, double z, double roll, double pitch, double yaw);
  void setWalkCommand(const char* ChIn);
  void start();
  void testMove();
  void fcontrol();
  void stop();


 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_mc_A;
  InPort<TimedDoubleSeq> m_mc_AIn;
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
  int dof;
  int armDof;

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

  Vector3 cm ; 
  vector6 velobj;

  Vector3 cm_ref_A;
  Vector3 absZMP_A, relZMP_A;//abs to waist coordinat
  Vector3 wZMP_A;// calc form sensor
  vector2 elbowAngleIni_A;
  std::deque<vector2> clipDeque_A;

  Vector3 cm_ref_B;
  Vector3 absZMP_B, relZMP_B;//abs to waist coordinat
  Vector3 wZMP_B;// calc form sensor
  vector2 elbowAngleIni_B;
  std::deque<vector2> clipDeque_B;
  
  bool playflag;
  bool stopflag_A;
  bool stopflag_B;
  bool flagcalczmp;
  bool exForceMode;
  std::deque<vector2> rfzmp_A;
  std::deque<vector2> rfzmp_B;

  int count;
  bool wflagRzmp;
  int CommandIn;

  double time2Neutral;
  //NMS
  hrp::Link* stick_r_A;
  hrp::Link* stick_l_A;
  hrp::Link* stick_r_B;
  hrp::Link* stick_l_B;
  hrp::Link* object_ref;
  dvector eh;
  //Path planning
  Vector3 p_obj2RLEG_A,p_obj2LLEG_A;
  Vector3 p_obj2RLEG_B,p_obj2LLEG_B;
  Matrix33 R_LEG_ini_A,R_LEG_ini_B;

  Vector3 RLEG_ref_p_A,LLEG_ref_p_A;
  Vector3 RLEG_ref_p_B,LLEG_ref_p_B;
  Matrix33 RLEG_ref_R_A,LLEG_ref_R_A;
  Matrix33 RLEG_ref_R_B,LLEG_ref_R_B;

  //test MySequencePlayer
  bool hoge;
  vector32 body_cur_A,body_cur_B;
  vector32 body_ref_A,body_ref_B;
  std::deque<vector32> bodyDeque_A;
  std::deque<vector32> bodyDeque_B;
  
};


extern "C"
{
  DLL_EXPORT void JunoInit(RTC::Manager* manager);
};

#endif // JUNO_H

