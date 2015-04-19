// -*- C++ -*-
/*!
 * @file  Juno.cpp * @brief NolanComponent * $Date$ 
 *
 * $Id$ 
 */
#include "Juno.h"
std::ofstream ofs("/home/grxuser/users/wu/junoRH/juno.log");
// Module specification
// <rtc-template block="module_spec">
static const char* juno_spec[] =
  {
    "implementation_id", "Juno",
    "type_name",         "Juno",
    "description",       "JunoComponent",
    "version",           "1.0",
    "vendor",            "tohoku",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "JunoComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    ""
  };
// </rtc-template>

Juno::Juno(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_mc_AIn("mc_A", m_mc_A),
    m_q_AIn("q_A", m_q_A),
    m_rhsensor_AIn("rhsensor_A", m_rhsensor_A),
    m_lhsensor_AIn("lhsensor_A", m_lhsensor_A),
    m_rfsensor_AIn("rfsensor_A", m_rfsensor_A),
    m_lfsensor_AIn("lfsensor_A", m_lfsensor_A),
    m_baseRPYInit_AIn("baseRPYInit_A", m_baseRPYInit_A),
    m_basePOSInit_AIn("basePOSInit_A", m_basePOSInit_A),
    m_mc_BIn("mc_B", m_mc_B),
    m_q_BIn("q_B", m_q_B),
    m_rhsensor_BIn("rhsensor_B", m_rhsensor_B),
    m_lhsensor_BIn("lhsensor_B", m_lhsensor_B),
    m_rfsensor_BIn("rfsensor_B", m_rfsensor_B),
    m_lfsensor_BIn("lfsensor_B", m_lfsensor_B),
    m_baseRPYInit_BIn("baseRPYInit_B", m_baseRPYInit_B),
    m_basePOSInit_BIn("basePOSInit_B", m_basePOSInit_B),
    m_rzmp_AOut("rzmp_A", m_rzmp_A),
    m_refq_AOut("refq_A", m_refq_A),
    m_baseRPY_AOut("baseRPY_A", m_baseRPY_A),
    m_basePOS_AOut("basePOS_A", m_basePOS_A),
    m_rzmp_BOut("rzmp_B", m_rzmp_B),
    m_refq_BOut("refq_B", m_refq_B),
    m_baseRPY_BOut("baseRPY_B", m_baseRPY_B),
    m_basePOS_BOut("basePOS_B", m_basePOS_B),
    m_FobjOut("Fobj", m_Fobj),
    m_wZMP_AOut("wZMP_A", m_wZMP_A),
    m_wZMP_BOut("wZMP_B", m_wZMP_B),
    m_JunoServicePort("JunoService"),
    m_ToSequencePlayerServicePort("ToSequencePlayerService")

    // </rtc-template>
{
  m_service0.setComponent(this);
}

Juno::~Juno()
{
}


RTC::ReturnCode_t Juno::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("mc_A", m_mc_AIn);
  addInPort("q_A", m_q_AIn);
  addInPort("rhsensor_A", m_rhsensor_AIn);
  addInPort("lhsensor_A", m_lhsensor_AIn);
  addInPort("rfsensor_A", m_rfsensor_AIn);
  addInPort("lfsensor_A", m_lfsensor_AIn);
  addInPort("baseRPYInit_A", m_baseRPYInit_AIn);
  addInPort("basePOSInit_A", m_basePOSInit_AIn);
  addInPort("mc_B", m_mc_BIn);
  addInPort("q_B", m_q_BIn);
  addInPort("rhsensor_B", m_rhsensor_BIn);
  addInPort("lhsensor_B", m_lhsensor_BIn);
  addInPort("rfsensor_B", m_rfsensor_BIn);
  addInPort("lfsensor_B", m_lfsensor_BIn);
  addInPort("baseRPYInit_B", m_baseRPYInit_BIn);
  addInPort("basePOSInit_B", m_basePOSInit_BIn);

  // Set OutPort buffer
  addOutPort("rzmp_A", m_rzmp_AOut);
  addOutPort("refq_A", m_refq_AOut);
  addOutPort("baseRPY_A", m_baseRPY_AOut);
  addOutPort("basePOS_A", m_basePOS_AOut);
  addOutPort("rzmp_B", m_rzmp_BOut);
  addOutPort("refq_B", m_refq_BOut);
  addOutPort("baseRPY_B", m_baseRPY_BOut);
  addOutPort("basePOS_B", m_basePOS_BOut);
  addOutPort("Fobj", m_FobjOut);
  addOutPort("wZMP_A", m_wZMP_AOut);
  addOutPort("wZMP_B", m_wZMP_BOut);
  // Set service provider to Ports
  m_JunoServicePort.registerProvider("service0", "JunoService", m_service0);

  // Set service consumers to Ports
  //m_ToSequencePlayerServicePort.registerConsumer("serviceSeq0", "SequencePlayerService", m_serviceSeq0);

  // Set CORBA Service Ports
  addPort(m_JunoServicePort);
  //addPort(m_ToSequencePlayerServicePort);

  prop=this->getProperties();
  
  /*
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int commaPos = nameServer.find(",");
  if (commaPos > 0)
    nameServer = nameServer.substr(0, commaPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  body_A = new hrp::Body();
  body_B = new hrp::Body();
  CosNaming::NamingContext_var m_rootNameContext = CosNaming::NamingContext::_duplicate(naming.getRootContext());
  int argc=0;
  char* argv[argc];
  if (!hrp::loadBodyFromModelLoader(body_A,  prop["model"].c_str(), m_rootNameContext))
    //if (!hrp::loadBodyFromModelLoader(body,  URL, m_rootNameContext))
    //if (!hrp::loadBodyFromModelLoader(body, URL, argc,argv))
    {
      std::cerr <<" : failed to load model" << std::endl;
    }
  if (!hrp::loadBodyFromModelLoader(body_B,  prop["model15B"].c_str(), m_rootNameContext))
    {
      std::cerr <<" : failed to load model" << std::endl;
    }
  ////
  */

  // parameters for corba
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
    comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());

  // parameters for internal robot model
  body_A = hrp::BodyPtr(new hrp::Body());
  if (!loadBodyFromModelLoader(body_A, prop["model"].c_str(), 
                               CosNaming::NamingContext::_duplicate(naming.getRootContext())
                               )){
    std::cerr << "failed to load model[" << prop["model"] << "]" 
              << std::endl;
    return RTC::RTC_ERROR;
  }
  body_A->totalMass();
  //B
  body_B = hrp::BodyPtr(new hrp::Body());
  if (!loadBodyFromModelLoader(body_B, prop["model15B"].c_str(), 
                               CosNaming::NamingContext::_duplicate(naming.getRootContext())
                               )){
    std::cerr << "failed to load model[" << prop["model15B"] << "]" 
              << std::endl;
    return RTC::RTC_ERROR;
  }
  body_B->totalMass();

  ////
  dof_A = body_A->numJoints();
  dof_B = body_B->numJoints();

  prop["kgain"]>>kgain;
  prop["fgain"]>>fgain;
  ///////////should renew from optcap//////////
  //A
  for(unsigned int i=0;i<dof_A;i++)
    body_A->joint(i)->q= body_cur_A[i]=0; 
  body_A->link("WAIST")->p=-0.6169, 0, 0.705;
  body_A->calcForwardKinematics();
  RenewModel(body_A, p_now_A, R_now_A);
  RenewModel(body_A, p_ref_A, R_ref_A);//no need
  updateInit(p_ref_A, p_Init_A, R_ref_A, R_Init_A);
  body_A->calcCM();
  //B
  dof_B = body_B->numJoints();//careful
  for(unsigned int i=0;i<dof_B;i++)
    body_B->joint(i)->q= body_cur_B[i]=0; 
  body_B->link("WAIST")->p=0.6169, 0, 0.705;//careful
  //180 rotate
  body_B->link("WAIST")->R=rotationZ(M_PI);//caution!!!!!!!!!!
  body_B->calcForwardKinematics();
  RenewModel(body_B, p_now_B, R_now_B);
  RenewModel(body_B, p_ref_B, R_ref_B);
  updateInit(p_ref_B, p_Init_B, R_ref_B, R_Init_B);
  body_B->calcCM(); 
  ////////////////////////
  //pini
  playflag=0;
  stopflag_A=1;
  stopflag_B=1;
  count_A=0;
  count_B=0;
  //data port
  m_mc_A.data.length(dof_A);
  m_q_A.data.length(dof_A);
  m_rhsensor_A.data.length(6);
  m_lhsensor_A.data.length(6);
  m_rfsensor_A.data.length(6);
  m_lfsensor_A.data.length(6);  
  m_baseRPYInit_A.data.length(3);
  m_basePOSInit_A.data.length(3); 
  
  m_rzmp_A.data.length(3); 
  m_refq_A.data.length(dof_A);
  m_baseRPY_A.data.length(3); 
  m_basePOS_A.data.length(3); 

  m_mc_B.data.length(dof_B);
  m_q_B.data.length(dof_B);
  m_rhsensor_B.data.length(6);
  m_lhsensor_B.data.length(6);
  m_rfsensor_B.data.length(6);
  m_lfsensor_B.data.length(6);  
  m_baseRPYInit_B.data.length(3);
  m_basePOSInit_B.data.length(3); 
  
  m_rzmp_B.data.length(3); 
  m_refq_B.data.length(dof_B);
  m_baseRPY_B.data.length(3); 
  m_basePOS_B.data.length(3); 
  
  absZMP_A=absZMP_B=(Vector3)(0);
  relZMP_A=relZMP_B=(Vector3)(0);
  absZMP_A(0)=-0.6169;//becareful!!!!
  absZMP_B(0)=0.6169;//becareful!!!!

  flagcalczmp_A=0;
  flagcalczmp_B=0;
  FT_A =FSRFsw;
  FT_B =FSRFsw;
  CommandIn_A=CommandIn_B=5;
  time2Neutral=0.5;

  //test paraini
  velobj=(vector6)(0);

  //NMS
  NMS= new NonMasterSlave( prop["model"], prop["model15B"]);
  stick_r_A= new hrp::Link();
  stick_l_A= new hrp::Link();
  stick_r_B= new hrp::Link();
  stick_l_B= new hrp::Link();
  object_ref= new hrp::Link();
  eh_buf=dzerovector(dof_A+dof_B);//careful
  forceControl=0;

  stick_rpy_r_A_buf=stick_rpy_l_A_buf=Vector3(0);
  stick_rpy_r_B_buf=stick_rpy_l_B_buf=Vector3(0);
  /*
  //chev filter class
  CF=new chevFilter;
  CF->reset();
  //Impedance
  Imp=new Impedance;
  forceIn=(vector6)(0);
  exForceMode=0;
  ForceControl=0;
  */
  // </rtc-template> 
  
  MAX_TRANSITION_COUNT= 50;
  transition_count= -MAX_TRANSITION_COUNT;
  step=0;

  //log
  m_Fobj.data.length(6); 

  //pini2
  rpcrr=vector2(0);
  yawTotal=0;
  logFlag=0;

  ref_q_load_A=ref_q_load_B=0;
  //for offline
  step_num=0;
  write_flag=0;
  comInpo_flag_A=0;
  comInpo_flag_B=0;
  comInpo_stop_A=0;
  comInpo_stop_B=0;

  //optcap 
  optcap=0;
  RH=1;
  //for inipos adjust
  stick_r_single= new hrp::Link();
  stick_l_single= new hrp::Link();
  object_ref_single= new hrp::Link();

  // </rtc-template>
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Juno::onExecute(RTC::UniqueId ec_id)
{  

  //timeval tv_s;
  //gettimeofday(&tv_s,0);
  //cout << tv.tv_usec << std::endl;

  //if( !m_mc_AIn.isNew()&&!m_mc_BIn.isNew())
  //return RTC::RTC_OK;

  //use this in RH
  if( !m_q_AIn.isNew()&&!m_q_BIn.isNew())
    return RTC::RTC_OK;
  
  /*
  if( m_mc_AIn.isNew()){
    m_mc_AIn.read();
    ref_q_load_A=1;
    //cout<<"NewMC"<<endl;
  }
  if( m_mc_BIn.isNew()){
    m_mc_BIn.read();
    ref_q_load_B=1;
    //cout<<"NewMC"<<endl;
  }
  */

  //invoke();
  //if( !ref_q_load_A&&!ref_q_load_B)
  //  return RTC::RTC_OK;
  
  
  //A 
  if( m_rhsensor_AIn.isNew() ){ 
    m_rhsensor_AIn.read();
    //if(logFlag)
    //  rhsensor_deque_A.push_back(m_rhsensor_A);
  }
  if( m_lhsensor_AIn.isNew() ) {
    m_lhsensor_AIn.read();
    //if(logFlag)
    //  lhsensor_deque_A.push_back(m_lhsensor_A);
  }
  //B
  if( m_rhsensor_BIn.isNew() ){ 
    m_rhsensor_BIn.read();
    //if(logFlag)
    //  rhsensor_deque_B.push_back(m_rhsensor_B);
  }
  if( m_lhsensor_BIn.isNew() ){ 
    m_lhsensor_BIn.read();
    //if(logFlag)
    //  lhsensor_deque_B.push_back(m_lhsensor_B);
  }

  
  //for log wzmp
  //A
  if( m_rfsensor_AIn.isNew() ) 
    m_rfsensor_AIn.read();
  if( m_lfsensor_AIn.isNew() ) 
    m_lfsensor_AIn.read();
  //B
  if( m_rfsensor_BIn.isNew() ) 
    m_rfsensor_BIn.read();
  if( m_lfsensor_BIn.isNew() ) 
    m_lfsensor_BIn.read();
  
 
  //_/main algorithm_/_/_/_/

  if(forceControl)
    NMS->controlRoutine(FT_A, m_rhsensor_A, m_lhsensor_A, m_mc_A, body_A, 
                      FT_B, m_rhsensor_B, m_lhsensor_B, m_mc_B, body_B,  
	      R_now_A[WAIST],R_now_B[WAIST],p_now_A[WAIST],p_now_B[WAIST]);

  invoke();

  //timeval tv_e;
  //gettimeofday(&tv_e,0);
  //cout << tv_e.tv_usec- tv_s.tv_usec << std::endl;

  return RTC::RTC_OK;
}//onExecute end
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
   
//function
inline void Juno::getInvResult()
{
 getModelPosture(body_A, m_refq_A);
 getModelPosture(body_B, m_refq_B);

 ////////////////////////////////
 if(transition_count < 0){
   double transition_smooth_gain = 1/(1+exp(-9.19*(((MAX_TRANSITION_COUNT + transition_count) / MAX_TRANSITION_COUNT) - 0.5)));
   transition_count++;
   for ( int i = 0; i < body_A->numJoints(); i++ )
     m_refq_A.data[i]  = ( m_refq_A.data[i] - qRef_tem_A(i) ) * transition_smooth_gain + qRef_tem_A(i);
   for ( int i = 0; i < body_B->numJoints(); i++ )
     m_refq_B.data[i]  = ( m_refq_B.data[i] - qRef_tem_B(i) ) * transition_smooth_gain + qRef_tem_B(i);
 }
 ///////////////////////////////

 /*
 if(forceControl){
   
   //integrator
   //dvector eh =  hrp::dzerovector(dof_A+dof_B);
   //NMS->get_eh(eh);
   //eh_buf+=eh;
   
   //non integrator
   NMS->get_eh(eh_buf);

   for(int i = 0; i < dof_A; i++) {
     m_refq_A.data[i]+= eh_buf[i];
   }
   for(int i = 0; i < dof_B; i++) { 
     m_refq_B.data[i]+= eh_buf[i+dof_A];
   }  

   //for clip 
   m_refq_A.data[23]=body_A->joint(23)->q=-0.017;
   m_refq_A.data[31]=body_A->joint(31)->q=0.017;
   m_refq_B.data[22]=body_B->joint(22)->q=0.006;
   m_refq_B.data[29]=body_B->joint(29)->q=-0.006;
 
 }
*/

 /*
 //clip
  if(!clipDeque_A.empty()){
    m_refq_A.data[23]=body_A->joint(23)->q=clipDeque_A.at(0)[0];
    m_refq_A.data[31]=body_A->joint(31)->q=clipDeque_A.at(0)[1];
    clipDeque_A.pop_front();
  }

  if(!clipDeque_B.empty()){
    m_refq_B.data[22]=body_B->joint(22)->q=clipDeque_B.at(0)[0];
    m_refq_B.data[29]=body_B->joint(29)->q=clipDeque_B.at(0)[1];
    clipDeque_B.pop_front();
  }
 */
  
  //final
  //m_refq_AOut.write();
  vector32 temp_q_A;
  for(int i=0;i<32;i++)
    temp_q_A(i)=m_refq_A.data[i];
  bodyDeque_A.push_back(temp_q_A);
  //bodyDeque_log_A.push_back(temp_q_A);
   
  //m_refq_BOut.write();
  vector30 temp_q_B;
  for(int i=0;i<30;i++)
    temp_q_B(i)=m_refq_B.data[i];
  bodyDeque_B.push_back(temp_q_B);
  //bodyDeque_log_B.push_back(temp_q_B);
  
}
inline void Juno::calcWholeIVK()
{
  //timeval tv_s, tv_e;
  //gettimeofday(&tv_s,0);
  /*
  if(CalcIVK_NMS_elbow(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, stick_r_A, stick_l_A, 
  	       body_B, cm_ref_B, p_ref_B, R_ref_B, FT_B, p_Init_B, R_Init_B, stick_r_B, stick_l_B, 
   stick_rpy_r_A_buf, stick_rpy_l_A_buf, stick_rpy_r_B_buf, stick_rpy_l_B_buf, object_ref, elbow_A))
  //if(CalcIVK_NMS(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, stick_r_A, stick_l_A, 
  //	       body_B, cm_ref_B, p_ref_B, R_ref_B, FT_B, p_Init_B, R_Init_B, stick_r_B, stick_l_B, 
  //	       stick_rpy_r_A_buf, stick_rpy_l_A_buf, stick_rpy_r_B_buf, stick_rpy_l_B_buf, object_ref)) 
  getInvResult();
  else
    cerr<<"ivk err"<<endl;
  */

  
  //calculate once  
  if(CalcIVK_NMS(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, stick_r_A, stick_l_A, 
	 body_B, cm_ref_B, p_ref_B, R_ref_B, FT_B, p_Init_B, R_Init_B, stick_r_B, stick_l_B, 
	stick_rpy_r_A_buf, stick_rpy_l_A_buf, stick_rpy_r_B_buf, stick_rpy_l_B_buf, object_ref)){
    if(CalcIVK_ARM_NUM_elbow(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, elbow_A))
       getInvResult();
    else{
      cerr<<"err 2"<<endl;
      playflag=0;//stop calculate
    }
  }
  else{
    cerr<<"ivk err"<<endl;
    playflag=0;
  }
  
  cmDeque_log_A.push_back(cm_ref_A);
  cmDeque_log_B.push_back(cm_ref_B);
  waist_p_Deque_A.push_back(body_A->link("WAIST")->p);
  waist_p_Deque_B.push_back(body_B->link("WAIST")->p);
  waist_R_Deque_A.push_back(R_ref_A[WAIST]);
  waist_R_Deque_B.push_back(R_ref_B[WAIST]);

  /*
  //calculate seperate
  CalcIVK_NMS_ARM(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, stick_r_A, stick_l_A, 
	  body_B, cm_ref_B, p_ref_B, R_ref_B, FT_B, p_Init_B, R_Init_B, stick_r_B, stick_l_B, 
	stick_rpy_r_A_buf, stick_rpy_l_A_buf, stick_rpy_r_B_buf, stick_rpy_l_B_buf, object_ref);
  
  //parallel compute
  int r,g;
#pragma omp parallel sections
  {
#pragma omp section
    r=CalcIVK_ARM_NUM_elbow(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, elbow_A);
#pragma omp section
    g=CalcIVK_ARM_NUM(body_B, cm_ref_B, p_ref_B, R_ref_B, FT_B, p_Init_B, R_Init_B);
}
  if(r&&g)
    getInvResult();
  else
    cerr<<"err ivk shit!"<<endl;
  */
  //serial compute
  //if(CalcIVK_ARM_NUM_elbow(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, elbow_A)&&
  //         CalcIVK_ARM_NUM(body_B, cm_ref_B, p_ref_B, R_ref_B, FT_B, p_Init_B, R_Init_B))
  //getInvResult();
  //else
  //cerr<<"err ivk shit!"<<endl;
  
  //gettimeofday(&tv_e,0);
  //cout << tv_e.tv_usec- tv_s.tv_usec << std::endl;
}

inline void Juno::update_mc(BodyPtr body)
{
  if(body->numJoints()==32){
    //A 
    if(RH)
      m_mc_AIn.read();//comment in non RH
    for(unsigned int i=0;i<m_mc_A.data.length();i++)
      m_refq_A.data[i]=m_mc_A.data[i];
    setModelPosture(body_A, m_mc_A, FT_A, p_Init_A, R_Init_A);
    //RenewModel(body_A, p_now_A, R_now_A);
  }
  else if(body->numJoints()==30){
    //B
    if(RH)
      m_mc_BIn.read();//comment in non RH
    for(unsigned int i=0;i<m_mc_B.data.length();i++)
      m_refq_B.data[i]=m_mc_B.data[i];
    setModelPosture(body_B, m_mc_B, FT_B, p_Init_B, R_Init_B);
    //RenewModel(body_B, p_now_B, R_now_B);
  }
}

inline void Juno::main_algorithm()
{
   //_/_/_/_/_/_/_/_/_/_/_/_/main algorithm_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
  if(playflag){
  
    //ofs<<cm_ref_A(0)<<" "<<cm_ref_A(1)<<" "<<cm_ref_A(2)<<endl;
    //for force control
    //NMS->controlRoutine(FT_A, m_rhsensor_A, m_lhsensor_A, m_mc_A, body_A, 
    //                            FT_B, m_rhsensor_B, m_lhsensor_B, m_mc_B, body_B, R_ref_A, R_ref_B);
    object_operate();   
    prmGenerator( flagcalczmp_A, flagcalczmp_B);//stopflag off here
    ///////AAAAAAAAAAAA////////////////////////////////////////////////////
    if(stopflag_A ){//waiting
      waitingMotion(cm_ref_A, PC_A, comInpo_flag_A, comInpo_stop_A);
    }
    else{//walking_A
      walkingMotion(body_A, FT_A, cm_ref_A, absZMP_A, p_Init_A, p_ref_A, R_ref_A, rfzmp_A, PC_A, zmpP_A, count_A);
    }    
    ///////BBBBBBBBBBB/////////////////////////////////////////////////////
    if(stopflag_B ){//waiting
      waitingMotion(cm_ref_B, PC_B, comInpo_flag_B, comInpo_stop_B);
    }
    else{//walking_B
      walkingMotion(body_B, FT_B, cm_ref_B, absZMP_B, p_Init_B, p_ref_B, R_ref_B, rfzmp_B, PC_B, zmpP_B, count_B);
    }
    //
    if(comInpo_stop_A&&comInpo_stop_B){
      playflag=0;//stop writing in deque
      //for offline
      comInpo_stop_A=0;
      comInpo_stop_B=0;
      step_num=0;
      cout<<"cal end"<<endl;
    }

    calcWholeIVK();//write in refq_deque here
    zmpHandler();
    
    //for next step
    if(ChangeSupLeg(body_A, FT_A, count_A, zmpP_A, PC_A, stopflag_A, CommandIn_A, p_ref_A, p_Init_A, R_ref_A, R_Init_A, comInpo_flag_A))
      flagcalczmp_A=1;
    if(ChangeSupLeg(body_B, FT_B, count_B, zmpP_B, PC_B, stopflag_B, CommandIn_B, p_ref_B, p_Init_B, R_ref_B, R_Init_B, comInpo_flag_B))
      flagcalczmp_B=1;
    //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/  
    //////////////write///////////////   
    rzmp2st();//write in refzmp_deque here
    
    
    /*
    //for log RTC
    m_wZMP_A=calcZMPTDS(body_A, m_rfsensor_A, m_lfsensor_A);
    m_wZMP_B=calcZMPTDS(body_B, m_rfsensor_B, m_lfsensor_B);
    m_wZMP_AOut.write();
    m_wZMP_BOut.write();
    m_FobjOut.write(); 
    */

    /*
    Vector3 cmcA=body_A->calcCM();
    Vector3 cmcB=body_B->calcCM();
    if(logFlag){
    ofs<<   absZMP_A[0]<<" "<<   absZMP_A[1]<<" "<<m_wZMP_A.data[0]<<" "<<m_wZMP_A.data[1]<<" "<<cmcA(0)<<" "<<cmcA(1)<<" "<<
      absZMP_B[0]<<" "<<   absZMP_B[1]<<" "<<m_wZMP_B.data[0]<<" "<<m_wZMP_B.data[1]<<" "<<cmcB(0)<<" "<<cmcB(1)<<" "<<object_ref->p(0)<<" "<<object_ref->p(1)<<" "<<crr(0)<<" "<<crr(1)<<endl;
      }*/

  }//playflag

}

inline void Juno::invoke()
{
  if(!write_flag)
    return;

  //for force control
  if(forceControl)
  NMS->controlRoutine(FT_A, m_rhsensor_A, m_lhsensor_A, m_mc_A, body_A, 
                      FT_B, m_rhsensor_B, m_lhsensor_B, m_mc_B, body_B,  
	      R_now_A[WAIST],R_now_B[WAIST],p_now_A[WAIST],p_now_B[WAIST]);
  
#pragma omp parallel sections
  {
    
#pragma omp section
    //if( m_mc_AIn.isNew()){
    //m_mc_AIn.read();
    if( m_q_AIn.isNew()){
      m_q_AIn.read();
      
      //cout<<"NewMC"<<endl;
      for(int i=0;i<dof_A;i++)
	m_refq_A.data[i]=body_A->joint(i)->q;
      //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
      if(!bodyDeque_A.empty()&&logFlag){
	for(int i=0;i<dof_A;i++){
	  m_refq_A.data[i]=body_A->joint(i)->q=bodyDeque_A.at(0)[i];
	}
	bodyDeque_A.pop_front();
	
	if(!waist_p_Deque_A.empty()){
	  //renew model online
	  p_now_A[WAIST]=body_A->link("WAIST")->p= waist_p_Deque_A.at(0);
	  R_now_A[WAIST]=body_A->link("WAIST")->R= waist_R_Deque_A.at(0);
	  body_A->calcForwardKinematics();
	  waist_p_Deque_A.pop_front();
	  waist_R_Deque_A.pop_front();
	  wZMP_A=calcZMP(body_A, m_rfsensor_A, m_lfsensor_A);
	  wZMPDeque_log_A.push_back(wZMP_A);
	}
	
	//rzmp
	if(!rzmpDeque_A.empty()){
	  for(int i=0;i< m_rzmp_A.data.length();i++)
	    m_rzmp_A.data[i]=rzmpDeque_A.at(0)[i];  
	  rzmpDeque_A.pop_front();
	  m_rzmp_AOut.write();
	}
	/*//force control only when playstart
	  if(forceControl){
	  for(int i=0;i<dof_A;i++)
	  m_refq_A.data[i]= body_A->joint(i)->q+eh_buf[i];      
	  }
	*/
	
      }//if(!bodyDeque_A.empty()&&logFlag)
       //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
      
      if(forceControl){
	for(int i=0;i<dof_A;i++){
	  m_refq_A.data[i]+=eh_buf[i];  
	  body_A->joint(i)->q= m_refq_A.data[i];
	}
      }

      if(logFlag||forceControl)
	m_refq_AOut.write();

    }//if q_Ain.isNew()

#pragma omp section
    //if( m_mc_BIn.isNew()){
    //m_mc_BIn.read();
    if( m_q_BIn.isNew()){
      m_q_BIn.read();
      //cout<<"NewMC"<<endl;
      for(int i=0;i<dof_B;i++)
	m_refq_B.data[i]=body_B->joint(i)->q;
      
      //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
      if(!bodyDeque_B.empty()&&logFlag){
	for(int i=0;i<dof_B;i++){
	  m_refq_B.data[i]=body_B->joint(i)->q=bodyDeque_B.at(0)[i];
	  //if(forceControl)
	  //  m_refq_B.data[i]+= eh_buf[i+dof_A];
	}
	bodyDeque_B.pop_front();
	//m_refq_BOut.write();
	
	if(!waist_p_Deque_B.empty()){
	  //renew model online
	  p_now_B[WAIST]=body_B->link("WAIST")->p= waist_p_Deque_B.at(0);
	  R_now_B[WAIST]=body_B->link("WAIST")->R= waist_R_Deque_B.at(0);
	  body_B->calcForwardKinematics();
	  waist_p_Deque_B.pop_front();
	  waist_R_Deque_B.pop_front();
	  wZMP_B=calcZMP(body_B, m_rfsensor_B, m_lfsensor_B);
	  wZMPDeque_log_B.push_back(wZMP_B);
	}

      	//rzmp
	if(!rzmpDeque_B.empty()){
	  for(int i=0;i< m_rzmp_B.data.length();i++)
	    m_rzmp_B.data[i]=rzmpDeque_B.at(0)[i];  
	  rzmpDeque_B.pop_front();
	  m_rzmp_BOut.write();
	}
	/*
	  if(forceControl){
	  for(int i=0;i<dof_B;i++)
	  m_refq_B.data[i]= body_B->joint(i)->q=eh_buf[i+dof_A];
	  }*/
	
      }// if(!bodyDeque_B.empty()&&logFlag)
      //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
      
      if(forceControl){
	for(int i=0;i<dof_B;i++){
	  m_refq_B.data[i]+=eh_buf[i+dof_A];  
	  body_B->joint(i)->q= m_refq_B.data[i];
	}
      }

      if(logFlag||forceControl)
	m_refq_BOut.write();
      
    }//if q_Bin.isNew()

  }//pragma
}

inline void Juno::rzmp2st()
{
  relZMP_A = trans(R_ref_A[WAIST])*(absZMP_A - body_A->link("WAIST")->p);
  relZMP_B = trans(R_ref_B[WAIST])*(absZMP_B - body_B->link("WAIST")->p);
 
  rzmpDeque_A.push_back(relZMP_A);
  rzmpDeque_B.push_back(relZMP_B);

  /* 
   for(int i=0;i< m_rzmp_A.data.length();i++){
    m_rzmp_A.data[i]=relZMP_A[i];  
    m_rzmp_B.data[i]=relZMP_B[i]; 
  }
#pragma omp parallel sections
  {
#pragma omp section
    m_rzmp_AOut.write();
#pragma omp section
    m_rzmp_BOut.write();
  }
  */
}

inline void Juno::zmpHandler()
{
  //waiting_A
    if(stopflag_A){
      //ivk calculate
      NaturalZmp(body_A, absZMP_A);
    }
    //walking_A
    else{
      ///rzmp To st
      absZMP_A[0]=rfzmp_A.at(0)[0];
      absZMP_A[1]=rfzmp_A.at(0)[1];
      rfzmp_A.pop_front();
    }
    //waiting_B
    if(stopflag_B){
      NaturalZmp(body_B, absZMP_B);
    }
    //walking_B
    else{
      ///rzmp To st
      absZMP_B[0]=rfzmp_B.at(0)[0];
      absZMP_B[1]=rfzmp_B.at(0)[1];
      rfzmp_B.pop_front();
    }

    //for log
    absZMPDeque_log_A.push_back(absZMP_A);
    absZMPDeque_log_B.push_back(absZMP_B);
}

inline void Juno::object_operate()
{
  //by operator
  Vector3 tep;
  //translation

  if(transition_count==0)
    tep=velobj(0)*0.00005,velobj(1)*0.00005, velobj(2)*0.00005;
  /*
  //rotate
  Vector3 rpy(0.01*velobj(3)*M_PI/180, 0.01*velobj(4)*M_PI/180, 0.01*velobj(5)*M_PI/180);
  Matrix33 rotR = hrp::rotFromRpy(rpy);
  //ref////////
  object_ref->p = object_ref->p + object_ref->R*tep; 
  object_ref->R = rotR * object_ref->R;
  */

  //ref////////
  yawTotal+=0.01*velobj(5)*M_PI/180;

  if(!rpdeque.empty()) {
    Vector3 rpyTemp;
    rpyTemp=Vector3(rpdeque.at(0)[0], rpdeque.at(0)[1], yawTotal) ;
    rotRTemp = hrp::rotFromRpy(rpyTemp);
    rpdeque.pop_front();
  }
  else{
    Matrix33 rZ(rotationZ( 0.01*velobj(5)*M_PI/180));
    rotRTemp = rZ*rotRTemp ;
  }
  
  object_ref->R = rotRTemp;
  object_ref->p = object_ref->p + rotationZ(yawTotal)*tep; 

  object_p_Deque_log.push_back(object_ref->p);
}

void Juno::waitingMotion(Vector3 &cm_ref, PreviewControl *PC,bool &comInpo_flag,bool &comInpo_stop)
{

  //CoM to nutral
  if(!(PC->CoM2Stop.empty())){
    cm_ref(0)=PC->CoM2Stop.at(0)[0];
    cm_ref(1)=PC->CoM2Stop.at(0)[1];//debug 0818
    PC->CoM2Stop.pop_front();
  }
  else if(comInpo_flag){
    comInpo_flag=0;
    comInpo_stop=1;
  }
 
}

void Juno::start2walk(BodyPtr body, ZmpPlaner *zmpP, PreviewControl *PC, bool &stopflag, Vector3 cm_ref)
{// this is for FSRF or FSLF
  Vector3 rzmpInit;
  NaturalZmp(body, rzmpInit);
  zmpP->setInit( rzmpInit(0) , rzmpInit(1) );
  
  PC->setInitial(cm_ref); 
  PC->Inituk();

  stopflag=0;
}

bool Juno::ChangeSupLeg(BodyPtr body, FootType &FT, int &count, ZmpPlaner *zmpP, PreviewControl *PC, bool &stopflag, int &CommandIn, Vector3 *p_ref, Vector3 *p_Init, Matrix33 *R_ref, Matrix33 *R_Init, bool &comInpo_flag)
{
  bool ifchange=0;
 
  switch(FT){
  case FSRFsw:
    if((count== zmpP->step1Num )&&(!stopflag)){
      FT=LFsw;
      IniNewStep(body, FT, count, zmpP, PC, stopflag, CommandIn, p_ref, p_Init, R_ref, R_Init, comInpo_flag);
      ifchange=1;
      step_num++;
      //update_mc(body);
    }
    break;
    
  case FSLFsw:
    if((count== zmpP->step1Num)&&(!stopflag)){
      FT=RFsw;
      IniNewStep(body, FT, count, zmpP, PC, stopflag, CommandIn, p_ref, p_Init, R_ref, R_Init, comInpo_flag);
      ifchange=1;
      step_num++;
      //update_mc(body);
    }
    break;
    
  case LFsw:
    if(count==zmpP->NomalPaceNum){	
      FT=RFsw; 
      IniNewStep(body, FT, count, zmpP, PC, stopflag, CommandIn, p_ref, p_Init, R_ref, R_Init, comInpo_flag);
      ifchange=1;
      step_num++;
      //update_mc(body);
    }
    break;
    
  case RFsw:
    if(count==zmpP->NomalPaceNum){
      FT=LFsw;
      IniNewStep(body, FT, count, zmpP, PC, stopflag, CommandIn, p_ref, p_Init, R_ref, R_Init, comInpo_flag);
      ifchange=1;
      step_num++;
      //update_mc(body);
    }
    break;
  } 
  return ifchange;
}

void Juno::IniNewStep(BodyPtr body, FootType &FT, int &count, ZmpPlaner *zmpP, PreviewControl *PC, bool &stopflag, int &CommandIn, Vector3 *p_ref, Vector3 *p_Init, Matrix33 *R_ref, Matrix33 *R_Init,bool &comInpo_flag)
{ 
  updateInit(p_ref, p_Init, R_ref, R_Init);
  count=0;
  //ifstop
  if(CommandIn==5){
    stopflag=1;
    PC->CoMInpo(body, time2Neutral);
    comInpo_flag=1;

    if (FT==RFsw)
      FT=FSRFsw;
    else if(FT==LFsw)
      FT=FSLFsw;
  }
  zmpP->stopOper=1;
}

inline void Juno::calcRefLeg()
{
  /*
  Matrix33 Rtem_Q=extractYow(object_ref->R);
  //actually in x-y plan only
  RLEG_ref_p_A = object_ref->p + Rtem_Q * p_obj2RLEG_A; 
  LLEG_ref_p_A = object_ref->p + Rtem_Q * p_obj2LLEG_A;
  LEG_ref_R_A= Rtem_Q * R_LEG_ini_A;
  
  RLEG_ref_p_B = object_ref->p + Rtem_Q * p_obj2RLEG_B; 
  LLEG_ref_p_B = object_ref->p + Rtem_Q * p_obj2LLEG_B;
  LEG_ref_R_B= Rtem_Q * R_LEG_ini_B;
  */
  
  //object_cur ver
  
  hrp::Link* c_object;
  c_object=new Link();
  dvector a_delta_p_r(18);
  updateObject(body_A, stick_r_A, stick_l_A, body_B, stick_r_B, stick_l_B,  stick_rpy_r_A_buf, stick_rpy_l_A_buf, stick_rpy_r_B_buf, stick_rpy_l_B_buf, c_object, a_delta_p_r);
  //ofs<<object_ref->p(0)<<" "<<object_ref->p(1)<<" "<<object_ref->p(2)<<" "<<c_object->p(0)<<" "<<c_object->p(1)<<" "<<c_object->p(2)<<endl;  
  Matrix33 Rtem_Q=extractYow(c_object->R);
  
  //actually in x-y plan only (from object center)
  //RLEG_ref_p_A = c_object->p + Rtem_Q * p_obj2RLEG_A; 
  //LLEG_ref_p_A = c_object->p + Rtem_Q * p_obj2LLEG_A;
  //RLEG_ref_p_B = c_object->p + Rtem_Q * p_obj2RLEG_B; 
  //LLEG_ref_p_B = c_object->p + Rtem_Q * p_obj2LLEG_B;

  //for RH(form middle of arms)
  Vector3 mid_A((body_A->link("RARM_JOINT6")->p + body_A->link("LARM_JOINT6")->p)/2);
  Vector3 mid_B((body_B->link("RARM_JOINT5")->p + body_B->link("LARM_JOINT5")->p)/2);
  RLEG_ref_p_A = mid_A + Rtem_Q * p_obj2RLEG_A; 
  LLEG_ref_p_A = mid_A + Rtem_Q * p_obj2LLEG_A;
  RLEG_ref_p_B = mid_B + Rtem_Q * p_obj2RLEG_B; 
  LLEG_ref_p_B = mid_B + Rtem_Q * p_obj2LLEG_B;
  
  ////////////
  LEG_ref_R_A= Rtem_Q * R_LEG_ini_A;
  LEG_ref_R_B= Rtem_Q * R_LEG_ini_B;

  //for log
  crr=c_object->p;  

  //middle arm ver
  /*
  hrp::Link* c_object;
  c_object=new Link();
  dvector a_delta_p_r(18);
  updateObject(body_A, stick_r_A, stick_l_A, body_B, stick_r_B, stick_l_B, c_object, a_delta_p_r);//stick p R recalculated
  Vector3 armMid(0.25*(p_now_A[RARM] + p_now_A[LARM] + p_now_B[RARM] + p_now_B[LARM]));
  Matrix33 Rtem_Q=extractYow(c_object->R);
  armMid(2)=object_ref->p(2);
  //actually in x-y plan only
  RLEG_ref_p_A = armMid + Rtem_Q * p_obj2RLEG_A; 
  LLEG_ref_p_A = armMid + Rtem_Q * p_obj2LLEG_A;
  LEG_ref_R_A= Rtem_Q * R_LEG_ini_A;
  
  RLEG_ref_p_B = armMid + Rtem_Q * p_obj2RLEG_B; 
  LLEG_ref_p_B = armMid + Rtem_Q * p_obj2LLEG_B;
  LEG_ref_R_B= Rtem_Q * R_LEG_ini_B;
  */
}

inline void Juno::prmGenerator(bool &flagA, bool &flagB)
{
  calcRefLeg();

  if(step_num==4){//4 default
    step=0;
    //step_num=0;
    velobj= 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    //cout<<"stop"<<endl;
  }
  ofs<< comInpo_stop_A<<" "<< comInpo_stop_B<<endl;
  //if(step_num==9)
  //  playflag=0;


  //////////////usually obmit when keep walking///////////////////////////////////////////////////////////////////////
  //start to walk or not A
  //waiting
  if( stopflag_A && PC_A->CoM2Stop.empty() ){
    //normal
    //if(walkJudge(p_ref_A, R_ref_A, FT_A, RLEG_ref_p_A, LLEG_ref_p_A, LEG_ref_R_A)){
    //for stepping
    if(step){ //commet out    
      CommandIn_A=0;//start to walk
      start2walk(body_A, zmpP_A, PC_A, stopflag_A, cm_ref_A);//stopflag off
      //cerr<<"start2walk A"<<endl;
      //calc trajectory 
      prm2Planzmp(FT_A, p_ref_A, R_ref_A, RLEG_ref_p_A, LLEG_ref_p_A, LEG_ref_R_A, rfzmp_A, zmpP_A, "A");
      flagA=0;
    }
  }
  //walking&&if stop
  else if(count_A==(stepLength(FT_A,zmpP_A)- 2*zmpP_A->TdblNum)){
    if(!step){//comment out
      if(!walkJudge(p_ref_A, R_ref_A, FT_A, RLEG_ref_p_A, LLEG_ref_p_A, LEG_ref_R_A)){
	CommandIn_A=5;//stop to walk>>quick stop
	zmpP_A->StopZMP(FT_A, rfzmp_A, count_A);
	cout<<"stop_A"<<endl;
      }
    }//comment out
  }
  else if(flagA==1){//keep walking change leg
    prm2Planzmp(FT_A, p_ref_A, R_ref_A, RLEG_ref_p_A, LLEG_ref_p_A, LEG_ref_R_A, rfzmp_A, zmpP_A, "A");
    flagA=0;
  }
  /////////////////////////////////////////////////////////////////////////////////////
  //start to walk or not B
  if( stopflag_B && PC_B->CoM2Stop.empty() ){//waiting
    //normal
    //if(walkJudge(p_ref_B, R_ref_B, FT_B, RLEG_ref_p_B, LLEG_ref_p_B, LEG_ref_R_B)){
    //for stepping
    if(step){ //commet out         
      CommandIn_B=0;//start to walk
      start2walk(body_B, zmpP_B, PC_B, stopflag_B, cm_ref_B);
      //cerr<<"start2walk B"<<endl;
      //calc trajectory 
      prm2Planzmp(FT_B, p_ref_B, R_ref_B, RLEG_ref_p_B, LLEG_ref_p_B, LEG_ref_R_B, rfzmp_B, zmpP_B, "B");
      flagB=0;
    }
  }
  //walking&&if stop
  else if(count_B==(stepLength(FT_B,zmpP_B)- 2*zmpP_B->TdblNum)){
    if(!step){//comment out
      if(!walkJudge(p_ref_B, R_ref_B, FT_B, RLEG_ref_p_B, LLEG_ref_p_B, LEG_ref_R_B)){
	CommandIn_B=5;//stop to walk>>quick stop
	zmpP_B->StopZMP(FT_B, rfzmp_B, count_B);
	cout<<"stop_B"<<endl;
      }
    }//comment out
  }
  else if(flagB==1){//keep walking change leg
    prm2Planzmp(FT_B, p_ref_B, R_ref_B, RLEG_ref_p_B, LLEG_ref_p_B, LEG_ref_R_B, rfzmp_B, zmpP_B, "B");
    flagB=0;
  }
}

int Juno::stepLength(FootType FT, ZmpPlaner *zmpP)
{
  int stepLength;

 if((FT==FSRFsw)||(FT==FSLFsw))
   stepLength=zmpP->step1Num;
  else if((FT==RFsw)||(FT==LFsw))
   stepLength=zmpP->NomalPaceNum;

 return stepLength;
}

void Juno::prm2Planzmp(FootType FT, Vector3 *p_ref, Matrix33 *R_ref, Vector3 RLEG_ref_p, Vector3 LLEG_ref_p, Matrix33 LEG_ref_R, std::deque<vector2> &rfzmp, ZmpPlaner *zmpP, const char* ChIn)
{
  vector2  swLegRef_p;
  if((FT==FSRFsw)||(FT==RFsw)){
    swLegRef_p = pfromVector3(RLEG_ref_p) ;
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    swLegRef_p = pfromVector3(LLEG_ref_p) ;
  }
  
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
  //limit
  int SupLeg;
  Vector3  SwLeg_p_ref;
  double limit_y;
  //RLEG_ref_R= LLEG_ref_R=obj
  if((FT==FSRFsw)||FT==RFsw){
    SupLeg=LLEG;
    SwLeg_p_ref=RLEG_ref_p;
    limit_y=-0.17;
  }
  else if((FT==FSLFsw)||FT==LFsw){
    SupLeg=RLEG;
    SwLeg_p_ref=LLEG_ref_p;
    limit_y=0.17;
  }
  
  Vector3 Shift2Zero(trans(R_ref[SupLeg])*( SwLeg_p_ref - p_ref[SupLeg]));
  if(fabs(Shift2Zero(1))<0.17)
    {
      Shift2Zero(1)=limit_y;
      SwLeg_p_ref= p_ref[SupLeg] + R_ref[SupLeg] * Shift2Zero;
      //adjust
      swLegRef_p= pfromVector3( SwLeg_p_ref);
      //cerr<<"interference"<<endl;
    }
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
  
  rfzmp.clear();
  zmpP->PlanZMPnew(FT, p_ref, R_ref, swLegRef_p, LEG_ref_R, rfzmp);///plan rzmp&swingLeg traje
  
}

void Juno::walkingMotion(BodyPtr body, FootType FT, Vector3 &cm_ref, Vector3 &absZMP, Vector3 *p_Init, Vector3 *p_ref, Matrix33 *R_ref, std::deque<vector2> &rfzmp, PreviewControl *PC, ZmpPlaner *zmpP, int &count)
{
  //preview control
  PC->calcInput(rfzmp);//input recalculated
  PC->calcNextState();
  cm_ref(0)=PC->cur_state(0,0);
  cm_ref(1)=PC->cur_state(0,1);
  PC->update();
  
  ///rzmp To st
  absZMP[0]=rfzmp.at(0)[0];
  absZMP[1]=rfzmp.at(0)[1];

  //swingLeg
  int swingLeg=swLeg(FT);
  p_ref[swingLeg](0)=zmpP->swLegxy.at(count)[0];
  p_ref[swingLeg](1)=zmpP->swLegxy.at(count)[1];
  p_ref[swingLeg](2)=p_Init[swingLeg](2)+zmpP->Trajzd.at(count);
  R_ref[swingLeg]= zmpP->swLeg_R.at(count);
  zmpP->calcWaistR(FT, p_ref, R_ref); 
 
  count++;
}

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
   
//method
//for clip
void Juno::testMove()
{

  int yaw=2;

  write_flag=0;

  while(transition_count<0)
    calcWholeIVK();
  
  for(int i=0;i<240;i++){
    Matrix33 rZ(rotationZ( 0.01*yaw*M_PI/180));
    object_ref->R=rotRTemp = rZ*rotRTemp ;
    calcWholeIVK();
  }
  for(int i=0;i<480;i++){
    Matrix33 rZ(rotationZ( 0.01*-yaw*M_PI/180));
    object_ref->R=rotRTemp = rZ*rotRTemp ;
    calcWholeIVK();
  }
  for(int i=0;i<240;i++){
    Matrix33 rZ(rotationZ( 0.01*yaw*M_PI/180));
    object_ref->R=rotRTemp = rZ*rotRTemp ;
    calcWholeIVK();
  }
  cerr<<"test move"<<endl;
  /*
  vector2 clip_cur_A(  body_A->joint(23)->q,  body_A->joint(31)->q);
  vector2 clip_cur_B(  body_B->joint(22)->q,  body_B->joint(29)->q);
  //vector2 clip_ref(15*M_PI/180 , -15*M_PI/180);
  //vector2 clip_ref(-0.01 , 0.01);
  //vector2 clip_ref(0.03 , -0.03);//walk simulation
  vector2 clip_ref(-0.017 , 0.017);
  vector2 clip_ref_B(0.006 , -0.006);
  vector2 zero(0.0,0.0);
  Interplation5(clip_cur_A,  zero,  zero, clip_ref,  zero,  zero, 1, clipDeque_A);
  Interplation5(clip_cur_B,  zero,  zero, clip_ref_B,  zero,  zero, 1, clipDeque_B);  
  */

  /*
  cerr<<body_B->sensor<hrp::ForceSensor>(2)->localR<<endl;
  cerr<<body_B->link("RARM_JOINT5")->R<<endl;
  cerr<<body_B->sensor<hrp::ForceSensor>(3)->localR<<endl;
  cerr<<body_B->link("LARM_JOINT5")->R<<endl;
  */

  /*
  zmpHandler();
  absZMP_A[1]+=0.05; 
  rzmp2st();
  */

  //adjust initial pos
  //A
  /*
  Matrix33 r,l;
  r=0.0, 0.0, -1.0,
    1.0, 0.0, 0.0,
    0.0, -1.0, 0.0;
  l=0.0, 0.0, -1.0,
    -1.0, 0.0, 0.0,
    0.0, 1.0, 0.0; 
  R_ref_A[RARM]=r;
  R_ref_A[LARM]=l;  
  cm_ref_A(0)=(body_A->link("RLEG_JOINT5")->p(0)+body_A->link("LLEG_JOINT5")->p(0))/2;
  cm_ref_A(1)=(body_A->link("RLEG_JOINT5")->p(1)+body_A->link("LLEG_JOINT5")->p(1))/2;
  p_ref_A[RARM](0)+=0.05;
  p_ref_A[LARM](0)+=0.05; 
 
  if(CalcIVK_ARM_NUM(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A)){
    for(int i=0;i<body_A->numJoints();i++){
      body_ref_A[i]=body_A->joint(i)->q;   
      ofs<<rad2deg( body_ref_A[i])<<",";
    }
    ofs<<endl;
    vector32 zero_A(0);  
    Interplation5(body_cur_A,  zero_A,  zero_A, body_ref_A,  zero_A,  zero_A, 1, bodyDeque_A);
    body_cur_A=body_ref_A;
  }
  else
    cerr<<"NO"<<endl;
  */
  
  /*
  cerr<< elbow_A<<endl;
  elbow_A(0)=2.9;
  elbow_A(1)=-2.9;
  if(CalcIVK_ARM_NUM_elbow(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, elbow_A)){
    for(int i=0;i<body_A->numJoints();i++){
      body_ref_A[i]=body_A->joint(i)->q;   []
    }
    vector32 zero_A(0);  
    Interplation5(body_cur_A,  zero_A,  zero_A, body_ref_A,  zero_A,  zero_A, 1, bodyDeque_A);
    body_cur_A=body_ref_A;
  }
  else
    cerr<<"NO"<<endl;
  */

  /*
  elbow_A(0)=2.9;
  if(IVK_elbow(body_A,  p_ref_A, R_ref_A, elbow_A)){
    for(int i=0;i<body_A->numJoints();i++){
      body_ref_A[i]=body_A->joint(i)->q;   
    }
    vector32 zero_A(0);  
    Interplation5(body_cur_A,  zero_A,  zero_A, body_ref_A,  zero_A,  zero_A, 1, bodyDeque_A);
    body_cur_A=body_ref_A;
  }
  else
    cerr<<"NO"<<endl;
  */


  /*
  //B
  Matrix33 r,l;
  r=0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0;
  l=0.0, 0.0, 1.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0; 
  R_ref_B[RARM]=r;
  R_ref_B[LARM]=l;  
  cm_ref_B(0)=(body_B->link("RLEG_JOINT5")->p(0)+body_B->link("LLEG_JOINT5")->p(0))/2;
  cm_ref_B(1)=(body_B->link("RLEG_JOINT5")->p(1)+body_B->link("LLEG_JOINT5")->p(1))/2;
  p_ref_B[RARM](0)-=0.05;
  p_ref_B[LARM](0)-=0.05; 
 
  if(CalcIVK_ARM_NUM(body_B, cm_ref_B, p_ref_B, R_ref_B, FT_B, p_Init_B, R_Init_B)){
    for(int i=0;i<body_B->numJoints();i++){
      body_ref_B[i]=body_B->joint(i)->q;   
      ofs<<rad2deg( body_ref_B[i])<<",";
    }
    ofs<<endl;
    vector30 zero_B(0);  
    Interplation5(body_cur_B,  zero_B,  zero_B, body_ref_B,  zero_B,  zero_B, 1, bodyDeque_B);
    body_cur_B=body_ref_B;
  }
  else
    cerr<<"NO"<<endl;
  */
}

void Juno::setObjectV(double x, double y, double z, double roll, double pitch, double yaw)
{   
  //always use below so long
   velobj= x,y,z,roll,pitch,yaw;

  if(roll!=rpcrr(0)||pitch!=rpcrr(1)){
    vector2 zero(0.0,0.0);
    vector2 rp_ref(roll*M_PI/180,pitch*M_PI/180);
    Interplation5(rpcrr,  zero,  zero, rp_ref,  zero,  zero, 1, rpdeque);
    rpcrr=rp_ref;
  }
  stepping();
  /*
 //for wzmp //no need
  body_A->link("WAIST")->R= waist_R_ini_A;
  body_A->link("WAIST")->p= waist_p_ini_A;
  body_B->link("WAIST")->R= waist_R_ini_B;
  body_B->link("WAIST")->p= waist_p_ini_B;
  for(unsigned int i=0;i<m_mc_A.data.length();i++)
    body_A->joint(i)->q=body_ini_A(i);
  for(unsigned int i=0;i<m_mc_B.data.length();i++)
    body_B->joint(i)->q=body_ini_B(i);
  body_A->calcForwardKinematics();
  body_B->calcForwardKinematics();  
  RenewModel(body_A, p_Init_A, R_Init_A);
  RenewModel(body_B, p_Init_B, R_Init_B);
  //body_A->link("WAIST")->R= waist_R_ini_A;
  //body_B->link("WAIST")->R= waist_R_ini_B;
  */

  /*
  write_flag=0;
  for(int i=0;i<240;i++){
    Matrix33 rZ(rotationZ( 0.01*yaw*M_PI/180));
    object_ref->R=rotRTemp = rZ*rotRTemp ;
    calcWholeIVK();
  }
  for(int i=0;i<480;i++){
    Matrix33 rZ(rotationZ( 0.01*-yaw*M_PI/180));
    object_ref->R=rotRTemp = rZ*rotRTemp ;
    calcWholeIVK();
  }
  for(int i=0;i<240;i++){
    Matrix33 rZ(rotationZ( 0.01*yaw*M_PI/180));
    object_ref->R=rotRTemp = rZ*rotRTemp ;
    calcWholeIVK();
  }
  */
  cout<<"calculate OK"<<endl;
  
  //////////////////////
  /*
  //V in
  object_ref->p(0)+=x;
  object_ref->p(1)+=y;
  object_ref->p(2)+=z;
  vector3 rpy(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180);
  object_ref->R = hrp::rotFromRpy(rpy);
  
  if(CalcIVK_NMSG(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, stick_r_A, stick_l_A, 
	 body_B, cm_ref_B, p_ref_B, R_ref_B, FT_B, p_Init_B, R_Init_B, stick_r_B, stick_l_B, 
	 stick_rpy_r_A_buf, stick_rpy_l_A_buf, stick_rpy_r_B_buf, stick_rpy_l_B_buf, object_ref )){
 
  cerr<<"OKOK!"<<endl;
 
  for(int i=0;i<body_A->numJoints();i++)
      body_ref_A[i]=body_A->joint(i)->q;
    for(int i=0;i<body_B->numJoints();i++)
      body_ref_B[i]=body_B->joint(i)->q;   
  
  vector32 zero_A(0);
  vector30 zero_B(0);
  Interplation5(body_cur_A,  zero_A,  zero_A, body_ref_A,  zero_A,  zero_A, 1, bodyDeque_A);
  Interplation5(body_cur_B,  zero_B,  zero_B, body_ref_B,  zero_B,  zero_B, 1, bodyDeque_B);
  
  body_cur_A=body_ref_A;
  body_cur_B=body_ref_B;
  

 }
 else
   cerr<<"NO!"<<endl;
  */

}

void Juno::move_hand(double x, double y, double z, double roll, double pitch, double yaw)
{
  cout<<"move hand"<<endl;
  /*
  p_ref_A[RARM](0)+=0.01*x; 
  p_ref_A[LARM](0)+=0.01*x; 

  if(CalcIVK_ARM_NUM_elbow(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, elbow_A,0)){
    for(int i=0;i<body_A->numJoints();i++){
      body_ref_A[i]=body_A->joint(i)->q;
    }
    vector32 zero_A(0);  
    Interplation5(body_cur_A,  zero_A,  zero_A, body_ref_A,  zero_A,  zero_A, 1, bodyDeque_A);
    body_cur_A=body_ref_A;
  }
  else
    cerr<<"NO"<<endl;
  */
  write_flag=logFlag=0;

  object_ref_single->p(0)+=0.01*x;

  Matrix33 R_before(object_ref_single->R);
  Matrix33 Rt(rotationZ(deg2rad(yaw))* object_ref_single->R);
  object_ref_single->R=Rt;
  
  /*
  m_mc_AIn.read();
  for(unsigned int i=0;i<m_mc_A.data.length();i++)
    body_cur_A[i]=m_mc_A.data[i];
  */
  
  if(CalcIVK_NMS_single(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, stick_r_single, stick_l_single, object_ref_single, elbow_A)){
    for(int i=0;i<body_A->numJoints();i++){
      body_ref_A[i]=body_A->joint(i)->q;
    }
    vector32 zero_A(0);  
    Interplation5(body_cur_A,  zero_A,  zero_A, body_ref_A,  zero_A,  zero_A, 1, bodyDeque_A);
    body_cur_A=body_ref_A;
    write_flag=1;
    logFlag=1;
  }
  else{
    cerr<<"NO"<<endl;
    object_ref_single->p(0)-=0.01*x;
    object_ref_single->R=R_before;
  }


  
}


void Juno::fcontrol()
{
  forceControl=!forceControl;
  write_flag=1;
  cout<<"fcontrol is "<<forceControl<<endl;
}

void Juno::exPos()
{
  //A 
  m_mc_AIn.read();
  for(unsigned int i=0;i<m_mc_A.data.length();i++)
    body_cur_A[i]=m_mc_A.data[i];
  
  //B
  m_mc_BIn.read();
  for(unsigned int i=0;i<m_mc_B.data.length();i++)
    body_cur_B[i]=m_mc_B.data[i];
 
  vector32 zero_A(0);
  vector30 zero_B(0);
  body_ref_A=0, 0.00332796, -0.482666, 0.859412, -0.370882, -0.00322683,  0, 0.00332796, -0.482666, 0.859412, -0.370882, -0.00322683,  0,  0,  0,  0, 0.135465, -0.290561, 0.14261, -1.81385, 1.30413, 0.0651451, 0.202547,  0, 0.135465, 0.290561, -0.14261, -1.81385, -1.30413, -0.0651451, 0.202547,  0;

  body_ref_B= 0, 0.00333798, -0.493139, 0.871946, -0.378195, -0.00329024,  0, 0.00333798, -0.493139, 0.871946, -0.378195, -0.00329024,  0,  0,  0,  0, 0.085065, -0.485823, 0.315933, -1.82036, 1.14179, 0.391601,  0, 0.085065, 0.485823, -0.315933, -1.82036, -1.14179, 0.391601,  0;

  Interplation5(body_cur_A,  zero_A,  zero_A, body_ref_A,  zero_A,  zero_A, 3, bodyDeque_A);
  Interplation5(body_cur_B,  zero_B,  zero_B, body_ref_B,  zero_B,  zero_B, 3, bodyDeque_B);
  
  body_cur_A=body_ref_A;
  body_cur_B=body_ref_B;

  write_flag=1;
  logFlag=1;
  
  cerr<<"my expos"<<endl;
  
}
void Juno::hogex()
{
  //for log
  NMS->logStart();
  
  //write_flag=1;//play start
  //playflag=0;//stop calculate
  //logFlag=1;

  write_flag=0;
  forceControl=1;//pool
  cout<<"play start"<<endl;
}

void Juno::save_log()
{
  //stop log
  logFlag=0;
  NMS->logStart();//stop when start twice
  int timmer=0;

  std::ofstream ofs_hctem("/home/grxuser/users/wu/junoRH/hcTem.log");
  while(!NMS->hcTem_deque.empty()){
    for(int i=6;i<24;i++)
      ofs_hctem<<NMS->hcTem_deque.at(0)[i]<<" ";
    ofs_hctem<<timmer*0.005<<" "<<endl;
    timmer++;
    NMS->hcTem_deque.pop_front();
  }
  timmer=0;

  std::ofstream ofs_cm("/home/grxuser/users/wu/junoRH/cm.log");
  while(!cmDeque_log_A.empty()){
    for(int i=0;i<2;i++)
      ofs_cm<<cmDeque_log_A.at(0)[i]<<" ";
    for(int i=0;i<2;i++)
      ofs_cm<<cmDeque_log_B.at(0)[i]<<" ";
    
    for(int i=0;i<2;i++)
      ofs_cm<<absZMPDeque_log_A.at(0)[i]<<" ";
    for(int i=0;i<2;i++)
      ofs_cm<<absZMPDeque_log_B.at(0)[i]<<" ";
    
    for(int i=0;i<2;i++)
      ofs_cm<<object_p_Deque_log.at(0)[i]<<" ";
   
    for(int i=0;i<2;i++)
      ofs_cm<<wZMPDeque_log_A.at(0)[i]<<" ";
    for(int i=0;i<2;i++)
      ofs_cm<<wZMPDeque_log_B.at(0)[i]<<" ";
    
    ofs_cm<<timmer*0.005<<" "<<endl;
    timmer++;
    cmDeque_log_A.pop_front();
    cmDeque_log_B.pop_front();
    absZMPDeque_log_A.pop_front();
    absZMPDeque_log_B.pop_front();
    object_p_Deque_log.pop_front();
    wZMPDeque_log_A.pop_front();
    wZMPDeque_log_B.pop_front();
    
  }
  timmer=0;
  /*
    std::ofstream ofs_hsensor_A("/home/grxuser/users/wu/junoRH/hsensor_A.log");
    std::ofstream ofs_hsensor_B("/home/grxuser/users/wu/junoRH/hsensor_B.log");
  while(!rhsensor_deque_B.empty()){
    for(int i=0;i<6;i++)
      ofs_hsensor_B<<rhsensor_deque_B.at(0).data[i]<<" ";
    
    for(int i=0;i<6;i++)
      ofs_hsensor_B<<lhsensor_deque_B.at(0).data[i]<<" ";
    
    ofs_hsensor_B<<endl;
    rhsensor_deque_B.pop_front();
    lhsensor_deque_B.pop_front();
  }
  */

  /*  
  Vector3 tem_r_f(0),tem_r_tau(0),r_f(0),r_tau(0);
  Vector3 tem_l_f(0),tem_l_tau(0),l_f(0),l_tau(0);
  
  while(!rhsensor_deque_A.empty()){
    for(int i = 0; i < 3; i++) {	
      tem_r_f(i)   = rhsensor_deque_A.at(0).data[i];
      tem_r_tau(i) = rhsensor_deque_A.at(0).data[i+3];
      tem_l_f(i)   = lhsensor_deque_A.at(0).data[i];
      tem_l_tau(i) = lhsensor_deque_A.at(0).data[i+3];
    }
    rhsensor_deque_A.pop_front();
    lhsensor_deque_A.pop_front();

    r_f= body_A->link("RARM_JOINT6")->R* tem_r_f;
    r_tau= body_A->link("RARM_JOINT6")->R*  tem_r_tau;
    l_f= body_A->link("LARM_JOINT6")->R*  tem_l_f;
    l_tau= body_A->link("LARM_JOINT6")->R*  tem_l_tau;

    for(int i=0;i<3;i++)
      ofs_hsensor_A<<r_f(i)<<" ";
    for(int i=0;i<3;i++)
      ofs_hsensor_A<<r_tau(i)<<" ";
    for(int i=0;i<3;i++)
      ofs_hsensor_A<<l_f(i)<<" ";
    for(int i=0;i<3;i++)
      ofs_hsensor_A<<l_tau(i)<<" ";

    ofs_hsensor_A<<endl;
  }

  Matrix33 Rs_RARM(rotationZ(M_PI/2));
  Matrix33 Rs_LARM(rotationZ(-M_PI/2));

  while(!rhsensor_deque_B.empty()){
    for(int i = 0; i < 3; i++) {	
      tem_r_f(i)   = rhsensor_deque_B.at(0).data[i];
      tem_r_tau(i) = rhsensor_deque_B.at(0).data[i+3];
      tem_l_f(i)   = lhsensor_deque_B.at(0).data[i];
      tem_l_tau(i) = lhsensor_deque_B.at(0).data[i+3];
    }
    rhsensor_deque_B.pop_front();
    lhsensor_deque_B.pop_front();

    r_f= body_B->link("RARM_JOINT5")->R*Rs_RARM * tem_r_f;
    r_tau= body_B->link("RARM_JOINT5")->R*Rs_RARM *  tem_r_tau;
    l_f= body_B->link("LARM_JOINT5")->R*Rs_LARM *  tem_l_f;
    l_tau= body_B->link("LARM_JOINT5")->R*Rs_LARM *  tem_l_tau;

    for(int i=0;i<3;i++)
      ofs_hsensor_B<<r_f(i)<<" ";
    for(int i=0;i<3;i++)
      ofs_hsensor_B<<r_tau(i)<<" ";
    for(int i=0;i<3;i++)
      ofs_hsensor_B<<l_f(i)<<" ";
    for(int i=0;i<3;i++)
      ofs_hsensor_B<<l_tau(i)<<" ";

    ofs_hsensor_B<<endl;
  }
  */

  /*
  std::ofstream ofs_qRef_A("/home/grxuser/users/wu/junoRH/qRef_A.log");
  std::ofstream ofs_qRef_B("/home/grxuser/users/wu/junoRH/qRef_B.log");
 
  while(!bodyDeque_log_A.empty()){
    for(int i = 0; i < 32; i++)
      ofs_qRef_A<<bodyDeque_log_A.at(0)[i]<<" ";
    ofs_qRef_A<<endl;

    bodyDeque_log_A.pop_front();
  }
  
  while(!bodyDeque_log_B.empty()){
    for(int i = 0; i < 30; i++)
      ofs_qRef_B<<bodyDeque_log_B.at(0)[i]<<" ";
    ofs_qRef_B<<endl;

    bodyDeque_log_B.pop_front();
  }
  */


 cout<<"log saved"<<endl;
}

void Juno::clip_open_A()
{
  write_flag=logFlag=0;
  m_mc_AIn.read();
  for(unsigned int i=0;i<m_mc_A.data.length();i++)
    body_cur_A[i]=body_A->joint(i)->q;//m_mc_A.data[i];
  
  vector32 zero_A(0);
 
  body_ref_A= body_cur_A;
  body_ref_A[23]=deg2rad(48);
  body_ref_A[31]=deg2rad(-48);

  Interplation5(body_cur_A,  zero_A,  zero_A, body_ref_A,  zero_A,  zero_A, 2, bodyDeque_A);
  body_cur_A=body_ref_A;

  write_flag=1;
  logFlag=1;
  cout<<"open A"<<endl;
}
void Juno::clip_open_B()
{
  write_flag=logFlag=0;
  m_mc_BIn.read();
  for(unsigned int i=0;i<m_mc_B.data.length();i++)
    body_cur_B[i]=body_B->joint(i)->q;//m_mc_B.data[i];
  
  vector30 zero_B(0);
 
  body_ref_B= body_cur_B;
  body_ref_B[22]=deg2rad(-46);
  body_ref_B[29]=deg2rad(-46);

  Interplation5(body_cur_B,  zero_B,  zero_B, body_ref_B,  zero_B,  zero_B, 2, bodyDeque_B);
  body_cur_B=body_ref_B;

  write_flag=1;
  logFlag=1;
  cout<<"open B"<<endl;
}
void Juno::clip_close_A()
{
  write_flag=logFlag=0;
  m_mc_AIn.read();
  for(unsigned int i=0;i<m_mc_A.data.length();i++)
    body_cur_A[i]=body_A->joint(i)->q;//m_mc_A.data[i];
  
  vector32 zero_A(0);
  
  body_ref_A= body_cur_A;
  body_ref_A[23]=deg2rad(2);
  body_ref_A[31]=deg2rad(-2);
  
  Interplation5(body_cur_A,  zero_A,  zero_A, body_ref_A,  zero_A,  zero_A, 2, bodyDeque_A);
  body_cur_A=body_ref_A;

  write_flag=1;
  logFlag=1;
  cout<<"close A"<<endl;
}
void Juno::clip_close_B()
{
  write_flag=logFlag=0;
  m_mc_BIn.read();
  for(unsigned int i=0;i<m_mc_B.data.length();i++)
    body_cur_B[i]=body_B->joint(i)->q;//m_mc_B.data[i];
  
  vector30 zero_B(0);
 
  body_ref_B= body_cur_B;
  body_ref_B[22]=deg2rad(-19.8);
  body_ref_B[29]=deg2rad(-19.8);

  Interplation5(body_cur_B,  zero_B,  zero_B, body_ref_B,  zero_B,  zero_B, 2, bodyDeque_B);
  body_cur_B=body_ref_B;

  write_flag=1;
  logFlag=1;
  cout<<"close B"<<endl;
}
void Juno::stepping()
{
  step=!step;
  logFlag=0;
  forceControl=0;
  write_flag=0;//force control still work
  playflag=1;

  while(playflag)
    main_algorithm();
  //cerr<<"stepping"<<endl;
}

void Juno::setMaker(double RARM_x_A , double RARM_y_A, double LARM_x_A, double LARM_y_A,
	    double RARM_x_B , double RARM_y_B, double LARM_x_B, double LARM_y_B)
{
  cout<<"set maker"<<endl;
  cout<<RARM_x_A<<" "<<RARM_y_A<<" "<< LARM_x_A<<" "<< LARM_y_A<<endl;
  cout<<RARM_x_B<<" "<<RARM_y_B<<" "<< LARM_x_B<<" "<< LARM_y_B<<endl;
  maker_r_A(0)= RARM_x_A ;maker_r_A(1)= RARM_y_A ;maker_r_A(2)=0.0;
  maker_l_A(0)= LARM_x_A ;maker_l_A(1)= LARM_y_A ;maker_l_A(2)=0.0;
  maker_r_B(0)= RARM_x_B ;maker_r_B(1)= RARM_y_B ;maker_r_B(2)=0.0;
  maker_l_B(0)= LARM_x_B ;maker_l_B(1)= LARM_y_B ;maker_l_B(2)=0.0;

  optcap=1;
}

void Juno::preSet()//in air
{
  cout<<"preset"<<endl;
  //for expos
  update_mc(body_A);
  update_mc(body_B);
  RenewModel(body_A, p_ref_A, R_ref_A);
  RenewModel(body_B, p_ref_B, R_ref_B);
  R_ref_A[WAIST]=tvmet::identity<hrp::Matrix33>();
  R_ref_B[WAIST]=rotationZ(M_PI);
  
  cm_ref_A=body_A->calcCM();// 
  cm_ref_B=body_B->calcCM();
  getElbowAngleARM(body_A, elbow_A);
  getElbowAngleARM(body_B, elbow_B);

  //for expos adjust
  hrp::Vector3 offset_r(0.0, 0.0, -0.155);
  hrp::Vector3 offset_l(0.0, 0.0, -0.155);
  hrp::Vector3 grasp_point_r( p_ref_A[RARM] +  R_ref_A[RARM] * offset_r);
  hrp::Vector3 grasp_point_l( p_ref_A[LARM] +  R_ref_A[LARM] * offset_l);

  object_ref_single->p = 0.5 * ( grasp_point_r + grasp_point_l );
  object_ref_single->R = hrp::rotFromRpy(0.0, 0.0, 0.0);

  stick_r_single->b  = tvmet::trans(R_ref_A[RARM]) * ( object_ref_single->p -  p_ref_A[RARM]);
  stick_r_single->Rs = tvmet::trans(R_ref_A[RARM]) * object_ref_single->R;
  stick_l_single->b  = tvmet::trans(R_ref_A[LARM]) * ( object_ref_single->p -  p_ref_A[LARM]);
  stick_l_single->Rs = tvmet::trans(R_ref_A[LARM]) * object_ref_single->R;

  //for walk planing
  Vector3 mid_arm_A((p_ref_A[RARM]+p_ref_A[LARM])/2);
  Vector3 mid_arm_B((p_ref_B[RARM]+p_ref_B[LARM])/2);
  p_obj2RLEG_A = p_Init_A[RLEG] - mid_arm_A; 
  p_obj2LLEG_A = p_Init_A[LLEG] - mid_arm_A; 
  p_obj2RLEG_B = p_Init_B[RLEG] - mid_arm_B; 
  p_obj2LLEG_B = p_Init_B[LLEG] - mid_arm_B; 
}

void Juno::start()
{
  
  //////////
  //for test in servo off, comment out this when experiment
  if(!RH){
    for(unsigned int i=0;i<m_mc_A.data.length();i++)
      m_mc_A.data[i]=body_cur_A(i);
    for(unsigned int i=0;i<m_mc_B.data.length();i++)
      m_mc_B.data[i]=body_cur_B(i);
  }
  ///////
  
  //for expos//already done in preSet
  //update_mc(body_A);
  //update_mc(body_B);
  //RenewModel(body_A, p_ref_A, R_ref_A);
  //RenewModel(body_B, p_ref_B, R_ref_B);
  //R_ref_A[WAIST]=tvmet::identity<hrp::Matrix33>();//already done in preSet
  //R_ref_B[WAIST]=rotationZ(M_PI);//caution!!!!!!!!!!
  
  //for log
  //for(unsigned int i=0;i<m_mc_A.data.length();i++)
  //  body_ini_A(i)=m_mc_A.data[i];
  //for(unsigned int i=0;i<m_mc_B.data.length();i++)
  //  body_ini_B(i)=m_mc_B.data[i];

  ///////optcap////
  if(optcap){
    body_A->calcForwardKinematics();//already body_A->joint(i)->q=bodyDeque_A.at(0)[i];
    body_B->calcForwardKinematics();

    Vector3 mid_A((body_A->link("RARM_JOINT6")->p + body_A->link("LARM_JOINT6")->p)/2);
    Vector3 mid_B((body_B->link("RARM_JOINT5")->p + body_B->link("LARM_JOINT5")->p)/2);
  
    Vector3 middleHand_2waist_A(body_A->link("WAIST")->p- mid_A);
    Vector3 middleHand_2waist_B(body_B->link("WAIST")->p- mid_B);
    alias(middleHand_2waist_B)=prod (trans(rotationZ(M_PI)),middleHand_2waist_B);

    Vector3 middleHand_A((maker_r_A + maker_l_A) /2);
    Vector3 middleHand_B((maker_r_B + maker_l_B) /2);
    middleHand_A(2)=mid_A(2);
    middleHand_B(2)=mid_B(2);

    cout<<middleHand_A<<endl;
    cout<<middleHand_B<<endl;

    Vector3 a;
    a= maker_l_A - maker_r_A;
    double theta_A= M_PI/2 - atan2(a(1),a(0));
    Matrix33 R_A(rotationZ(-theta_A));

    a=maker_l_B - maker_r_B;
    double theta_B=  M_PI/2 -atan2(a(1),a(0));
    Matrix33 R_B(rotationZ(-theta_B));

    yawTotal=-theta_A;

    cout<<"Theta_A "<<rad2deg(-theta_A)<<endl;  
    cout<<"Theta_B "<<rad2deg(-theta_B)<<endl;  
    //main
    body_A->link("WAIST")->R=R_A;
    body_B->link("WAIST")->R=R_B;
    body_A->link("WAIST")->p= middleHand_A+ R_A*middleHand_2waist_A;
    body_B->link("WAIST")->p= middleHand_B+ R_B*middleHand_2waist_B;
    body_A->calcForwardKinematics();
    body_B->calcForwardKinematics();

    RenewModel(body_A, p_ref_A, R_ref_A);
    RenewModel(body_B, p_ref_B, R_ref_B);
    R_ref_A[WAIST]=R_A;//inportant
    R_ref_B[WAIST]=R_B;

    updateInit(p_ref_A, p_Init_A, R_ref_A, R_Init_A);
    updateInit(p_ref_B, p_Init_B, R_ref_B, R_Init_B);

    R_LEG_ini_A=  LEG_ref_R_A= R_A;
    R_LEG_ini_B=  LEG_ref_R_B= R_B;
    Vector3 Am((body_A->link("RARM_JOINT6")->p + body_A->link("LARM_JOINT6")->p)/2);
    Vector3 Bm((body_B->link("RARM_JOINT5")->p + body_B->link("LARM_JOINT5")->p)/2);
    //cout<<Am<<endl;
    //cout<<Bm<<endl;

    cout<<body_A->link("RARM_JOINT6")->p<<endl;
    cout<<body_A->link("LARM_JOINT6")->p<<endl;
    cout<<body_B->link("RARM_JOINT5")->p<<endl;
    cout<<body_B->link("LARM_JOINT5")->p<<endl;

    Vector3 ALm((body_A->link("RLEG_JOINT5")->p + body_A->link("LLEG_JOINT5")->p)/2);
    Vector3 BLm((body_B->link("RLEG_JOINT5")->p + body_B->link("LLEG_JOINT5")->p)/2);
    absZMP_A=ALm;
    absZMP_B=BLm;
    //relZMP_A=relZMP_B=BLm;

    absZMP_A(2)=0.0;
    absZMP_B(2)=0.0;
    //NMS
    stick_rpy_r_A_buf=stick_rpy_l_A_buf=Vector3(0.0, 0.0, -theta_A);
    stick_rpy_r_B_buf=stick_rpy_l_B_buf=Vector3(0.0, 0.0, -theta_A);
  }
  ///////////optcap over////


  for(unsigned int i=0;i<dof_A;i++)
    body_cur_A[i]=qRef_tem_A(i)=body_A->joint(i)->q; 
  for(unsigned int i=0;i<dof_B;i++)
    body_cur_B[i]=qRef_tem_B(i)=body_B->joint(i)->q; 
  
  cm_ref_A=body_A->calcCM();// 
  cm_ref_B=body_B->calcCM();
  getElbowAngleARM(body_A, elbow_A);
  getElbowAngleARM(body_B, elbow_B);

 
  //tvmet::identity<hrp::Matrix33>();
 
  //renew body in NMS
  NMS->reNewModel(body_A, m_mc_A, FT_A, p_Init_A, R_Init_A, body_B, m_mc_B, FT_B, p_Init_B, R_Init_B);
  NMS->start();
  NMS->getStick(stick_r_A, stick_l_A,stick_r_B, stick_l_B);
  NMS->getObject(object_ref);
  //class ini
  zmpP_A= new ZmpPlaner;
  PC_A= new PreviewControl(0.005, cm_ref_A(2), 9.8);
  if(!PC_A->loadGain( kgain,  fgain))
    std::cerr<<"GainLoadErr"<<std::endl; 
  zmpP_B= new ZmpPlaner;
  PC_B= new PreviewControl(0.005, cm_ref_B(2), 9.8);
  if(!PC_B->loadGain( kgain,  fgain))
    std::cerr<<"GainLoadErr"<<std::endl; 

  //for path planning/////////////////////////////////////////
  //ini do this at last 
  //p_obj2RLEG_A = p_Init_A[RLEG] - object_ref->p; 
  //p_obj2LLEG_A = p_Init_A[LLEG] - object_ref->p; 
  //p_obj2RLEG_B = p_Init_B[RLEG] - object_ref->p; 
  //p_obj2LLEG_B = p_Init_B[LLEG] - object_ref->p; 
  

  //normal
  if(!optcap){
    R_LEG_ini_A=  LEG_ref_R_A= tvmet::identity<matrix33>();
    R_LEG_ini_B=  LEG_ref_R_B= rotationZ(M_PI);//180 rotate
  }
  //////////////////////////////////////////////////////////////

  rotRTemp=object_ref->R;
  //cerr<<"startQ"<<endl;

  zmpHandler();
  //rzmp2st();/////////////////////////////
  relZMP_A = trans(R_ref_A[WAIST])*(absZMP_A - body_A->link("WAIST")->p);
  relZMP_B = trans(R_ref_B[WAIST])*(absZMP_B - body_B->link("WAIST")->p);
  for(int i=0;i< m_rzmp_A.data.length();i++){
    m_rzmp_A.data[i]=relZMP_A[i];  
    m_rzmp_B.data[i]=relZMP_B[i]; 
  }
#pragma omp parallel sections
  {
#pragma omp section
    m_rzmp_AOut.write();
#pragma omp section
    m_rzmp_BOut.write();
  }
  ///////////////////////////////////////////
  //for log///
  waist_R_ini_A=R_ref_A[WAIST];
  waist_p_ini_A=body_A->link("WAIST")->p;
  waist_R_ini_B=R_ref_B[WAIST];
  waist_p_ini_B=body_B->link("WAIST")->p;
  ///
  p_now_A[WAIST]=p_ref_A[WAIST]=body_A->link("WAIST")->p;
  p_now_B[WAIST]=p_ref_B[WAIST]=body_B->link("WAIST")->p;
  R_now_A[WAIST]=R_ref_A[WAIST];
  R_now_B[WAIST]=R_ref_B[WAIST];

  //onoff 
  //playflag=1;//comment out in RH experiment for off line recording
}




void Juno::stop()
{
  write_flag=0;
}

void Juno::setWalkCommand(const char* ChIn)
{ 
 
  
}

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
   
extern "C"
{
 
  void JunoInit(RTC::Manager* manager)
  {
    coil::Properties profile(juno_spec);
    manager->registerFactory(profile,
                             RTC::Create<Juno>,
                             RTC::Delete<Juno>);
  }
  
};

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_Fin//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
/*
void Juno::setObjectV(double x, double y, double z, double roll, double pitch, double yaw)
{ 
  vector32 zero(0);
  //V in
  object_ref->p(0)+=x;
  object_ref->p(1)+=y;
  object_ref->p(2)+=z;
  vector3 rpy(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180);
  object_ref->R = hrp::rotFromRpy(rpy);
   
  if(CalcIVK_NMS(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, stick_r_A, stick_l_A, 
	  body_B, cm_ref_B, p_ref_B, R_ref_B, FT_B, p_Init_B, R_Init_B, stick_r_B, stick_l_B, object_ref )){
    cerr<<"test OK"<<endl;
    
    for(int i=0;i<body_A->numJoints();i++)
      body_ref_A[i]=body_A->joint(i)->q;
    for(int i=0;i<body_B->numJoints();i++)
      body_ref_B[i]=body_B->joint(i)->q;   

    Interplation5(body_cur_A,  zero,  zero, body_ref_A,  zero,  zero, 1, bodyDeque_A);
    Interplation5(body_cur_B,  zero,  zero, body_ref_B,  zero,  zero, 1, bodyDeque_B);

    body_cur_A=body_ref_A;
    body_cur_B=body_ref_B;
 
  }
  else
    cerr<<"test err"<<endl;
}
////////////////put in control loop
if(!bodyDeque_A.empty()){
	for(int i=0;i<body_A->numJoints();i++)
	  m_refq_A.data[i]=bodyDeque_A.at(0)[i];
	for(int i=0;i<body_B->numJoints();i++)
	  m_refq_B.data[i]=bodyDeque_B.at(0)[i];
	
	bodyDeque_A.pop_front();
	bodyDeque_B.pop_front();
	m_refq_AOut.write();
	m_refq_BOut.write();
      }
      //////////////////


*/
/*
void Juno::onlineMonitor(BodyPtr body, FootType &FT, ZmpPlaner *zmpP, int &count, Vector3 *p_Init, Matrix33 *R_Init)
{ 
  count++;
  my_ForwardKinematics(body, FT, p_Init, R_Init);

  switch(FT){
  case FSRFsw:
    if(count== zmpP->step1Num ){
      FT=LFsw;
      RenewModel(body, p_Init, R_Init);
      count=0;
    }
    break;
    
  case FSLFsw:
    if(count== zmpP->step1Num){
      FT=RFsw;
      RenewModel(body, p_Init, R_Init);
      count=0;
    }
    break;
    
  case LFsw:
    if(count==zmpP->NomalPaceNum){	
      FT=RFsw; 
      RenewModel(body, p_Init, R_Init);
      count=0;
    }
    break;
    
  case RFsw:
    if(count==zmpP->NomalPaceNum){
      FT=LFsw;
      RenewModel(body, p_Init, R_Init);
      count=0;
    }
    break;
  } 
 
}


 */
