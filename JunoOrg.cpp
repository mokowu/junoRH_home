// -*- C++ -*-
/*!
 * @file  Juno.cpp * @brief NolanComponent * $Date$ 
 *
 * $Id$ 
 */
#include "Juno.h"
std::ofstream ofs("/home/grxuser/users/wu/juno/juno.log");
// Module specification
// <rtc-template block="module_spec">
static const char* juno_spec[] =
  {
    "implementation_id", "Juno",
    "type_name",         "Juno",
    "description",       "NolanComponent",
    "version",           "1.0",
    "vendor",            "tohoku",
    "category",          "example",
    "activity_type",     "SPORADIC",
    "kind",              "NolanComponent",
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
    m_rhsensor_AIn("rhsensor_A", m_rhsensor_A),
    m_lhsensor_AIn("lhsensor_A", m_lhsensor_A),
    m_rfsensor_AIn("rfsensor_A", m_rfsensor_A),
    m_lfsensor_AIn("lfsensor_A", m_lfsensor_A),
    m_baseRPYInit_AIn("baseRPYInit_A", m_baseRPYInit_A),
    m_basePOSInit_AIn("basePOSInit_A", m_basePOSInit_A),
    m_mc_BIn("mc_B", m_mc_B),
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
  addInPort("rhsensor_A", m_rhsensor_AIn);
  addInPort("lhsensor_A", m_lhsensor_AIn);
  addInPort("rfsensor_A", m_rfsensor_AIn);
  addInPort("lfsensor_A", m_lfsensor_AIn);
  addInPort("baseRPYInit_A", m_baseRPYInit_AIn);
  addInPort("basePOSInit_A", m_basePOSInit_AIn);
  addInPort("mc_B", m_mc_BIn);
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
  m_ToSequencePlayerServicePort.registerConsumer("serviceSeq0", "SequencePlayerService", m_serviceSeq0);

  // Set CORBA Service Ports
  addPort(m_JunoServicePort);
  addPort(m_ToSequencePlayerServicePort);

  prop=this->getProperties();
  
  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int commaPos = nameServer.find(",");
  if (commaPos > 0)
    nameServer = nameServer.substr(0, commaPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  body_A = new hrp::Body();
  body_B = new hrp::Body();
  CosNaming::NamingContext_var m_rootNameContext = CosNaming::NamingContext::_duplicate(naming.getRootContext());

  if (!hrp::loadBodyFromModelLoader(body_A,  prop["model"].c_str(), m_rootNameContext))
    //if (!hrp::loadBodyFromModelLoader(body,  URL, m_rootNameContext))
    //if (!hrp::loadBodyFromModelLoader(body, URL, argc,argv))
    {
      std::cerr <<" : failed to load model" << std::endl;
    }
  if (!hrp::loadBodyFromModelLoader(body_B,  prop["model"].c_str(), m_rootNameContext))
    {
      std::cerr <<" : failed to load model" << std::endl;
    }

  dof = body_A->numJoints();
  if( body_A->numJoints()==32){
    armDof=7;
  }
  else if( body_A->numJoints()==30){
    armDof=6;
  }  

  prop["kgain"]>>kgain;
  prop["fgain"]>>fgain;
  //A
  for(unsigned int i=0;i<dof;i++)
    body_A->joint(i)->q=0; 
  body_A->link("WAIST")->p=0, 0, 0.705;
  body_A->calcForwardKinematics();
  RenewModel(body_A, p_now_A, R_now_A);
  updateInit(p_now_A, p_Init_A, R_now_A, R_Init_A);
  body_A->calcCM();
  //B
  //dof_B = body_B->numJoints();//careful
  for(unsigned int i=0;i<dof;i++)
    body_B->joint(i)->q=0; 
  body_B->link("WAIST")->p=1.2338, 0, 0.705;//careful
  //180 rotate
  body_B->link("WAIST")->R(0,0)=body_B->link("WAIST")->R(1,1)=-1;
  body_B->calcForwardKinematics();
  RenewModel(body_B, p_now_B, R_now_B);
  updateInit(p_now_B, p_Init_B, R_now_B, R_Init_B);
  body_B->calcCM();
  
  //pini
  count=0;
  playflag=0;
  stopflag_A=1;
  stopflag_B=1;
  //data port
  m_mc_A.data.length(dof);
  m_rhsensor_A.data.length(6);
  m_lhsensor_A.data.length(6);
  m_rfsensor_A.data.length(6);
  m_lfsensor_A.data.length(6);  
  m_baseRPYInit_A.data.length(3);
  m_basePOSInit_A.data.length(3); 
  
  m_rzmp_A.data.length(3); 
  m_refq_A.data.length(dof);
  m_baseRPY_A.data.length(3); 
  m_basePOS_A.data.length(3); 

  m_mc_B.data.length(dof);
  m_rhsensor_B.data.length(6);
  m_lhsensor_B.data.length(6);
  m_rfsensor_B.data.length(6);
  m_lfsensor_B.data.length(6);  
  m_baseRPYInit_B.data.length(3);
  m_basePOSInit_B.data.length(3); 
  
  m_rzmp_B.data.length(3); 
  m_refq_B.data.length(dof);
  m_baseRPY_B.data.length(3); 
  m_basePOS_B.data.length(3); 
  
  
  absZMP_A=absZMP_B=(Vector3)(0);
  relZMP_A=relZMP_B=(Vector3)(0);
  absZMP_B(0)=relZMP_B(0)=1;//becareful!!!!

  flagcalczmp=1;
  FT_A =FSRFsw;
  FT_B =FSRFsw;
  CommandIn=5;
  time2Neutral=0.5;

  //test paraini
  velobj=(vector6)(0);

  //NMS
  NMS= new NonMasterSlave( prop["model"], prop["model"]);//careful!!
  stick_r_A= new hrp::Link();
  stick_l_A= new hrp::Link();
  stick_r_B= new hrp::Link();
  stick_l_B= new hrp::Link();
  object_ref= new hrp::Link();
  eh=dzerovector(14);//careful
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
  
  //init
  setVector3(body_A->link("RLEG_JOINT5")->p, p_Init_A[0]);
  setVector3(body_A->link("LLEG_JOINT5")->p, p_Init_A[1]);
  setMatrix33(body_A->link("RLEG_JOINT5")->R, R_Init_A[0]);
  setMatrix33(body_A->link("LLEG_JOINT5")->R, R_Init_A[1]);    
  setVector3(body_B->link("RLEG_JOINT5")->p, p_Init_B[0]);
  setVector3(body_B->link("LLEG_JOINT5")->p, p_Init_B[1]);
  setMatrix33(body_B->link("RLEG_JOINT5")->R, R_Init_B[0]);
  setMatrix33(body_B->link("LLEG_JOINT5")->R, R_Init_B[1]);    
  
  //log
  m_Fobj.data.length(6); 


  hoge=1;
  // </rtc-template>
  return RTC::RTC_OK;
}


RTC::ReturnCode_t Juno::onExecute(RTC::UniqueId ec_id)
{
  //wflagRzmp=0;
  //A
  if(m_mc_AIn.isNew()&&m_mc_BIn.isNew()){
    //A 
    m_mc_AIn.read();
    for(unsigned int i=0;i<m_mc_A.data.length();i++)
      m_refq_A.data[i]=m_mc_A.data[i];
    setModelPosture(body_A, m_mc_A, FT_A, p_Init_A, R_Init_A);
    RenewModel(body_A, p_now_A, R_now_A);
    //B
    m_mc_BIn.read();
    for(unsigned int i=0;i<m_mc_B.data.length();i++)
      m_refq_B.data[i]=m_mc_B.data[i];
    setModelPosture(body_B, m_mc_B, FT_B, p_Init_B, R_Init_B);
    RenewModel(body_B, p_now_B, R_now_B);
  } 

  if( m_rhsensor_AIn.isNew() ) 
    m_rhsensor_AIn.read();
  if( m_lhsensor_AIn.isNew() ) 
    m_lhsensor_AIn.read();
  if( m_rfsensor_AIn.isNew() ) 
    m_rfsensor_AIn.read();
  if( m_lfsensor_AIn.isNew() ) 
    m_lfsensor_AIn.read();
  //B
  if( m_rhsensor_BIn.isNew() ) 
    m_rhsensor_BIn.read();
  if( m_lhsensor_BIn.isNew() ) 
    m_lhsensor_BIn.read();
  if( m_rfsensor_BIn.isNew() ) 
    m_rfsensor_BIn.read();
  if( m_lfsensor_BIn.isNew() ) 
    m_lfsensor_BIn.read();
  ////////////////////main algorithm///////////////////////////////////////////////////
  if(playflag){
    
    object_operate();   
    prmGenerator();



    calcWholeIVK();

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
   
    
    //////////////write///////////////
    m_baseRPY_AOut.write();
    m_basePOS_AOut.write();    
    
    //rzmp2st();
    // if(wflagRzmp){  
    m_rzmp_AOut.write();
    m_rzmp_BOut.write();
      //}
    m_wZMP_AOut.write();
    m_wZMP_BOut.write();
    
    m_FobjOut.write(); 
  }//playflag

  return RTC::RTC_OK;
}
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
   
//function
inline void Juno::getInvResult()
{
 getModelPosture(body_A, m_refq_A);
 getModelPosture(body_B, m_refq_B);

 /*
  for(int i = 0; i < armDof; i++) {
  m_refq.data[i+16]+=  eh[i];
  m_refq.data[i+17+armDof]+= eh[i];	
  }
 */
 /*
  if(!clipDeque_A.empty()){
    m_refq_A.data[23]=clipDeque_A.at(0)[0];
    m_refq_A.data[31]=clipDeque_A.at(0)[1];
    clipDeque_A.pop_front();
  }
 */
   
   m_refq_AOut.write();
   m_refq_BOut.write();
}
inline void Juno::calcWholeIVK()
{
  if(CalcIVK_NMS(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, stick_r_A, stick_l_A, 
		 body_B, cm_ref_B, p_ref_B, R_ref_B, FT_B, p_Init_B, R_Init_B, stick_r_B, stick_l_B, object_ref )){
    getInvResult();
  }
  else{
    cerr<<"ivk err"<<endl;
  }
}

inline void Juno::rzmp2st()
{
  RenewModel(body_A, p_now_A, R_now_A);
  relZMP_A = trans(body_A->link("WAIST")->R)*(absZMP_A - body_A->link("WAIST")->p);
  RenewModel(body_B, p_now_B, R_now_B);
  relZMP_B = trans(body_B->link("WAIST")->R)*(absZMP_B - body_B->link("WAIST")->p);
  for(int i=0;i< m_rzmp_A.data.length();i++){
    m_rzmp_A.data[i]=relZMP_A[i];  
    m_rzmp_B.data[i]=relZMP_B[i];  
  }
  //wflagRzmp=1;
}

inline void Juno::object_operate()
{
  //by operator
  Vector3 tep;
  //translation
  tep=velobj(0)*0.00005,velobj(1)*0.00005, velobj(2)*0.00005;
  //rotate
  Vector3 rpy(0.01*velobj(3)*M_PI/180, 0.01*velobj(4)*M_PI/180, 0.01*velobj(5)*M_PI/180);
  Matrix33 rotR = hrp::rotFromRpy(rpy);

  //ref////////
  object_ref->p = object_ref->p + object_ref->R*tep; 
  object_ref->R = rotR * object_ref->R;

 
}

inline void Juno::calcRefLeg()
{
  Vector3 Rpytem= rpyFromRot(object_ref->R);
  Rpytem(0)=Rpytem(1)=0;
  Matrix33 Rtem_Q= rotFromRpy(Rpytem);
  //actually in x-y plan only
  RLEG_ref_p_A = object_ref->p + Rtem_Q * p_obj2RLEG_A; 
  LLEG_ref_p_A = object_ref->p + Rtem_Q * p_obj2LLEG_A;
  RLEG_ref_R_A= LLEG_ref_R_A= object_ref->R* R_LEG_ini_A;
  
  RLEG_ref_p_B = object_ref->p + Rtem_Q * p_obj2RLEG_B; 
  LLEG_ref_p_B = object_ref->p + Rtem_Q * p_obj2LLEG_B;
  RLEG_ref_R_B= LLEG_ref_R_B= object_ref->R* R_LEG_ini_B;
}

inline void Juno::prmGenerator()//object rotate in yow direction only
{
  calcRefLeg();
  //start to walk
  Vector3 FErr(0);
  Vector3 FErrb(0);
  Vector3 object_omega_err(0);
  Vector3 object_omega_errb(0);

  /*
  if((FT==FSRFsw)||(FT==RFsw)){
    FErr=  LLEG_ref_p - body->link("LLEG_JOINT5")->p; 
    FErrb=  RLEG_ref_p - body->link("RLEG_JOINT5")->p; 
    object_omega_err=body->link("LLEG_JOINT5")->R * omegaFromRot(Matrix33(trans(body->link("LLEG_JOINT5")->R) * object_ref->R));
    object_omega_errb=body->link("RLEG_JOINT5")->R * omegaFromRot(Matrix33(trans(body->link("RLEG_JOINT5")->R) * object_ref->R));
  }
  else if((FT==FSLFsw)||(FT==LFsw)){
    FErr=  RLEG_ref_p - body->link("RLEG_JOINT5")->p; 
    FErrb=  LLEG_ref_p - body->link("LLEG_JOINT5")->p; 
    object_omega_err=body->link("RLEG_JOINT5")->R * omegaFromRot(Matrix33(trans(body->link("RLEG_JOINT5")->R) * object_ref->R));
    object_omega_errb=body->link("LLEG_JOINT5")->R * omegaFromRot(Matrix33(trans(body->link("LLEG_JOINT5")->R) * object_ref->R));
  }

  if((sqrt(FErr(0)*FErr(0)+FErr(1)*FErr(1))>0.03)||(sqrt(FErrb(0)*FErrb(0)+FErrb(1)*FErrb(1))>0.03)){
    if( stopflag && PC->CoM2Stop.empty() ){
      CommandIn=0;//start to walk
    }
  }

  if(dot(object_omega_err, object_omega_err)>0.03||(dot(object_omega_errb, object_omega_errb)>0.03)){
    if( stopflag && PC->CoM2Stop.empty() ){
      CommandIn=0;//start to walk
    }
  }

  //stop walking
  if(count==(stepLength(FT,zmpP)- 2*zmpP->TdblNum)){  
    if(sqrt(FErr(0)*FErr(0)+FErr(1)*FErr(1))<0.03 && dot(object_omega_err, object_omega_err)<0.03 ){//&&swingleg _R object_ref_R
      CommandIn=5;
    }//quick stop
  }
*/
}

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
   
//method
void Juno::testMove()
{
}

void Juno::setObjectV(double x, double y, double z, double roll, double pitch, double yaw)
{ 
  velobj= x,y,z,roll,pitch,yaw;
}


void Juno::fcontrol()
{
  dvector a_delta_p_r(18);
  updateObject(body_A, stick_r_A, stick_l_A, body_B, stick_r_B, stick_l_B, object_ref, a_delta_p_r);
  cerr<<a_delta_p_r<<endl;
  cerr<<object_ref->p<<endl;
}

void Juno::start()
{
  cm_ref_A=body_A->calcCM();// 
  cm_ref_B=body_B->calcCM();
 
  //for expos
  for(int i=0;i<LINKNUM;i++){
    setVector3(p_now_A[i], p_Init_A[i]);
    setMatrix33(R_now_A[i], R_Init_A[i]);
    setVector3(p_now_A[i], p_ref_A[i]);
    setMatrix33(R_now_A[i], R_ref_A[i]);
    //B
    setVector3(p_now_B[i], p_Init_B[i]);
    setMatrix33(R_now_B[i], R_Init_B[i]);
    setVector3(p_now_B[i], p_ref_B[i]);
    setMatrix33(R_now_B[i], R_ref_B[i]);
  }
  //R_ref_A[0]= R_ref_A[1]=R_ref_B[0]= R_ref_B[1]=tvmet::identity<hrp::Matrix33>();
  //B turn 180  
  //R_ref_B[0](0,0)= R_ref_B[1](0,0)= R_ref_B[0](1,1)= R_ref_B[1](1,1)=-1;

  //renew body in NMS
  NMS->reNewModel( m_mc_A, FT_A, p_Init_A, R_Init_A,  m_mc_B, FT_B, p_Init_B, R_Init_B);
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
  //ini
  p_obj2RLEG_A = body_A->link("RLEG_JOINT5")->p - object_ref->p; 
  p_obj2LLEG_A = body_A->link("LLEG_JOINT5")->p - object_ref->p; 
  p_obj2RLEG_B = body_B->link("RLEG_JOINT5")->p - object_ref->p; 
  p_obj2LLEG_B = body_B->link("LLEG_JOINT5")->p - object_ref->p; 

  Vector3 Rtem(0);
  R_LEG_ini_A= rotFromRpy(Rtem);
  Rtem(2)= M_PI;//180 rotate
  R_LEG_ini_B= rotFromRpy(Rtem);
  
  //ref
  calcRefLeg();

  //////////////////////////////////////////////////////////////

  cerr<<"startQ"<<endl;
  playflag=1;
}

void Juno::stop()
{

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

  if(hoge){
    for(int i=0;i<body_A->numJoints();i++)
      body_cur_A[i]=body_A->joint(i)->q;
    for(int i=0;i<body_B->numJoints();i++)
      body_cur_B[i]=body_B->joint(i)->q;
    hoge=0;
  }
   
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
