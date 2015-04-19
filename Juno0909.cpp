// -*- C++ -*-
/*!
 * @file  Juno.cpp * @brief NolanComponent * $Date$ 
 *
 * $Id$ 
 */
//REAL TIME VERSION wu
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
    m_stuck_AIn("stuck_A",m_stuck_A),
    m_stuck_BIn("stuck_B",m_stuck_B),
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


/////////////for thread/////////////////////////////////
void foo(int i)
{
  std::cout << "hello from foo "<<i <<endl;
}

///////////////////////////////////////


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

  addInPort("stuck_A", m_stuck_AIn);
  addInPort("stuck_B", m_stuck_BIn);
  //call back
  //m_mc_AIn.addConnectorDataListener(ON_RECEIVED,new DataListener_A("ON_RECEIVED", this));
  //m_mc_BIn.addConnectorDataListener(ON_RECEIVED,new DataListener_B("ON_RECEIVED", this));

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
  preset_flag=0;
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
  p_all_ref=dzerovector(24);  
  p_all=dzerovector(24);  

  stick_rpy_r_A_buf=stick_rpy_l_A_buf=Vector3(0);
  stick_rpy_r_B_buf=stick_rpy_l_B_buf=Vector3(0);
  // </rtc-template> 
  
  MAX_TRANSITION_COUNT= 50;
  transition_count= -MAX_TRANSITION_COUNT;
  step=0;

  //log
  m_Fobj.data.length(6); 

  //pini2
  rpcrr=vector2(0);
  yawTotal=0;


  ref_q_load_A=ref_q_load_B=0;
  //for offline
  step_num=0;

  comInpo_flag_A=0;
  comInpo_flag_B=0;
  comInpo_stop_A=0;
  comInpo_stop_B=0;

  //optcap 
  optcap=0;

  //for inipos adjust
  stick_r_single= new hrp::Link();
  stick_l_single= new hrp::Link();
  object_ref_single= new hrp::Link();

  cmain=cA=cB=0;
 
  A_seq=B_seq=0;
  A_go=B_go=1;
  printDebug=0;
  //thread
  flag_t=1;
   
  //auto func =  std::mem_fn(&Juno::threadfunc_main);
  //std::thread t_counter(func, this);
  //t_counter.detach();

  callFrom_A=callFrom_B= remot;
  first_loop=1;

  /*
  //signal( SIGALRM, my_alarm_handler );
  //alarm(1);

  struct sigaction action; 
  struct itimerval timer; 
  memset(&action, 0, sizeof(action)); //set signal handler  
  action.sa_handler = my_alarm_handler; 
  //action.sa_flags = SA_RESTART;
  */


  /*
  ///////////////
  auto now = std::chrono::system_clock::now();
  std::function<void()> func;
  func=bind(foo,1);
  events::add(func, now + std::chrono::seconds(4));
  func=bind(foo,3);
  events::add(func, now + std::chrono::seconds(2));
  func=bind(foo,9);
  events::add(func, now + std::chrono::seconds(6));
  CManiac maniac;          // クラスオブジェクト maniac を作成する。
  int iValue = maniac();   // 単項の+ 演算子を maniac に適用する。
  iValue = +maniac;
  printf("iValue = %d\n", iValue);    // 答えは 2 になります。
  ////////////////////
  */

  /*
  //first write
  auto funcA =  std::mem_fn(&Juno::send_A);
  std::thread tsea(funcA, this);
  auto funcB =  std::mem_fn(&Juno::send_B);
  std::thread tseb(funcB, this);
  tsea.join();
  tseb.join();
  */

  server= new iserver;
  server->setDataLength(dof_A, dof_B);
   
  ////std::function<void()> func;
  //auto func = std::bind(std::mem_fn(&iserver::wait_connect), server);
  //std::thread t_connect(func);
  
  //use this
  auto func =  std::mem_fn(&iserver::wait_connect);
  std::thread t_connect(func,server);
  t_connect.detach();
  
  //one more time


  m_stuck_A.data=m_stuck_B.data=0.0;
  stuck_A=stuck_B=0;

  tempcount=0;
  temptotal=0;
  cout<<"init end"<<endl;
    // </rtc-template>
  return RTC::RTC_OK;
}


void Juno::write_A()  
{ 
  //data port
  while(!m_refq_AOut.write()){
    cout<<"unreach A"<<endl;
  }
  m_rzmp_AOut.write();

}
void Juno::write_B()  
{ 
  //data port
  while(!m_refq_BOut.write()){
    cout<<"unreach B"<<endl;
   }
  m_rzmp_BOut.write();

}

//this is counter.unuse
void Juno::threadfunc_main()  
{ 
  while(1){
     
    usleep(100);//micro second
    events::timer();


    if(flag_t==0)
      break;

  }//while 1
}

void Juno::calcNextTrajectory()
{

  while(1){
    ///////////////////////////////////////////////////////////////////
     
    if(playflag){ 

      timeval tv_a,tv_z;
      gettimeofday(&tv_a,0);
      
      ///miso//
      if(stuck_A>1&&stuck_B>1&&stopflag_A&&stopflag_B){
                //if(m_stuck_A.data>2&&m_stuck_B.data>2&&stopflag_A&&stopflag_B){
                //if(m_stuck_A.data>2&&m_stuck_B.data>2){
	usleep(4000);
	gettimeofday(&tv_a,0);
      }
      
      //walking
      else if(m_stuck_A.data>7&&m_stuck_B.data>7){
	if(!stopflag_A||!stopflag_B){
	  usleep(4000);//>not test yet sleep (7-2)*5000 ?
	  gettimeofday(&tv_a,0);
	}
      }
      
      //if(!stopflag_A||!stopflag_B)
      //cout<<"AB "<<stuck_A<<" "<<stuck_B<<endl;
      /////////
      /*
      if(m_stuck_AIn.isNew())
	m_stuck_AIn.read();
      if(m_stuck_BIn.isNew())
	m_stuck_BIn.read();
      */

      fSensorRead();
      ////////fcontrol///////////////
      if(forceControl){
	//mc here unused
	NMS->controlRoutine(FT_A, m_rhsensor_A, m_lhsensor_A, m_mc_A, body_A, 
			    FT_B, m_rhsensor_B, m_lhsensor_B, m_mc_B, body_B,  
			    R_ref_A[WAIST],R_ref_B[WAIST]); 

	NMS->get_p_all(p_all);
	//NMS->get_eh(eh_buf);//old way
      }
      else
	p_all=hrp::dzerovector(24);
      //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
      gettimeofday(&tv_a,0);
      main_algorithm();//invkinemetic, rzmp2st here
      
      gettimeofday(&tv_z,0);
      //write socket
      write2client();
     
      //sleep
      double dif=tv_z.tv_usec- tv_a.tv_usec;
      double sleep_t=4700 - dif;
      //double sleep_t_walking=4700 - dif; //4200
      
      if(dif>4900)
	cout << "dif_time "<<dif  << std::endl;
      
      /*
      if(!stopflag_A||!stopflag_B){
	if(dif>0){
	  tempcount++;
	  temptotal+=dif;
	}
      }
      */
      
      if(sleep_t>0 && dif>0){
	//if(stopflag_A&&stopflag_B)
	  usleep(sleep_t);
      }
      
      /*
      //walking
      if(sleep_t_walking>0 && dif>0){
	if(!stopflag_A||!stopflag_B){
	  //if(stuck_A>30&&stuck_B>30)
	    usleep(sleep_t_walking);
	  //cout<<"sleep "<<sleep_t_walking<<endl;
	}
      }
      */

    }//playflag
    //////////////////////////////////////////////////////////////////
    if(flag_t==0)
      break;
  }///while(1)

}



void Juno::getTime(timeval &tv, std::mutex &mtx)
{
  std::lock_guard<std::mutex> lock(mtx);
  gettimeofday(&tv,0);
}

void Juno::send_A()
{
  std::lock_guard<std::mutex> lock(mtx_sA);
  server->send2client_A(m_refq_A, stuck_A);
   
}

void Juno::send_B()
{
  std::lock_guard<std::mutex> lock(mtx_sB);
  server->send2client_B(m_refq_B, stuck_B);
}

void Juno::routine_A()
{
 
  if(callFrom_A==local){
    callFrom_A=remot;
    //cout<<"return A"<<endl;
    return;
  }
  
  events::pop_A();
  send_A();
  
}

void Juno::routine_B()
{
  //std::lock_guard<std::mutex> lock(mtx_comuB);
  if(callFrom_B==local){
    callFrom_B=remot;
    //cout<<"return B"<<endl;
    return;
  }
  
  events::pop_B();
  send_B();
  
}




void Juno::add_A(vector32 value)
{
  std::lock_guard<std::mutex> lock(mtx_A);
  bodyDeque_A.push_back(value);
}

void Juno::add_B(vector30 value)
{
  std::lock_guard<std::mutex> lock(mtx_B);
  bodyDeque_B.push_back(value);
}

void Juno::get_A(TimedDoubleSeq &m_refq)
{
  std::lock_guard<std::mutex> lock(mtx_A);
  for(int i=0;i<dof_A;i++)
    m_refq.data[i]=bodyDeque_A.at(0)[i];
  bodyDeque_A.pop_front();

}
           
void Juno::get_B(TimedDoubleSeq &m_refq)
{
  std::lock_guard<std::mutex> lock(mtx_B);
  for(int i=0;i<dof_B;i++)
    m_refq.data[i]=bodyDeque_B.at(0)[i];
  bodyDeque_B.pop_front();

}

int Juno::Asize()
{
  std::lock_guard<std::mutex> lock(mtx_A);
  int a=(int)bodyDeque_A.size();
  if(a>1)
    cout<<"a "<<a<<endl;
  return a;
}

int Juno::Bsize()
{
  std::lock_guard<std::mutex> lock(mtx_B);
  int b=(int)bodyDeque_B.size();
  if(b>1)
    cout<<"b "<<b<<endl;
  return b;
}

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
   for ( int i = 0; i < body_A->numJoints(); i++ ){
     m_refq_A.data[i]  = ( m_refq_A.data[i] - qRef_tem_A(i) ) * transition_smooth_gain + qRef_tem_A(i);
     //for ( int i = 0; i < body_B->numJoints(); i++ )
     if(i<dof_B)
       m_refq_B.data[i]  = ( m_refq_B.data[i] - qRef_tem_B(i) ) * transition_smooth_gain + qRef_tem_B(i);
   }
 }
 ///////////////////////////////

 /*
 //for data port
 vector32 temp_q_A;
 vector30 temp_q_B;
#pragma omp parallel sections
 {
#pragma omp section
   for(int i=0;i<dof_A;i++)
     temp_q_A(i)= m_refq_A.data[i];

   add_A(temp_q_A);
  
#pragma omp section
   for(int i=0;i<dof_B;i++)
     temp_q_B(i)= m_refq_B.data[i];
 
   add_B(temp_q_B);
 
 }//pragma
 */
}

void Juno::calcIvkA(bool &r)
{
  r=CalcIVK_ARM_NUM_elbow(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, elbow_A);
}

void Juno::calcIvkB(bool &g)
{
  g= CalcIVK_ARM_NUM(body_B, cm_ref_B, p_ref_B, R_ref_B, FT_B, p_Init_B, R_Init_B);
}

inline void Juno::calcWholeIVK()
{
  //calculate seperate
  CalcIVK_NMS_ARM(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, stick_r_A, stick_l_A, 
	          body_B, cm_ref_B, p_ref_B, R_ref_B, FT_B, p_Init_B, R_Init_B, stick_r_B, stick_l_B, 
		  stick_rpy_r_A_buf, stick_rpy_l_A_buf, stick_rpy_r_B_buf, stick_rpy_l_B_buf, object_ref, p_all);
  
  //parallel compute
  bool r,g;

  
  #pragma omp parallel sections
     {
  #pragma omp section
    r=CalcIVK_ARM_NUM_elbow(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, elbow_A);
    #pragma omp section
    g=      CalcIVK_ARM_NUM(body_B, cm_ref_B, p_ref_B, R_ref_B, FT_B, p_Init_B, R_Init_B);
    }
  

  /*
  auto funcIA =  std::mem_fn(&Juno::calcIvkA);
  std::thread t_ivkA(funcIA, this, std::ref(r));
  auto funcIB =  std::mem_fn(&Juno::calcIvkB);
  std::thread t_ivkB(funcIB, this, std::ref(g));
  t_ivkA.join();
  t_ivkB.join();
  */

  if(r&&g)
    getInvResult();
  else
    cerr<<"err ivk shit!"<<endl;


}

inline void Juno::update_mc(BodyPtr body)
{
  if(body->numJoints()==32){
    //A 
    m_mc_AIn.read();//comment in non RH
    //for(unsigned int i=0;i<m_mc_A.data.length();i++)
    //  m_refq_A.data[i]=m_mc_A.data[i];
    setModelPosture(body_A, m_mc_A, FT_A, p_Init_A, R_Init_A);
    //RenewModel(body_A, p_now_A, R_now_A);
  }
  else if(body->numJoints()==30){
    //B
    m_mc_BIn.read();//comment in non RH
    //for(unsigned int i=0;i<m_mc_B.data.length();i++)
    //  m_refq_B.data[i]=m_mc_B.data[i];
    setModelPosture(body_B, m_mc_B, FT_B, p_Init_B, R_Init_B);
    //RenewModel(body_B, p_now_B, R_now_B);
  }
}

inline void Juno::main_algorithm()
{
   //_/_/_/_/_/_/_/_/_/_/_/_/main algorithm_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
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
  calcWholeIVK();//write in refq_deque here
  zmpHandler();
    
  //for next step
  if(ChangeSupLeg(body_A, FT_A, count_A, zmpP_A, PC_A, stopflag_A, CommandIn_A, p_ref_A, p_Init_A, R_ref_A, R_Init_A, comInpo_flag_A))
    flagcalczmp_A=1;
  if(ChangeSupLeg(body_B, FT_B, count_B, zmpP_B, PC_B, stopflag_B, CommandIn_B, p_ref_B, p_Init_B, R_ref_B, R_Init_B, comInpo_flag_B))
    flagcalczmp_B=1;
  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/  
  //////////////write///////////////   
  //rzmp2st();//integrate in zmpHandler

}

inline void Juno::fSensorRead()
{
  //A 
  if( m_rhsensor_AIn.isNew() )
    m_rhsensor_AIn.read();
  
  if( m_lhsensor_AIn.isNew() )
    m_lhsensor_AIn.read();
  
  //B
  if( m_rhsensor_BIn.isNew() )
    m_rhsensor_BIn.read();
  
  if( m_lhsensor_BIn.isNew() )
    m_lhsensor_BIn.read();
  
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
 }

//integrate into zmpHandler
inline void Juno::rzmp2st()
{
  relZMP_A = trans(R_ref_A[WAIST])*(absZMP_A - body_A->link("WAIST")->p);
  relZMP_B = trans(R_ref_B[WAIST])*(absZMP_B - body_B->link("WAIST")->p);
 
  for(int i=0;i< 3;i++){
    m_rzmp_A.data[i]=relZMP_A[i];
    m_rzmp_B.data[i]=relZMP_B[i];
  }
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
    
    //rzmp2st_A
    relZMP_A = trans(R_ref_A[WAIST])*(absZMP_A - body_A->link("WAIST")->p);
    for(int i=0;i< 3;i++){
      m_rzmp_A.data[i]=relZMP_A[i];
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

    //rzmp2st_B
    relZMP_B = trans(R_ref_B[WAIST])*(absZMP_B - body_B->link("WAIST")->p);
    for(int i=0;i< 3;i++){
      m_rzmp_B.data[i]=relZMP_B[i];
    }


    //for log
    //absZMPDeque_log_A.push_back(absZMP_A);
    //absZMPDeque_log_B.push_back(absZMP_B);
}

inline void Juno::object_operate()
{
  //by operator
  Vector3 tep(0);
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
  double rot_deg;
  if(!yawdeque.empty()){
    yawTotal+=0.01*yawdeque.at(0)*M_PI/180;
    rot_deg=yawdeque.at(0);
    yawdeque.pop_front();
  }
  else{//usually this
    yawTotal+=0.01*velobj(5)*M_PI/180;
    rot_deg=velobj(5);
  }

  if(!rpdeque.empty()) {//for rotate ICRA
    Vector3 rpyTemp;
    rpyTemp=Vector3(rpdeque.at(0)[0], rpdeque.at(0)[1], yawTotal) ;
    rotRTemp = hrp::rotFromRpy(rpyTemp);
    rpdeque.pop_front();
  }
  else{//usually this
    Matrix33 rZ(rotationZ( 0.01*rot_deg*M_PI/180));
    rotRTemp = rZ*rotRTemp ;
  }
  
  object_ref->R = rotRTemp;
  object_ref->p = object_ref->p + rotationZ(yawTotal)*tep; 

  object_p_Deque_log.push_back(object_ref->p);
}


inline void Juno::write2client()
{
  //write
  //auto funcA =  std::mem_fn(&Juno::write_A);
  //auto funcB =  std::mem_fn(&Juno::write_B);
  
  auto funcA = std::bind(std::mem_fn(&Juno::send_A), this);
  std::thread tsea(funcA);
  auto funcB = std::bind(std::mem_fn(&Juno::send_B), this);
  std::thread tseb(funcB);
  //auto funcA =  std::mem_fn(&iserver::send2client_A);
  //std::thread tsea(funcA, server, m_refq_A,std::ref(stuck_A));
  //auto funcB =  std::mem_fn(&iserver::send2client_B);
  //std::thread tseb(funcB, server, m_refq_B,std::ref(stuck_B));
  tsea.detach();
  tseb.detach();

  
  
  /*
    #pragma omp parallel sections
    {
    #pragma omp section
    server->send2client_A(m_refq_A, stuck_A);
    #pragma omp section
    server->send2client_B(m_refq_B, stuck_B);
    }//pragma
  */
  
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
  
  /*
  //object_cur ver commmet out 0429
  hrp::Link* c_object;
  c_object=new Link();
  dvector a_delta_p_r(18);
  updateObject(body_A, stick_r_A, stick_l_A, body_B, stick_r_B, stick_l_B,  stick_rpy_r_A_buf, stick_rpy_l_A_buf, stick_rpy_r_B_buf, stick_rpy_l_B_buf, c_object, a_delta_p_r);
  //ofs<<object_ref->p(0)<<" "<<object_ref->p(1)<<" "<<object_ref->p(2)<<" "<<c_object->p(0)<<" "<<c_object->p(1)<<" "<<c_object->p(2)<<endl;  
  Matrix33 Rtem_Q=extractYow(c_object->R);
  */
  //light ver
  Matrix33 Rtem_Q=extractYow(object_ref->R);

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

}


void Juno::prmGeneA(bool &flagA)
{
  //start to walk or not A
  //waiting
  if( stopflag_A && PC_A->CoM2Stop.empty() ){
    //normal
    //if(walkJudge(p_ref_A, R_ref_A, FT_A, RLEG_ref_p_A, LLEG_ref_p_A, LEG_ref_R_A)){
    //for stepping
    if(step){ //commet out    
      CommandIn_A=0;//start to walk
      start2walk(body_A, zmpP_A, PC_A, stopflag_A, cm_ref_A);//stopflag off ini PC,zmpP
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

}

void Juno::prmGeneB(bool &flagB)
{ 
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

inline void Juno::prmGenerator(bool &flagA, bool &flagB)
{
  calcRefLeg();

  
  if(step_num==4){//8 default
    step=0;
    step_num=0;
    velobj= 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    //cout<<"stop"<<endl;
    }

  #pragma omp parallel sections
     {
  #pragma omp section
  prmGeneA(flagA);
  #pragma omp section
  prmGeneB(flagB);
   }//pragma

  /*
  auto funcgA =  std::mem_fn(&Juno::prmGeneA);
  std::thread t_geneA(funcgA, this, std::ref(flagA));
  auto funcgB =  std::mem_fn(&Juno::prmGeneB);
  std::thread t_geneB(funcgB, this, std::ref(flagB));
  t_geneA.join();
  t_geneB.join();
  */
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
  
  //if(*ChIn=='A')
    
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
  zmpP->calcWaistR(FT,  R_ref); 
 
  count++;
}

//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_  //_/_/_/_/_/_/_/_/_/_/_/_/_/_/_
   
//method
//for clip
void Juno::testMove()
{
  /*
  //V in
  object_ref->p(0)+=x;
  object_ref->p(1)+=y;
  object_ref->p(2)+=z;
  vector3 rpy(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180);
  object_ref->R = hrp::rotFromRpy(rpy);
  
  if(CalcIVK_NMS(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, stick_r_A, stick_l_A, 
	 body_B, cm_ref_B, p_ref_B, R_ref_B, FT_B, p_Init_B, R_Init_B, stick_r_B, stick_l_B, 
	stick_rpy_r_A_buf, stick_rpy_l_A_buf, stick_rpy_r_B_buf, stick_rpy_l_B_buf, object_ref)){
    if(CalcIVK_ARM_NUM_elbow(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, elbow_A)){
      for(int i=0;i<body_A->numJoints();i++)
	body_ref_A[i]=body_A->joint(i)->q;
      for(int i=0;i<body_B->numJoints();i++)
	body_ref_B[i]=body_B->joint(i)->q;   
    }
    else{
      cerr<<"err 2"<<endl;
    }
  }
  else{
    cerr<<"ivk err!"<<endl;
  }
 
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


  /*
  vector32 tema;
  vector30 temb;

  for(unsigned int i=0;i<dof_A;i++)
    tema(i)= body_A->joint(i)->q;
  
  for(unsigned int i=0;i<dof_B;i++)
    temb(i)= body_B->joint(i)->q;
  */
  

  
  //for checking optitrack
  /*
  int yaw=2;
  forceControl=0;

  while(transition_count<0)
    calcWholeIVK();//overlord to pA pB
  
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
  for(int i=0;i<240;i++)
    yawdeque.push_back(2.0);
  for(int i=0;i<480;i++)
    yawdeque.push_back(-2.0);
  for(int i=0;i<240;i++)
    yawdeque.push_back(2.0);
  
  cerr<<"test move"<<endl;
  



  /*
 for(unsigned int i=0;i<dof_A;i++)
    body_A->joint(i)->q=tema(i) ;
  
  for(unsigned int i=0;i<dof_B;i++)
    body_B->joint(i)->q= temb(i);
  */


  //body_A->calcForwardKinematics();
  //body_B->calcForwardKinematics();

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

   /*
   //for icra roll pitch
  if(roll!=rpcrr(0)||pitch!=rpcrr(1)){
    vector2 zero(0.0,0.0);
    vector2 rp_ref(roll*M_PI/180,pitch*M_PI/180);
    Interplation5(rpcrr,  zero,  zero, rp_ref,  zero,  zero, 1, rpdeque);
    rpcrr=rp_ref;
  }
   */
  stepping();
  
  /*
  //for unsymchronal lag
  bodyDeque_A.clear();
  waist_p_Deque_A.clear();
  waist_R_Deque_A.clear();
  rzmpDeque_A.clear();

  bodyDeque_B.clear();
  waist_p_Deque_B.clear();
  waist_R_Deque_B.clear();
  rzmpDeque_B.clear();

  wZMPDeque_log_A.clear();
  wZMPDeque_log_B.clear();
  */

  /*

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
  //cout<<"calculate OK"<<endl;
  
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

void Juno::setgain(double kp, double kd, double ki)
{ 
  NMS->set_kh(kp, kd, ki);
}

void Juno::set_hr(double hx, double hy, double hz)
{ 
  NMS->set_hr(hx, hy, hz);
}

void Juno::set_max_eh(double max_eh)
{ 
  NMS->set_max_eh(max_eh);
 
}

void Juno::move_hand(double x, double y, double z, double roll, double pitch, double yaw)
{
  cout<<"move hand"<<endl;

  object_ref_single->p(0)+=0.01*x;

  Matrix33 R_before(object_ref_single->R);
  Matrix33 Rt(rotationZ(deg2rad(yaw))* object_ref_single->R);
  object_ref_single->R=Rt;
  
  
  if(CalcIVK_NMS_single(body_A, cm_ref_A, p_ref_A, R_ref_A, FT_A, p_Init_A, R_Init_A, stick_r_single, stick_l_single, object_ref_single, elbow_A)){
    for(int i=0;i<body_A->numJoints();i++){
      body_ref_A[i]=body_A->joint(i)->q;
    }
    vector32 zero_A(0);  
    Interplation5(body_cur_A,  zero_A,  zero_A, body_ref_A,  zero_A,  zero_A, 1, bodyDeque_pA);
    body_cur_A=body_ref_A;
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
  NMS->logStart();
  cout<<"fcontrol is "<<forceControl<<endl;
}

void Juno::fcontrol_write()
{
  NMS->force_write();
}

void Juno::halfPos()
{
  playflag=0;

  //vector32 body_ref_A(0);
  body_ref_A =  0.0,0.0,-26.0,50.0,-24.0,0.0, 
                0.0,0.0,-26.0,50.0,-24.0,0.0,
                0.0,0.0,
                0.0,0.0,
                15.0,-10.0,0.0,-30.0, 80.0,0.0,0.0,0.0,
                15.0, 10.0,0.0,-30.0,-80.0,0.0,0.0,0.0;
  //vector30 body_ref_B(0);
  body_ref_B = 0.0,0.0,-26.0,50.0,-24.0,0.0, 
               0.0,0.0,-26.0,50.0,-24.0,0.0,
               0.0,0.0,
               0.0,0.0,
               15.0,-10.0,0.0,-30.0, 80.0,0.0,0.0,
               15.0, 10.0,0.0,-30.0,-80.0,0.0,0.0;

  //A 
  m_mc_AIn.read();
  for(unsigned int i=0;i<m_mc_A.data.length();i++){
    body_cur_A[i]=m_mc_A.data[i];
    body_ref_A[i]=deg2rad(body_ref_A[i]);
  }
  
  //B
  m_mc_BIn.read();
  for(unsigned int i=0;i<m_mc_B.data.length();i++){
    body_cur_B[i]=m_mc_B.data[i];
    body_ref_B[i]=deg2rad(body_ref_B[i]);
  }
 
  vector32 zero_A(0);
  vector30 zero_B(0);
 
  Interplation5(body_cur_A,  zero_A,  zero_A, body_ref_A,  zero_A,  zero_A, 3, bodyDeque_pA);
  Interplation5(body_cur_B,  zero_B,  zero_B, body_ref_B,  zero_B,  zero_B, 3, bodyDeque_pB);
  
  body_cur_A=body_ref_A;
  body_cur_B=body_ref_B;

  
  //trajectory is the same length
  while(!bodyDeque_pA.empty()){

    //cout<<"A"<<endl;
    for(int i=0;i<dof_A;i++){
      m_refq_A.data[i]=bodyDeque_pA.at(0)[i]; 
      //cout<< m_refq_A.data[i]<< " ";
    }
    //cout<<'\n'<<endl;
    
    //cout<<"B"<<endl;
    for(int i=0;i<dof_B;i++){
      m_refq_B.data[i]=bodyDeque_pB.at(0)[i]; 
      // cout<< m_refq_B.data[i]<< " ";
    }
    //cout<<'\n'<<endl;

    write2client();
    //cout<<"AB "<<stuck_A<<" "<<stuck_B<<endl;
    bodyDeque_pA.pop_front();
    bodyDeque_pB.pop_front();
    usleep(4000);//micro second
    //coil::sleep(coil::TimeValue(0.002)); 

  }

  cout<<"halfsit end"<<endl;


}
void Juno::exPos()
{ 
  playflag=0;

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
  body_ref_A=0.0, 0.00332796, -0.482666, 0.859412, -0.370882, -0.00322683,  0.0, 0.00332796, -0.482666, 0.859412, -0.370882, -0.00322683,  0.0,  0.0,  0.0,  0.0, 0.135465, -0.290561, 0.14261, -1.81385, 1.30413, 0.0651451, 0.202547,  0.0, 0.135465, 0.290561, -0.14261, -1.81385, -1.30413, -0.0651451, 0.202547,  0.0;

  body_ref_B= 0.0, 0.00333798, -0.493139, 0.871946, -0.378195, -0.00329024,  0.0, 0.00333798, -0.493139, 0.871946, -0.378195, -0.00329024,  0.0,  0.0,  0.0,  0.0, 0.085065, -0.485823, 0.315933, -1.82036, 1.14179, 0.391601,  0.0, 0.085065, 0.485823, -0.315933, -1.82036, -1.14179, 0.391601,  0.0;

  Interplation5(body_cur_A,  zero_A,  zero_A, body_ref_A,  zero_A,  zero_A, 3, bodyDeque_pA);
  Interplation5(body_cur_B,  zero_B,  zero_B, body_ref_B,  zero_B,  zero_B, 3, bodyDeque_pB);
  
  body_cur_A=body_ref_A;
  body_cur_B=body_ref_B;

  
  //trajectory is the same length
  while(!bodyDeque_pA.empty()){

    //cout<<"A"<<endl;
    for(int i=0;i<dof_A;i++){
      m_refq_A.data[i]=bodyDeque_pA.at(0)[i];
      //cout<< m_refq_A.data[i]<< " ";
    }
    //cout<<'\n'<<endl;

    //cout<<"B"<<endl;
    for(int i=0;i<dof_B;i++){
      m_refq_B.data[i]=bodyDeque_pB.at(0)[i]; 
      //cout<< m_refq_B.data[i]<< " ";
    }  
    //cout<<'\n'<<endl;
    
    //if(!m_refq_AOut.write())
    //  cout<<"no reach A"<<endl;
    //if(!m_refq_BOut.write())
    //  cout<<"no reach B"<<endl;
    
    write2client();//wait until receive
    
    //cout<<"AB "<<stuck_A<<" "<<stuck_B<<endl;
    bodyDeque_pA.pop_front();
    bodyDeque_pB.pop_front();
    usleep(4000);//micro second
    
    //coil::sleep(coil::TimeValue(0.002)); 
  }
  cout<<"expos end"<<endl;
 
  /*
  vector32 zero_A(0);
  body_ref_A=0, 0.00332796, -0.482666, 0.859412, -0.370882, -0.00322683,  0, 0.00332796, -0.482666, 0.859412, -0.370882, -0.00322683,  0,  0,  0,  0, 0.135465, -0.290561, 0.14261, -1.81385, 1.30413, 0.0651451, 0.202547,  0, 0.135465, 0.290561, -0.14261, -1.81385, -1.30413, -0.0651451, 0.202547,  0;

  for(int i=0;i<dof_A;i++)
    m_refq_A.data[i]=body_ref_A(i); 

  server->send2client(m_refq_A);
*/
   
}
void Juno::hogex()//for playstart in offline
{

}

void Juno::clear_log()
{
  forceControl=0;

  NMS->hcTem_deque.clear();
  NMS->logStop();
  cmDeque_log_A.clear();
  cmDeque_log_B.clear();
  absZMPDeque_log_A.clear();
  absZMPDeque_log_B.clear();
  object_p_Deque_log.clear();
  wZMPDeque_log_A.clear();
  wZMPDeque_log_B.clear();

  cout<<"clear log"<<endl;
}

void Juno::save_log()
{
  playflag=0;

  //stop log
  NMS->logStop();
  int timmer=0;
  
  /*
  std::ofstream ofs_hctem("/home/grxuser/users/wu/junoRH/hcTem.log");
  while(!NMS->hcTem_deque.empty()){
    for(int i=0;i<24;i++)
      ofs_hctem<<NMS->hcTem_deque.at(0)[i]<<" ";

    for(int i=0;i<24;i++)
      ofs_hctem<<NMS->hc_deque.at(0)[i]<<" ";
    
    ofs_hctem<<timmer*0.005<<" "<<endl;
    timmer++;
    NMS->hcTem_deque.pop_front();
    NMS->hc_deque.pop_front();
  }
  timmer=0;

  
  std::ofstream ofs_cm("/home/grxuser/users/wu/junoRH/obj.log");
  while(!object_p_Deque_log.empty()){
    for(int i=0;i<2;i++)
      ofs_cm<<object_p_Deque_log.at(0)[i]<<" ";
   
    ofs_cm<<timmer*0.005<<" "<<endl;
    timmer++;
 
    object_p_Deque_log.pop_front();
    
  }
  timmer=0;

  std::ofstream ofs_wzmpA("/home/grxuser/users/wu/junoRH/wzmpA.log");
  while(!wZMPDeque_log_A.empty()){
    for(int i=0;i<2;i++)
      ofs_wzmpA<<wZMPDeque_log_A.at(0)[i]<<" ";
    for(int i=0;i<2;i++)
      ofs_wzmpA<<absZMPDeque_log_A.at(0)[i]<<" ";
    for(int i=0;i<2;i++)
      ofs_wzmpA<<cmDeque_log_A.at(0)[i]<<" ";

    ofs_wzmpA<<timmer*0.005<<" "<<endl;
    timmer++;
   
    wZMPDeque_log_A.pop_front();
    absZMPDeque_log_A.pop_front();
    cmDeque_log_A.pop_front();
  }
  timmer=0;

  std::ofstream ofs_wzmpB("/home/grxuser/users/wu/junoRH/wzmpB.log");
  while(!wZMPDeque_log_B.empty()){
    for(int i=0;i<2;i++)
      ofs_wzmpB<<wZMPDeque_log_B.at(0)[i]<<" ";
    for(int i=0;i<2;i++)
      ofs_wzmpB<<absZMPDeque_log_B.at(0)[i]<<" ";
    for(int i=0;i<2;i++)
      ofs_wzmpB<<cmDeque_log_B.at(0)[i]<<" ";

    ofs_wzmpB<<timmer*0.005<<" "<<endl;
    timmer++;
   
    wZMPDeque_log_B.pop_front();
    absZMPDeque_log_B.pop_front();
    cmDeque_log_B.pop_front();
  }
  timmer=0;


  clear_log();
*/

  /*//unused
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

 cout<<"log saved"<<endl;
}

void Juno::clip_open_A()
{
  //m_mc_AIn.read();
  for(unsigned int i=0;i<m_mc_A.data.length();i++)
    body_cur_A[i]=m_mc_A.data[i];
    //body_cur_A[i]=body_A->joint(i)->q;//m_mc_A.data[i];
 
  vector32 zero_A(0);
 
  body_ref_A= body_cur_A;
  body_ref_A[23]=deg2rad(48);
  body_ref_A[31]=deg2rad(-48);

  Interplation5(body_cur_A,  zero_A,  zero_A, body_ref_A,  zero_A,  zero_A, 2, bodyDeque_pA);
  body_cur_A=body_ref_A;
  playflag=0;
  bodyDeque_A.clear();

  cout<<"open A"<<endl;
 
}
void Juno::clip_open_B()
{
  //m_mc_BIn.read();
  for(unsigned int i=0;i<m_mc_B.data.length();i++)
    body_cur_B[i]=m_mc_B.data[i];
  
  vector30 zero_B(0);
 
  body_ref_B= body_cur_B;
  body_ref_B[22]=deg2rad(-46);
  body_ref_B[29]=deg2rad(-46);

  Interplation5(body_cur_B,  zero_B,  zero_B, body_ref_B,  zero_B,  zero_B, 2, bodyDeque_pB);
  body_cur_B=body_ref_B;
  playflag=0;
  bodyDeque_B.clear();

  cout<<"open B"<<endl;
}
void Juno::clip_close_A()
{
  //m_mc_AIn.read();
  for(unsigned int i=0;i<m_mc_A.data.length();i++)
    body_cur_A[i]=m_mc_A.data[i];
  
  vector32 zero_A(0);
  
  body_ref_A= body_cur_A;
  body_ref_A[23]=deg2rad(2);
  body_ref_A[31]=deg2rad(-2);
  
  Interplation5(body_cur_A,  zero_A,  zero_A, body_ref_A,  zero_A,  zero_A, 2, bodyDeque_pA);
  body_cur_A=body_ref_A;
  playflag=0;
  bodyDeque_A.clear();

  cout<<"close A"<<endl;
}
void Juno::clip_close_B()
{
  //m_mc_BIn.read();
  for(unsigned int i=0;i<m_mc_B.data.length();i++)
    body_cur_B[i]=m_mc_B.data[i];
  
  vector30 zero_B(0);
 
  body_ref_B= body_cur_B;
  body_ref_B[22]=deg2rad(-19.8);
  body_ref_B[29]=deg2rad(-19.8);

  Interplation5(body_cur_B,  zero_B,  zero_B, body_ref_B,  zero_B,  zero_B, 2, bodyDeque_pB);
  body_cur_B=body_ref_B;
  playflag=0;
  bodyDeque_B.clear();

  cout<<"close B"<<endl;
}
void Juno::stepping()
{
  /* //testcheck
  hrp::Link* c_object;
  c_object=new Link();
  dvector a_delta_p_r=dzerovector(18);
  updateObject(body_A, stick_r_A, stick_l_A, body_B, stick_r_B, stick_l_B,  stick_rpy_r_A_buf, stick_rpy_l_A_buf, stick_rpy_r_B_buf, stick_rpy_l_B_buf, c_object, a_delta_p_r);//stick p R recalculated

  //cout<<"stick r jun "<<'\n'<<stick_r_B->p<<endl;
  //cout<< c_object->R<<endl; //no equal
  //cout<<stick_r_B->R<<endl;

  dmatrix Jacobian=dzeromatrix(24,dof_A+dof_B);
  Jacobian = calcQuadJacobian_upper( body_A, body_A->link("LLEG_JOINT5"), stick_r_A, stick_l_A, body_B, body_B->link("LLEG_JOINT5"), stick_r_B, stick_l_B, c_object);

  //cout<<Jacobian(6,16)<<endl;

  NMS->controlRoutine(FT_A, m_rhsensor_A, m_lhsensor_A, m_mc_A, body_A, 
		      FT_B, m_rhsensor_B, m_lhsensor_B, m_mc_B, body_B,  
			      R_ref_A[WAIST],R_ref_B[WAIST]);
  */
  //for unsynchron
  cout<<"stepping"<<endl;
  object_p_Deque_log.clear();
  //bodyDeque_A.clear();
  waist_p_Deque_A.clear();//not need
  waist_R_Deque_A.clear();//not need
  rzmpDeque_A.clear();

  //bodyDeque_B.clear();
  waist_p_Deque_B.clear();//not need
  waist_R_Deque_B.clear();//not need
  rzmpDeque_B.clear();
  //log clear()
  wZMPDeque_log_A.clear();
  wZMPDeque_log_B.clear();
  absZMPDeque_log_A.clear();
  absZMPDeque_log_B.clear();
  cmDeque_log_A.clear();
  cmDeque_log_B.clear();

  NMS->hcTem_deque.clear();//new

  step_num=0;
  step=!step;
    
}

void Juno::setMaker(double RARM_x_A , double RARM_y_A, double LARM_x_A, double LARM_y_A,
	    double RARM_x_B , double RARM_y_B, double LARM_x_B, double LARM_y_B)
{

  if(!preset_flag){
    preSet();
  }

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

  /*
  for(int i=0;i<dof_A;i++)
    m_mc_A.data[i]= body_ref_A(i);
  for(int i=0;i<dof_B;i++)
    m_mc_B.data[i]= body_ref_B(i);
  setModelPosture(body_A, m_mc_A, FT_A, p_Init_A, R_Init_A);
  setModelPosture(body_B, m_mc_B, FT_B, p_Init_B, R_Init_B);
  */

  RenewModel(body_A, p_ref_A, R_ref_A);
  RenewModel(body_B, p_ref_B, R_ref_B);
  R_ref_A[WAIST]=tvmet::identity<hrp::Matrix33>();
  R_ref_B[WAIST]=rotationZ(M_PI);
  
  cm_ref_A=body_A->calcCM();// 
  cm_ref_B=body_B->calcCM();
  getElbowAngleARM(body_A, elbow_A);
  getElbowAngleARM(body_B, elbow_B);

  //for expos adjust hrp2020
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

  preset_flag=1;
}

void Juno::start()
{  
  if(!preset_flag){
    preSet();
  }
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
 
  vector32 temp_q_A;
  vector30 temp_q_B;

  //renew body in NMS
  for(unsigned int i=0;i<m_mc_A.data.length();i++)
    m_mc_A.data[i]=temp_q_A(i)=body_A->joint(i)->q;
  for(unsigned int i=0;i<m_mc_B.data.length();i++)
    m_mc_B.data[i]=temp_q_B(i)=body_B->joint(i)->q;

  add_A(temp_q_A);
  add_B(temp_q_B);
 
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

  //normal
  if(!optcap){
    R_LEG_ini_A=  LEG_ref_R_A= tvmet::identity<matrix33>();
    R_LEG_ini_B=  LEG_ref_R_B= rotationZ(M_PI);//180 rotate
  }
  //////////////////////////////////////////////////////////////

  rotRTemp=object_ref->R;
  //cerr<<"startQ"<<endl;

  zmpHandler();
  relZMP_A = trans(R_ref_A[WAIST])*(absZMP_A - body_A->link("WAIST")->p);
  relZMP_B = trans(R_ref_B[WAIST])*(absZMP_B - body_B->link("WAIST")->p);
  for(int i=0;i< m_rzmp_A.data.length();i++){
    m_rzmp_A.data[i]=relZMP_A[i];  
    m_rzmp_B.data[i]=relZMP_B[i]; 
  }
  
  m_rzmp_AOut.write();
  m_rzmp_BOut.write();
  
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


  gettimeofday(&tv_s,0);
  gettimeofday(&tv_As,0);
  gettimeofday(&tv_Bs,0);

  //onoff 
  playflag=1;//comment out in RH experiment for off line recording

  auto func =  std::mem_fn(&Juno::calcNextTrajectory);
  std::thread ts_calcNextTra(func, this);
  ts_calcNextTra.detach();
}//start()

void Juno::stop()//emergicy stop
{
  cout<<"average "<<temptotal/tempcount<<" "<<tempcount<<" "<<temptotal<<endl;
  /*
  CommandIn_A=5;//stop to walk>>quick stop
  zmpP_A->StopZMP(FT_A, rfzmp_A, count_A);
  
  CommandIn_B=5;//stop to walk>>quick stop
  zmpP_B->StopZMP(FT_B, rfzmp_B, count_B);
  step=0;
  step_num=0;
  velobj= 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  */
  cout<<"stop"<<endl;
}

void Juno::setWalkCommand(const char* ChIn)
{ 
 
  
}


RTC::ReturnCode_t Juno::onFinalize()
{
  flag_t=0;
  //pthread_join(tmain, NULL);

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
