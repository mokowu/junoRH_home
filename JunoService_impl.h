// -*-C++-*-
/*!
 * @file  JunoService_impl.h
 * @brief Service implementation header of JunoService.idl
 *
 */

#include "JunoService.hh"


#ifndef JUNOSERVICE_IMPL_H
#define JUNOSERVICE_IMPL_H
 
/*
 * Example class implementing IDL interface OpenHRP::JunoService
 */
class Juno;

class JunoService_impl
 : public virtual POA_OpenHRP::JunoService,
   public virtual PortableServer::RefCountServantBase
{
 private:
   // Make sure all instances are built on the heap by making the
   // destructor non-public
   //virtual ~JunoService_impl();
  Juno * m_comp;

 public:
   // standard constructor
   JunoService_impl();
   virtual ~JunoService_impl();

  void setComponent (Juno * i_comp) {
    m_comp = i_comp;
  }

  // attributes and operations
  void setObjectV(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z, ::CORBA::Double roll, ::CORBA::Double pitch, ::CORBA::Double yaw);
  void move_hand(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z, ::CORBA::Double roll, ::CORBA::Double pitch, ::CORBA::Double yaw);
  void setMaker(::CORBA::Double RARM_x_A , ::CORBA::Double RARM_y_A, ::CORBA::Double LARM_x_A, ::CORBA::Double LARM_y_A, ::CORBA::Double RARM_x_B , ::CORBA::Double RARM_y_B, ::CORBA::Double LARM_x_B, ::CORBA::Double LARM_y_B);
  void setgain(::CORBA::Double kp, ::CORBA::Double kd, ::CORBA::Double ki);
  void set_hr(::CORBA::Double hx, ::CORBA::Double hy, ::CORBA::Double hz);
  void set_max_eh(::CORBA::Double max_eh);
  void setWalkCommand(const char* ChIn);
  void preSet();
  void start();
  void testMove();
  void fcontrol();
  void fcontrol_write();
  void exPos();
  void halfPos();
  void hogex();
  void clear_log();
  void save_log();
  void clip_open_A();
  void clip_open_B();
  void clip_close_A();
  void clip_close_B();
  void stepping();
  void stop();

};



#endif // JUNOSERVICE_IMPL_H


