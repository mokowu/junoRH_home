// -*-C++-*-
/*!
 * @file  JunoService_impl.cpp
 * @brief Service implementation code of JunoService.idl
 *
 */

#include "JunoService_impl.h"
#include "Juno.h"
/*
 * Example implementational code for IDL interface OpenHRP::JunoService
 */
JunoService_impl::JunoService_impl()
{
  // Please add extra constructor code here.
}


JunoService_impl::~JunoService_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
void JunoService_impl::setObjectV(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z, ::CORBA::Double roll, ::CORBA::Double pitch, ::CORBA::Double yaw)
{
  m_comp->setObjectV(x, y, z, roll, pitch, yaw);
}

void JunoService_impl::move_hand(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z, ::CORBA::Double roll, ::CORBA::Double pitch, ::CORBA::Double yaw)
{
  m_comp->move_hand(x, y, z, roll, pitch, yaw);
}

void JunoService_impl::setMaker(::CORBA::Double RARM_x_A , ::CORBA::Double RARM_y_A, ::CORBA::Double LARM_x_A, ::CORBA::Double LARM_y_A, ::CORBA::Double RARM_x_B , ::CORBA::Double RARM_y_B, ::CORBA::Double LARM_x_B, ::CORBA::Double LARM_y_B)
{
  m_comp->setMaker(RARM_x_A , RARM_y_A, LARM_x_A, LARM_y_A, RARM_x_B, RARM_y_B, LARM_x_B, LARM_y_B);
}

void JunoService_impl::setWalkCommand(const char* ChIn)
{
  m_comp->setWalkCommand(ChIn);
  
}

void JunoService_impl::setgain(::CORBA::Double kp, ::CORBA::Double kd, ::CORBA::Double ki)
{
  m_comp->setgain(kp, kd, ki);
}

void JunoService_impl::set_hr(::CORBA::Double hx, ::CORBA::Double hy, ::CORBA::Double hz)
{
  m_comp->set_hr(hx, hy, hz);
}

void JunoService_impl::set_max_eh(::CORBA::Double max_eh)
{
  m_comp->set_max_eh(max_eh);
}

void JunoService_impl::preSet()
{
  m_comp->preSet();
}


void JunoService_impl::start()
{
  m_comp->start();
}

void JunoService_impl::testMove()
{
  m_comp->testMove();
}

void JunoService_impl::fcontrol()
{
  m_comp->fcontrol();
}

void JunoService_impl::fcontrol_write()
{
  m_comp->fcontrol_write();
}

void JunoService_impl::exPos()
{
  m_comp->exPos();
}
void JunoService_impl::halfPos()
{
  m_comp->halfPos();
}


void JunoService_impl::hogex()
{
  m_comp->hogex();
}

void JunoService_impl::clear_log()
{
  m_comp->clear_log();
}

void JunoService_impl::save_log()
{
  m_comp->save_log();
}

void JunoService_impl::clip_open_A()
{
  m_comp->clip_open_A();
}

void JunoService_impl::clip_open_B()
{
  m_comp->clip_open_B();
}

void JunoService_impl::clip_close_A()
{
  m_comp->clip_close_A();
}

void JunoService_impl::clip_close_B()
{
  m_comp->clip_close_B();
}



void JunoService_impl::stepping()
{
  m_comp->stepping();
}

void JunoService_impl::stop()
{
  m_comp->stop();
}



// End of example implementational code



