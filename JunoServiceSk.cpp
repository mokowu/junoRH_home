// This file is generated by omniidl (C++ backend)- omniORB_4_1. Do not edit.

#include "JunoService.hh"
#include <omniORB4/IOP_S.h>
#include <omniORB4/IOP_C.h>
#include <omniORB4/callDescriptor.h>
#include <omniORB4/callHandle.h>
#include <omniORB4/objTracker.h>


OMNI_USING_NAMESPACE(omni)

static const char* _0RL_library_version = omniORB_4_1;



OpenHRP::JunoService_ptr OpenHRP::JunoService_Helper::_nil() {
  return ::OpenHRP::JunoService::_nil();
}

::CORBA::Boolean OpenHRP::JunoService_Helper::is_nil(::OpenHRP::JunoService_ptr p) {
  return ::CORBA::is_nil(p);

}

void OpenHRP::JunoService_Helper::release(::OpenHRP::JunoService_ptr p) {
  ::CORBA::release(p);
}

void OpenHRP::JunoService_Helper::marshalObjRef(::OpenHRP::JunoService_ptr obj, cdrStream& s) {
  ::OpenHRP::JunoService::_marshalObjRef(obj, s);
}

OpenHRP::JunoService_ptr OpenHRP::JunoService_Helper::unmarshalObjRef(cdrStream& s) {
  return ::OpenHRP::JunoService::_unmarshalObjRef(s);
}

void OpenHRP::JunoService_Helper::duplicate(::OpenHRP::JunoService_ptr obj) {
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
}

OpenHRP::JunoService_ptr
OpenHRP::JunoService::_duplicate(::OpenHRP::JunoService_ptr obj)
{
  if( obj && !obj->_NP_is_nil() )  omni::duplicateObjRef(obj);
  return obj;
}

OpenHRP::JunoService_ptr
OpenHRP::JunoService::_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_realNarrow(_PD_repoId);
  return e ? e : _nil();
}


OpenHRP::JunoService_ptr
OpenHRP::JunoService::_unchecked_narrow(::CORBA::Object_ptr obj)
{
  if( !obj || obj->_NP_is_nil() || obj->_NP_is_pseudo() ) return _nil();
  _ptr_type e = (_ptr_type) obj->_PR_getobj()->_uncheckedNarrow(_PD_repoId);
  return e ? e : _nil();
}

OpenHRP::JunoService_ptr
OpenHRP::JunoService::_nil()
{
#ifdef OMNI_UNLOADABLE_STUBS
  static _objref_JunoService _the_nil_obj;
  return &_the_nil_obj;
#else
  static _objref_JunoService* _the_nil_ptr = 0;
  if( !_the_nil_ptr ) {
    omni::nilRefLock().lock();
    if( !_the_nil_ptr ) {
      _the_nil_ptr = new _objref_JunoService;
      registerNilCorbaObject(_the_nil_ptr);
    }
    omni::nilRefLock().unlock();
  }
  return _the_nil_ptr;
#endif
}

const char* OpenHRP::JunoService::_PD_repoId = "IDL:OpenHRP/JunoService:1.0";


OpenHRP::_objref_JunoService::~_objref_JunoService() {
  
}


OpenHRP::_objref_JunoService::_objref_JunoService(omniIOR* ior, omniIdentity* id) :
   omniObjRef(::OpenHRP::JunoService::_PD_repoId, ior, id, 1)
   
   
{
  _PR_setobj(this);
}

void*
OpenHRP::_objref_JunoService::_ptrToObjRef(const char* id)
{
  if( id == ::OpenHRP::JunoService::_PD_repoId )
    return (::OpenHRP::JunoService_ptr) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (::CORBA::Object_ptr) this;

  if( omni::strMatch(id, ::OpenHRP::JunoService::_PD_repoId) )
    return (::OpenHRP::JunoService_ptr) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (::CORBA::Object_ptr) this;

  return 0;
}

// Proxy call descriptor class. Mangled signature:
//  void_i_cdouble_i_cdouble_i_cdouble_i_cdouble_i_cdouble_i_cdouble
class _0RL_cd_a6d83b30232ce860_00000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_a6d83b30232ce860_00000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  void marshalArguments(cdrStream&);
  void unmarshalArguments(cdrStream&);

    
  
  static const char* const _user_exns[];

  ::CORBA::Double arg_0;
  ::CORBA::Double arg_1;
  ::CORBA::Double arg_2;
  ::CORBA::Double arg_3;
  ::CORBA::Double arg_4;
  ::CORBA::Double arg_5;
};

void _0RL_cd_a6d83b30232ce860_00000000::marshalArguments(cdrStream& _n)
{
  arg_0 >>= _n;
  arg_1 >>= _n;
  arg_2 >>= _n;
  arg_3 >>= _n;
  arg_4 >>= _n;
  arg_5 >>= _n;

}

void _0RL_cd_a6d83b30232ce860_00000000::unmarshalArguments(cdrStream& _n)
{
  (::CORBA::Double&)arg_0 <<= _n;
  (::CORBA::Double&)arg_1 <<= _n;
  (::CORBA::Double&)arg_2 <<= _n;
  (::CORBA::Double&)arg_3 <<= _n;
  (::CORBA::Double&)arg_4 <<= _n;
  (::CORBA::Double&)arg_5 <<= _n;

}

const char* const _0RL_cd_a6d83b30232ce860_00000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_10000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_a6d83b30232ce860_00000000* tcd = (_0RL_cd_a6d83b30232ce860_00000000*)cd;
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->setObjectV(tcd->arg_0, tcd->arg_1, tcd->arg_2, tcd->arg_3, tcd->arg_4, tcd->arg_5);


}

void OpenHRP::_objref_JunoService::setObjectV(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z, ::CORBA::Double roll, ::CORBA::Double pitch, ::CORBA::Double yaw)
{
  _0RL_cd_a6d83b30232ce860_00000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_10000000, "setObjectV", 11);
  _call_desc.arg_0 = x;
  _call_desc.arg_1 = y;
  _call_desc.arg_2 = z;
  _call_desc.arg_3 = roll;
  _call_desc.arg_4 = pitch;
  _call_desc.arg_5 = yaw;

  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_20000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_a6d83b30232ce860_00000000* tcd = (_0RL_cd_a6d83b30232ce860_00000000*)cd;
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->move_hand(tcd->arg_0, tcd->arg_1, tcd->arg_2, tcd->arg_3, tcd->arg_4, tcd->arg_5);


}

void OpenHRP::_objref_JunoService::move_hand(::CORBA::Double x, ::CORBA::Double y, ::CORBA::Double z, ::CORBA::Double roll, ::CORBA::Double pitch, ::CORBA::Double yaw)
{
  _0RL_cd_a6d83b30232ce860_00000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_20000000, "move_hand", 10);
  _call_desc.arg_0 = x;
  _call_desc.arg_1 = y;
  _call_desc.arg_2 = z;
  _call_desc.arg_3 = roll;
  _call_desc.arg_4 = pitch;
  _call_desc.arg_5 = yaw;

  _invoke(_call_desc);



}
// Proxy call descriptor class. Mangled signature:
//  void_i_cdouble_i_cdouble_i_cdouble_i_cdouble_i_cdouble_i_cdouble_i_cdouble_i_cdouble
class _0RL_cd_a6d83b30232ce860_30000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_a6d83b30232ce860_30000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  void marshalArguments(cdrStream&);
  void unmarshalArguments(cdrStream&);

    
  
  static const char* const _user_exns[];

  ::CORBA::Double arg_0;
  ::CORBA::Double arg_1;
  ::CORBA::Double arg_2;
  ::CORBA::Double arg_3;
  ::CORBA::Double arg_4;
  ::CORBA::Double arg_5;
  ::CORBA::Double arg_6;
  ::CORBA::Double arg_7;
};

void _0RL_cd_a6d83b30232ce860_30000000::marshalArguments(cdrStream& _n)
{
  arg_0 >>= _n;
  arg_1 >>= _n;
  arg_2 >>= _n;
  arg_3 >>= _n;
  arg_4 >>= _n;
  arg_5 >>= _n;
  arg_6 >>= _n;
  arg_7 >>= _n;

}

void _0RL_cd_a6d83b30232ce860_30000000::unmarshalArguments(cdrStream& _n)
{
  (::CORBA::Double&)arg_0 <<= _n;
  (::CORBA::Double&)arg_1 <<= _n;
  (::CORBA::Double&)arg_2 <<= _n;
  (::CORBA::Double&)arg_3 <<= _n;
  (::CORBA::Double&)arg_4 <<= _n;
  (::CORBA::Double&)arg_5 <<= _n;
  (::CORBA::Double&)arg_6 <<= _n;
  (::CORBA::Double&)arg_7 <<= _n;

}

const char* const _0RL_cd_a6d83b30232ce860_30000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_40000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_a6d83b30232ce860_30000000* tcd = (_0RL_cd_a6d83b30232ce860_30000000*)cd;
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->setMaker(tcd->arg_0, tcd->arg_1, tcd->arg_2, tcd->arg_3, tcd->arg_4, tcd->arg_5, tcd->arg_6, tcd->arg_7);


}

void OpenHRP::_objref_JunoService::setMaker(::CORBA::Double RARM_x_A, ::CORBA::Double RARM_y_A, ::CORBA::Double LARM_x_A, ::CORBA::Double LARM_y_A, ::CORBA::Double RARM_x_B, ::CORBA::Double RARM_y_B, ::CORBA::Double LARM_x_B, ::CORBA::Double LARM_y_B)
{
  _0RL_cd_a6d83b30232ce860_30000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_40000000, "setMaker", 9);
  _call_desc.arg_0 = RARM_x_A;
  _call_desc.arg_1 = RARM_y_A;
  _call_desc.arg_2 = LARM_x_A;
  _call_desc.arg_3 = LARM_y_A;
  _call_desc.arg_4 = RARM_x_B;
  _call_desc.arg_5 = RARM_y_B;
  _call_desc.arg_6 = LARM_x_B;
  _call_desc.arg_7 = LARM_y_B;

  _invoke(_call_desc);



}
// Proxy call descriptor class. Mangled signature:
//  void_i_cdouble_i_cdouble_i_cdouble
class _0RL_cd_a6d83b30232ce860_50000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_a6d83b30232ce860_50000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  void marshalArguments(cdrStream&);
  void unmarshalArguments(cdrStream&);

    
  
  static const char* const _user_exns[];

  ::CORBA::Double arg_0;
  ::CORBA::Double arg_1;
  ::CORBA::Double arg_2;
};

void _0RL_cd_a6d83b30232ce860_50000000::marshalArguments(cdrStream& _n)
{
  arg_0 >>= _n;
  arg_1 >>= _n;
  arg_2 >>= _n;

}

void _0RL_cd_a6d83b30232ce860_50000000::unmarshalArguments(cdrStream& _n)
{
  (::CORBA::Double&)arg_0 <<= _n;
  (::CORBA::Double&)arg_1 <<= _n;
  (::CORBA::Double&)arg_2 <<= _n;

}

const char* const _0RL_cd_a6d83b30232ce860_50000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_60000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_a6d83b30232ce860_50000000* tcd = (_0RL_cd_a6d83b30232ce860_50000000*)cd;
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->setgain(tcd->arg_0, tcd->arg_1, tcd->arg_2);


}

void OpenHRP::_objref_JunoService::setgain(::CORBA::Double kp, ::CORBA::Double kd, ::CORBA::Double ki)
{
  _0RL_cd_a6d83b30232ce860_50000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_60000000, "setgain", 8);
  _call_desc.arg_0 = kp;
  _call_desc.arg_1 = kd;
  _call_desc.arg_2 = ki;

  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_70000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_a6d83b30232ce860_50000000* tcd = (_0RL_cd_a6d83b30232ce860_50000000*)cd;
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->set_hr(tcd->arg_0, tcd->arg_1, tcd->arg_2);


}

void OpenHRP::_objref_JunoService::set_hr(::CORBA::Double hx, ::CORBA::Double hy, ::CORBA::Double hz)
{
  _0RL_cd_a6d83b30232ce860_50000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_70000000, "set_hr", 7);
  _call_desc.arg_0 = hx;
  _call_desc.arg_1 = hy;
  _call_desc.arg_2 = hz;

  _invoke(_call_desc);



}
// Proxy call descriptor class. Mangled signature:
//  void_i_cdouble
class _0RL_cd_a6d83b30232ce860_80000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_a6d83b30232ce860_80000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  void marshalArguments(cdrStream&);
  void unmarshalArguments(cdrStream&);

    
  
  static const char* const _user_exns[];

  ::CORBA::Double arg_0;
};

void _0RL_cd_a6d83b30232ce860_80000000::marshalArguments(cdrStream& _n)
{
  arg_0 >>= _n;

}

void _0RL_cd_a6d83b30232ce860_80000000::unmarshalArguments(cdrStream& _n)
{
  (::CORBA::Double&)arg_0 <<= _n;

}

const char* const _0RL_cd_a6d83b30232ce860_80000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_90000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_a6d83b30232ce860_80000000* tcd = (_0RL_cd_a6d83b30232ce860_80000000*)cd;
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->set_max_eh(tcd->arg_0);


}

void OpenHRP::_objref_JunoService::set_max_eh(::CORBA::Double max_eh)
{
  _0RL_cd_a6d83b30232ce860_80000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_90000000, "set_max_eh", 11);
  _call_desc.arg_0 = max_eh;

  _invoke(_call_desc);



}
// Proxy call descriptor class. Mangled signature:
//  void_i_cstring
class _0RL_cd_a6d83b30232ce860_a0000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_a6d83b30232ce860_a0000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  void marshalArguments(cdrStream&);
  void unmarshalArguments(cdrStream&);

    
  
  static const char* const _user_exns[];

  ::CORBA::String_var arg_0_;
  const char* arg_0;
};

void _0RL_cd_a6d83b30232ce860_a0000000::marshalArguments(cdrStream& _n)
{
  _n.marshalString(arg_0,0);

}

void _0RL_cd_a6d83b30232ce860_a0000000::unmarshalArguments(cdrStream& _n)
{
  arg_0_ = _n.unmarshalString(0);
  arg_0 = arg_0_.in();

}

const char* const _0RL_cd_a6d83b30232ce860_a0000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_b0000000(omniCallDescriptor* cd, omniServant* svnt)
{
  _0RL_cd_a6d83b30232ce860_a0000000* tcd = (_0RL_cd_a6d83b30232ce860_a0000000*)cd;
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->setWalkCommand(tcd->arg_0);


}

void OpenHRP::_objref_JunoService::setWalkCommand(const char* ChIn)
{
  _0RL_cd_a6d83b30232ce860_a0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_b0000000, "setWalkCommand", 15);
  _call_desc.arg_0 = ChIn;

  _invoke(_call_desc);



}
// Proxy call descriptor class. Mangled signature:
//  void
class _0RL_cd_a6d83b30232ce860_c0000000
  : public omniCallDescriptor
{
public:
  inline _0RL_cd_a6d83b30232ce860_c0000000(LocalCallFn lcfn,const char* op_,size_t oplen,_CORBA_Boolean upcall=0):
     omniCallDescriptor(lcfn, op_, oplen, 0, _user_exns, 0, upcall)
  {
    
  }
  
  
    
  
  static const char* const _user_exns[];

  
};

const char* const _0RL_cd_a6d83b30232ce860_c0000000::_user_exns[] = {
  0
};

// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_d0000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->preSet();


}

void OpenHRP::_objref_JunoService::preSet()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_d0000000, "preSet", 7);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_e0000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->start();


}

void OpenHRP::_objref_JunoService::start()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_e0000000, "start", 6);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_f0000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->testMove();


}

void OpenHRP::_objref_JunoService::testMove()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_f0000000, "testMove", 9);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_01000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->fcontrol();


}

void OpenHRP::_objref_JunoService::fcontrol()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_01000000, "fcontrol", 9);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_11000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->fcontrol_write();


}

void OpenHRP::_objref_JunoService::fcontrol_write()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_11000000, "fcontrol_write", 15);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_21000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->exPos();


}

void OpenHRP::_objref_JunoService::exPos()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_21000000, "exPos", 6);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_31000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->halfPos();


}

void OpenHRP::_objref_JunoService::halfPos()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_31000000, "halfPos", 8);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_41000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->hogex();


}

void OpenHRP::_objref_JunoService::hogex()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_41000000, "hogex", 6);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_51000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->clear_log();


}

void OpenHRP::_objref_JunoService::clear_log()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_51000000, "clear_log", 10);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_61000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->clip_open_A();


}

void OpenHRP::_objref_JunoService::clip_open_A()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_61000000, "clip_open_A", 12);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_71000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->clip_open_B();


}

void OpenHRP::_objref_JunoService::clip_open_B()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_71000000, "clip_open_B", 12);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_81000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->clip_close_A();


}

void OpenHRP::_objref_JunoService::clip_close_A()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_81000000, "clip_close_A", 13);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_91000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->clip_close_B();


}

void OpenHRP::_objref_JunoService::clip_close_B()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_91000000, "clip_close_B", 13);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_a1000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->save_log();


}

void OpenHRP::_objref_JunoService::save_log()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_a1000000, "save_log", 9);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_b1000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->stop();


}

void OpenHRP::_objref_JunoService::stop()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_b1000000, "stop", 5);


  _invoke(_call_desc);



}
// Local call call-back function.
static void
_0RL_lcfn_a6d83b30232ce860_c1000000(omniCallDescriptor*, omniServant* svnt)
{
  
  OpenHRP::_impl_JunoService* impl = (OpenHRP::_impl_JunoService*) svnt->_ptrToInterface(OpenHRP::JunoService::_PD_repoId);
  impl->stepping();


}

void OpenHRP::_objref_JunoService::stepping()
{
  _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_c1000000, "stepping", 9);


  _invoke(_call_desc);



}
OpenHRP::_pof_JunoService::~_pof_JunoService() {}


omniObjRef*
OpenHRP::_pof_JunoService::newObjRef(omniIOR* ior, omniIdentity* id)
{
  return new ::OpenHRP::_objref_JunoService(ior, id);
}


::CORBA::Boolean
OpenHRP::_pof_JunoService::is_a(const char* id) const
{
  if( omni::ptrStrMatch(id, ::OpenHRP::JunoService::_PD_repoId) )
    return 1;
  
  return 0;
}

const OpenHRP::_pof_JunoService _the_pof_OpenHRP_mJunoService;

OpenHRP::_impl_JunoService::~_impl_JunoService() {}


::CORBA::Boolean
OpenHRP::_impl_JunoService::_dispatch(omniCallHandle& _handle)
{
  const char* op = _handle.operation_name();

  if( omni::strMatch(op, "setObjectV") ) {

    _0RL_cd_a6d83b30232ce860_00000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_10000000, "setObjectV", 11, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "move_hand") ) {

    _0RL_cd_a6d83b30232ce860_00000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_20000000, "move_hand", 10, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "setMaker") ) {

    _0RL_cd_a6d83b30232ce860_30000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_40000000, "setMaker", 9, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "setgain") ) {

    _0RL_cd_a6d83b30232ce860_50000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_60000000, "setgain", 8, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "set_hr") ) {

    _0RL_cd_a6d83b30232ce860_50000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_70000000, "set_hr", 7, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "set_max_eh") ) {

    _0RL_cd_a6d83b30232ce860_80000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_90000000, "set_max_eh", 11, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "setWalkCommand") ) {

    _0RL_cd_a6d83b30232ce860_a0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_b0000000, "setWalkCommand", 15, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "preSet") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_d0000000, "preSet", 7, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "start") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_e0000000, "start", 6, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "testMove") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_f0000000, "testMove", 9, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "fcontrol") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_01000000, "fcontrol", 9, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "fcontrol_write") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_11000000, "fcontrol_write", 15, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "exPos") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_21000000, "exPos", 6, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "halfPos") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_31000000, "halfPos", 8, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "hogex") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_41000000, "hogex", 6, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "clear_log") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_51000000, "clear_log", 10, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "clip_open_A") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_61000000, "clip_open_A", 12, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "clip_open_B") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_71000000, "clip_open_B", 12, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "clip_close_A") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_81000000, "clip_close_A", 13, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "clip_close_B") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_91000000, "clip_close_B", 13, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "save_log") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_a1000000, "save_log", 9, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "stop") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_b1000000, "stop", 5, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }

  if( omni::strMatch(op, "stepping") ) {

    _0RL_cd_a6d83b30232ce860_c0000000 _call_desc(_0RL_lcfn_a6d83b30232ce860_c1000000, "stepping", 9, 1);
    
    _handle.upcall(this,_call_desc);
    return 1;
  }


  return 0;
}

void*
OpenHRP::_impl_JunoService::_ptrToInterface(const char* id)
{
  if( id == ::OpenHRP::JunoService::_PD_repoId )
    return (::OpenHRP::_impl_JunoService*) this;
  
  if( id == ::CORBA::Object::_PD_repoId )
    return (void*) 1;

  if( omni::strMatch(id, ::OpenHRP::JunoService::_PD_repoId) )
    return (::OpenHRP::_impl_JunoService*) this;
  
  if( omni::strMatch(id, ::CORBA::Object::_PD_repoId) )
    return (void*) 1;
  return 0;
}

const char*
OpenHRP::_impl_JunoService::_mostDerivedRepoId()
{
  return ::OpenHRP::JunoService::_PD_repoId;
}

POA_OpenHRP::JunoService::~JunoService() {}

