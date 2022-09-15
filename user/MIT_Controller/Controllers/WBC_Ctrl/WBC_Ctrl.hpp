#ifndef WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

#include <FSM_States/ControlFSMData.h>
#include <Dynamics/FloatingBaseModel.h>
#include <Dynamics/Quadruped.h>
#include "cppTypes.h"
#include <WBC/WBIC/WBIC.hpp>
#include <WBC/WBIC/KinWBC.hpp>

#ifdef LCM_MSG
#include <lcm/lcm-cpp.hpp>
#include "wbc_test_data_t.hpp"
#else
typedef struct _wbc_test_data_t {
  int32_t contact_est[4];
  float Fr_des[12];
  float Fr[12];
  float body_ori_cmd[4];
  float body_pos_cmd[3];
  float body_vel_cmd[3];
  float body_ang_vel_cmd[3];
  float body_pos[3];
  float body_vel[3];
  float body_ori[4];
  float body_ang_vel[3];
  float foot_pos_cmd[12];
  float foot_vel_cmd[12];
  float foot_acc_cmd[12];
  float foot_acc_numeric[12];
  float foot_pos[12];
  float foot_vel[12];
  float foot_local_pos[12];
  float foot_local_vel[12];
  float jpos_cmd[12];
  float jvel_cmd[12];
  float jacc_cmd[12];
  float jpos[12];
  float jvel[12];
  float vision_loc[3];
}
wbc_test_data_t;
#endif

#define WBCtrl WBC_Ctrl<T>

class MIT_UserParameters;

template<typename T>
class WBC_Ctrl{
  public:
    WBC_Ctrl(FloatingBaseModel<T> model);
    virtual ~WBC_Ctrl();

    void run(void * input, ControlFSMData<T> & data);
    void setFloatingBaseWeight(const T & weight){
      _wbic_data->_W_floating = DVec<T>::Constant(6, weight);
    }

  protected:
    virtual void _ContactTaskUpdate(void * input, ControlFSMData<T> & data) = 0;
    virtual void _ContactTaskUpdateTEST(void * input, ControlFSMData<T> & data){
      (void)input;
      (void)data;
    }
#ifdef LCM_MSG
    virtual void _LCM_PublishData(){}
#endif
    void _UpdateModel(const StateEstimate<T> & state_est, const LegControllerData<T> * leg_data);
    void _UpdateLegCMD(ControlFSMData<T> & data);
    void _ComputeWBC();

    KinWBC<T>* _kin_wbc;
    WBIC<T>* _wbic;
    WBIC_ExtraData<T>* _wbic_data;

    FloatingBaseModel<T> _model;
    std::vector<ContactSpec<T> * > _contact_list;
    std::vector<Task<T> * > _task_list;

    DMat<T> _A;
    DMat<T> _Ainv;
    DVec<T> _grav;
    DVec<T> _coriolis;

    FBModelState<T> _state;

    DVec<T> _full_config;
    DVec<T> _tau_ff;
    DVec<T> _des_jpos;
    DVec<T> _des_jvel;

    std::vector<T> _Kp_joint, _Kd_joint;
    //std::vector<T> _Kp_joint_swing, _Kd_joint_swing;

    unsigned long long _iter;

#ifdef LCM_MSG
    lcm::LCM _wbcLCM;
    wbc_test_data_t _wbc_data_lcm;
#else
    wbc_test_data_t _wbc_data_lcm;
#endif
};
#endif
