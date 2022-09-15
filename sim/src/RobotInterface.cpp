#include "RobotInterface.h"
#include <ControlParameters/ControlParameterInterface.h>
#include <Dynamics/Cheetah3.h>
#include <Dynamics/MiniCheetah.h>
#include <unistd.h>
#include "ControlParameters/SimulatorParameters.h"

RobotInterface::RobotInterface(RobotType robotType, Graphics3D *gfx,
                               PeriodicTaskManager *tm, ControlParameters& userParameters)
    : PeriodicTask(tm, ROBOT_INTERFACE_UPDATE_PERIOD, "robot-interface"),
#ifdef LCM_MSG
      _lcm(getLcmUrl(255)),
#endif
      _userParameters(userParameters) {
#ifdef LCM_MSG
  _parameter_request_lcmt.requestNumber = 0;
#endif
  _gfx = gfx;
  _robotType = robotType;
  printf("[RobotInterface] Load parameters...\n");
  if (_robotType == RobotType::MINI_CHEETAH) {
    _controlParameters.initializeFromYamlFile(getConfigDirectoryPath(MINI_CHEETAH_DEFAULT_PARAMETERS));
  } else if (_robotType == RobotType::CHEETAH_3) {
    _controlParameters.initializeFromYamlFile(getConfigDirectoryPath(CHEETAH_3_DEFAULT_PARAMETERS));
  } else {
    assert(false);
  }

  if (!_controlParameters.isFullyInitialized()) {
    printf("Not all robot control parameters were initialized. Missing:\n%s\n",
           _controlParameters.generateUnitializedList().c_str());
    throw std::runtime_error("not all parameters initialized from ini file");
  }
  printf("[RobotInterface] Init LCM\n");
  printf("[RobotInterface] Init graphics\n");
  Vec4<float> robotColor;
  robotColor << 0.6, 0.2, 0.2, 1.0;

  if (_robotType == RobotType::MINI_CHEETAH)
    _robotID = gfx->setupMiniCheetah(robotColor, true, false);
#ifdef CHEETAH3
  else if (_robotType == RobotType::CHEETAH_3)
    _robotID = gfx->setupCheetah3(robotColor, true, false);
#endif
  else
    assert(false);

  printf("draw list has %lu items\n", _gfx->_drawList._kinematicXform.size());
  _gfx->_drawList._visualizationData = &_visualizationData;
  Checkerboard checker(10, 10, 10, 10);
  uint64_t floorID = _gfx->_drawList.addCheckerboard(checker, true);
  _gfx->_drawList.updateCheckerboard(0, floorID);
  _gfx->_drawList.buildDrawList();

#ifdef LCM_MSG
  _lcm.subscribe("interface_response", &RobotInterface::handleControlParameter,
                 this);
  _lcm.subscribe("main_cheetah_visualization",
                 &RobotInterface::handleVisualizationData, this);
#endif

  printf("[RobotInterface] Init dynamics\n");
  if (robotType == RobotType::MINI_CHEETAH)
    _quadruped = buildMiniCheetah<double>();
#ifdef CHEETAH3
  else if (robotType == RobotType::CHEETAH_3)
    _quadruped = buildCheetah3<double>();
#endif
  else
    assert(false);
  _model = _quadruped.buildModel();
  _simulator = new DynamicsSimulator<double>(_model, false);
  DVec<double> zero12(12);
  for (u32 i = 0; i < 12; i++) {
    zero12[i] = 0.;
  }

  _fwdKinState.q = zero12;
  _fwdKinState.qd = zero12;
}

#ifdef LCM_MSG
void RobotInterface::handleVisualizationData(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const cheetah_visualization_lcmt *msg) {
  (void)rbuf;
  (void)chan;
  for (int i = 0; i < 3; i++) {
    _fwdKinState.bodyPosition[i] = msg->x[i];
  }

  for (int i = 0; i < 4; i++) {
    _fwdKinState.bodyOrientation[i] = msg->quat[i];
  }

  for (int i = 0; i < 12; i++) {
    _fwdKinState.q[i] = msg->q[i];
  }

  _simulator->setState(_fwdKinState);
  _simulator->forwardKinematics();
}
#endif

void RobotInterface::run() {
  if (_gfx) {
    _gfx->_drawList.updateRobotFromModel(*_simulator, _robotID, true);
    _gfx->update();
#ifdef LCM_MSG
    _gfx->getDriverCommand().get(&_gamepad_lcmt);
    _lcm.publish(INTERFACE_LCM_NAME, &_gamepad_lcmt);
#endif
  }
}

using namespace std::chrono_literals;

void RobotInterface::sendControlParameter(const std::string &name,
                                          ControlParameterValue value,
                                          ControlParameterValueKind kind, bool isUser) {
#ifdef LCM_MSG
// temporarily disable robot interface
  if (_pendingControlParameterSend) {
    printf(
        "[ERROR] trying to send control parameter while a send is in progress, "
        "ignoring!\n");
    return;
  }
  _pendingControlParameterSend = true;
  for (int iteration = 0; iteration < TIMES_TO_RESEND_CONTROL_PARAM;) {
    // new message
    _parameter_request_lcmt.requestNumber++;

    // message data
    _parameter_request_lcmt.requestKind = isUser ?
        (s8)ControlParameterRequestKind::SET_USER_PARAM_BY_NAME : (s8)ControlParameterRequestKind::SET_ROBOT_PARAM_BY_NAME;
    strcpy((char *)_parameter_request_lcmt.name, name.c_str());
    memcpy(_parameter_request_lcmt.value, &value, sizeof(value));
    _parameter_request_lcmt.parameterKind = (s8)kind;
    printf("set %s to %s (%d)\n", name.c_str(),
           controlParameterValueToString(value, kind).c_str(), iteration);

    // send
    _lcm.publish("interface_request", &_parameter_request_lcmt);

    // wait for response with timeout
    _waitingForLcmResponse = true;
    _lcmResponseBad = true;
    std::unique_lock<std::mutex> lock(_lcmMutex);

    if (_lcmCV.wait_for(lock, 100ms) == std::cv_status::no_timeout) {
      _waitingForLcmResponse = false;
      // check it
      if (_waitingForLcmResponse || _lcmResponseBad) {
        printf(
            "[RobotInterface] Failed to send parameter %s (iter %d) "
            "wakeup %d bad? %d\n",
            name.c_str(), iteration, _waitingForLcmResponse, _lcmResponseBad);
        usleep(100000);  // sleep a bit to let other bad sends happen
      } else {
        iteration++;
      }
    } else {
      _waitingForLcmResponse = false;
      // fail!
      printf(
          "[RobotInterface] Failed to send parameter %s (iter %d timed out), "
          "trying again...\n",
          name.c_str(), iteration);
      usleep(100000);  // sleep a bit to let other bad sends happen
    }
  }
  _pendingControlParameterSend = false;
#endif
}

#ifdef LCM_MSG
void RobotInterface::handleControlParameter(
    const lcm::ReceiveBuffer *rbuf, const std::string &chan,
    const control_parameter_respones_lcmt *msg) {
  (void)rbuf;
  (void)chan;
  if (!_waitingForLcmResponse) {
    printf(
        "[RobotInterface] Got a control parameter response when we weren't "
        "expecting one, ignoring it!\n");
    _lcmResponseBad = true;
    return;
  }

  // got a real response, let's check it
  if (msg->requestNumber == _parameter_request_lcmt.requestNumber &&
      msg->parameterKind == _parameter_request_lcmt.parameterKind &&
      std::string((char *)msg->name) == (char *)_parameter_request_lcmt.name) {
    _lcmResponseBad = false;
    std::unique_lock<std::mutex> lock(_lcmMutex);
    _waitingForLcmResponse = false;
    _lcmCV.notify_all();
  }
}
#endif

void RobotInterface::startInterface() {
  _running = true;
  this->start();
#ifdef LCM_MSG
  _lcmThread = std::thread(&RobotInterface::lcmHandler, this);
#endif
  printf("[RobotInterface] Send parameters to robot...\n");
  for (auto &kv : _controlParameters.collection._map) {
    sendControlParameter(kv.first, kv.second->get(kv.second->_kind),
                         kv.second->_kind, false);
  }

  for (auto &kv : _userParameters.collection._map) {
    sendControlParameter(kv.first, kv.second->get(kv.second->_kind),
                         kv.second->_kind, true);
  }
  
}

void RobotInterface::stopInterface() {
  printf("stopInterface\n");
  _running = false;
  _taskManager.stopAll();
  printf("stopall done\n");
  _lcmThread.join();
  printf("lcmthread joined\n");
}

#ifdef LCM_MSG
void RobotInterface::lcmHandler() {
  while (_running) {
    _lcm.handleTimeout(1000);
  }
}
#endif
