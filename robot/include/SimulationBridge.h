/*! @file SimulationBridge.h
 *  @brief  The SimulationBridge runs a RobotController and connects it to a
 * Simulator, using shared memory. It is the simulation version of the
 * HardwareBridge.
 */

#ifndef PROJECT_SIMULATIONDRIVER_H
#define PROJECT_SIMULATIONDRIVER_H

#include <thread>

#include "ControlParameters/RobotParameters.h"
#include "RobotRunner.h"
#include "SimUtilities/SimulatorMessage.h"
#include "Types.h"
#include "Utilities/PeriodicTask.h"
#include "Utilities/SharedMemory.h"

class SimulationBridge {
 public:
  explicit SimulationBridge(RobotType robot, RobotController* robot_ctrl) : 
    _robot(robot) {
     _taskManager = new PeriodicTaskManager;
    _robotRunner = new RobotRunner(robot_ctrl, _taskManager, 0, "robot-task");
    _userParams = robot_ctrl->getUserControlParameters();

 }
  void run();
  void handleControlParameters();
  void runRobotControl();
  ~SimulationBridge() {
    delete _taskManager;
    delete _robotRunner;
  }

#ifdef SBUS_CONTROLLER
  void run_sbus();
#endif

 private:
  bool _firstControllerRun = true;
  PeriodicTaskManager* _taskManager = nullptr;
  RobotType _robot;
  RobotRunner* _robotRunner = nullptr;
  SimulatorMode _simMode;
  SimulatorSyncronized _sharedMemory;
  RobotControlParameters _robotParams;
  ControlParameters* _userParams = nullptr;
  u64 _iterations = 0;

#ifdef SBUS_CONTROLLER
  std::thread* sbus_thread;
#endif
};

#endif  // PROJECT_SIMULATIONDRIVER_H
