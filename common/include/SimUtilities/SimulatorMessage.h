/*! @file SimulatorMessage.h
 *  @brief Messages sent to/from the development simulator
 *
 *  These messsages contain all data that is exchanged between the robot program
 * and the simulator using shared memory.   This is basically everything except
 * for debugging logs, which are handled by LCM instead
 */

#ifndef PROJECT_SIMULATORTOROBOTMESSAGE_H
#define PROJECT_SIMULATORTOROBOTMESSAGE_H

#include "ControlParameters/ControlParameterInterface.h"
#include "SimUtilities/GamepadCommand.h"
#include "SimUtilities/IMUTypes.h"
#include "SimUtilities/SpineBoard.h"
#include "SimUtilities/VisualizationData.h"
#include "SimUtilities/ti_boardcontrol.h"
#include "Utilities/SharedMemory.h"

/*!
 * The mode for the simulator
 */
enum class SimulatorMode {
  RUN_CONTROL_PARAMETERS,  // don't run the robot controller, just process
                           // Control Parameters
  RUN_CONTROLLER,          // run the robot controller
  DO_NOTHING,              // just to check connection
  EXIT                     // quit!
};

/*!
 * A plain message from the simulator to the robot
 */
struct SimulatorToRobotMessage {
  GamepadCommand gamepadCommand;  // joystick
  RobotType robotType;  // which robot the simulator thinks we are simulating

  // imu data
  VectorNavData vectorNav;
  CheaterState<double> cheaterState;

  // leg data
  SpiData spiData;
#ifdef CHEETAH3
  TiBoardData tiBoardData[4];
#endif
  // todo cheetah 3
  ControlParameterRequest controlParameterRequest;

  SimulatorMode mode;
};

/*!
 * A plain message from the robot to the simulator
 */
struct RobotToSimulatorMessage {
  RobotType robotType;
  SpiCommand spiCommand;
#ifdef CHEETAH3
  TiBoardCommand tiBoardCommand[4];
#endif

  VisualizationData visualizationData;
  CheetahVisualization mainCheetahVisualization;
  ControlParameterResponse controlParameterResponse;

  char errorMessage[2048];
};

/*!
 * All the data shared between the robot and the simulator
 */
struct SimulatorMessage {
  RobotToSimulatorMessage robotToSim;
  SimulatorToRobotMessage simToRobot;
};

#if 0
/*!
 * A SimulatorSyncronizedMessage is stored in shared memory and is accessed by
 * both the simulator and the robot The simulator and robot take turns have
 * exclusive access to the entire message. The intended sequence is:
 *  - robot: waitForSimulator()
 *  - simulator: *simulates robot* (simulator can read/write, robot cannot do
 * anything)
 *  - simulator: simDone()
 *  - simulator: waitForRobot()
 *  - robot: *runs controller*    (robot can read/write, simulator cannot do
 * anything)
 *  - robot: robotDone();
 *  - robot: waitForSimulator()
 *  ...
 */
struct SimulatorSyncronizedMessage : public SimulatorMessage {

  /*!
   * The init() method should only be called *after* shared memory is connected!
   * This initializes the shared memory semaphores used to keep things in sync
   */
  void init() {
    robotToSimSemaphore.init(0);
    simToRobotSemaphore.init(0);
  }

  /*!
   * Wait for the simulator to respond
   */
  void waitForSimulator() { simToRobotSemaphore.decrement(); }

  /*!
   * Simulator signals that it is done
   */
  void simulatorIsDone() { simToRobotSemaphore.increment(); }

  /*!
   * Wait for the robot to finish
   */
  void waitForRobot() { robotToSimSemaphore.decrement(); }

  /*!
   * Check if the robot is done
   * @return if the robot is done
   */
  bool tryWaitForRobot() { return robotToSimSemaphore.tryDecrement(); }

  /*!
   * Wait for the robot to finish with a timeout
   * @return if we finished before timing out
   */
  bool waitForRobotWithTimeout() {
    return robotToSimSemaphore.decrementTimeout(1, 0);
  }

  /*!
   * Signal that the robot is done
   */
  void robotIsDone() { robotToSimSemaphore.increment(); }

 private:
  SharedMemorySemaphore robotToSimSemaphore, simToRobotSemaphore;
};

#else

#define ROBOT_SEMAPHORE_NAME "robot-semaphore"
#define SIMULATOR_SEMAPHORE_NAME "simulator-semaphore"

class SimulatorSyncronized {
 public:

  int create() {
    _simToRobotSemaphore.create(SIMULATOR_SEMAPHORE_NAME);
    _robotToSimSemaphore.create(ROBOT_SEMAPHORE_NAME);
    _sharedMemory.create(DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME, true);

    return 0;
  }

  void destory() {
    _simToRobotSemaphore.destroy();
    _robotToSimSemaphore.destroy();
    _sharedMemory.destroy();
  }

  int attach() {
    debug_memory_usage();
    _sharedMemory.attach(DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME);
    _simToRobotSemaphore.attach(SIMULATOR_SEMAPHORE_NAME);
    _robotToSimSemaphore.attach(ROBOT_SEMAPHORE_NAME);

    return 0;
  }

  SimulatorMessage& getObject() {
    return _sharedMemory.getObject();
  }

  void debug_memory_usage() {
    printf("SimulatorMessage: %jd bytes\n", sizeof(SimulatorMessage));

    printf("\tRobotToSimulatorMessage: %jd bytes\n", sizeof(RobotToSimulatorMessage));
    printf("\t\tSpiCommand: %jd bytes\n", sizeof(SpiCommand));
    printf("\t\tVisualizationData: %jd bytes\n", sizeof(VisualizationData));
    printf("\t\tCheetahVisualization: %jd bytes\n", sizeof(CheetahVisualization));
    printf("\t\tControlParameterResponse: %jd bytes\n", sizeof(ControlParameterResponse));

    printf("\tSimulatorToRobotMessage: %jd bytes\n", sizeof(SimulatorToRobotMessage));
    printf("\t\tGamepadCommand: %jd bytes\n", sizeof(GamepadCommand));
    printf("\t\tVectorNavData: %jd bytes\n", sizeof(VectorNavData));
    printf("\t\tCheaterState<double>: %jd bytes\n", sizeof(CheaterState<double>));
    printf("\t\tSpiData: %jd bytes\n", sizeof(SpiData));
    printf("\t\tControlParameterRequest: %jd bytes\n", sizeof(ControlParameterRequest));
  }

  /*!
   * Wait for the simulator to respond
   */
  void waitForSimulator() { _simToRobotSemaphore.wait(); }

  /*!
   * Simulator signals that it is done
   */
  void simulatorIsDone() { _simToRobotSemaphore.post(); }

  /*!
   * Wait for the robot to finish
   */
  void waitForRobot() { _robotToSimSemaphore.wait(); }

  /*!
   * Check if the robot is done
   * @return if the robot is done
   */
  bool tryWaitForRobot() { return _robotToSimSemaphore.tryWait(); }

  /*!
   * Wait for the robot to finish with a timeout
   * @return if we finished before timing out
   */
  bool waitForRobotWithTimeout() {
    return _robotToSimSemaphore.waitWithTimeout(1, 0);
  }

  /*!
   * Signal that the robot is done
   */
  void robotIsDone() { _robotToSimSemaphore.post(); }

 private:
  SharedMemorySemaphore _robotToSimSemaphore;
  SharedMemorySemaphore _simToRobotSemaphore;
  SharedMemoryObject<SimulatorMessage> _sharedMemory;
};

#endif 

#endif  // PROJECT_SIMULATORTOROBOTMESSAGE_H
