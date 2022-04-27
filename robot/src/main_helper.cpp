/*!
 * @file main.cpp
 * @brief Main Function for the robot program
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <cassert>
#include <iostream>

#include "HardwareBridge.h"
#include "SimulationBridge.h"
#include "main_helper.h"
#include "RobotController.h"

MasterConfig gMasterConfig;

/*!
 * Print a message describing the command line flags for the robot program
 */
void printUsage() {
  printf(
      "Usage: robot [robot-id] [sim-or-robot] [parameters-from-file]\n"
      "\trobot-id:\n"
#ifdef CHEETAH3
      "\t                3 for cheetah 3\n"
#endif
      "\t                m for mini-cheetah\n"
      "\t                c for cyberdog\n"
      "\tsim-or-robot:\n"
      "\t                s for sim\n"
      "\t                r for robot\n"
      "\tparam-file:\n"
      "\t                f for loading parameters from file\n"
      "\t                l (or nothing) for LCM\n"
      "\t                  this option can only be used in robot mode\n");
}

/*!
 * Setup and run the given robot controller
 */
int main_helper(int argc, char** argv, RobotController* ctrl) {
  if (argc != 3 && argc != 4) {
    printUsage();
    return EXIT_FAILURE;
  }

  if (argv[1][0] == 'm') {
    gMasterConfig._robot = RobotType::MINI_CHEETAH;
  } else if (argv[1][0] == 'c') {
    gMasterConfig._robot = RobotType::CYBERDOG;
#ifdef CHEETAH3
  } else if (argv[1][0] == '3') {
    gMasterConfig._robot = RobotType::CHEETAH_3;
#endif
  } else {
    printUsage();
    return EXIT_FAILURE;
  }

  if (argv[2][0] == 's') {
    gMasterConfig.simulated = true;
  } else if (argv[2][0] == 'r') {
    gMasterConfig.simulated = false;
  } else {
    printUsage();
    return EXIT_FAILURE;
  }

  if(argc == 4 && argv[3][0] == 'f') {
    gMasterConfig.load_from_file = true;
    printf("Load parameters from file\n");
  } else {
    gMasterConfig.load_from_file = false;
    printf("Load parameters from network\n");
  }

  const char *robotName = NULL;
  switch (gMasterConfig._robot) {
    case RobotType::MINI_CHEETAH:
      robotName = "Mini Cheetah";
      break;
    case RobotType::CYBERDOG:
      robotName = "CyberDog";
      break;
#ifdef CHEETAH3
    case RobotType::CHEETAH_3:
      robotName = "Cheetah 3";
      break;
#endif
    default:
      robotName = "unknown";
  }

  printf("[Quadruped] Cheetah Software\n");
  printf("        Quadruped:  %s\n", robotName);
  printf("        Driver: %s\n", gMasterConfig.simulated
                                     ? "Development Simulation Driver"
                                     : "Quadruped Driver");

  // dispatch the appropriate driver
  if (gMasterConfig.simulated) {
    if(argc != 3) {
      printUsage();
      return EXIT_FAILURE;
    }
    if (gMasterConfig._robot == RobotType::MINI_CHEETAH) {
      SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);
      simulationBridge.run();
    } else if (gMasterConfig._robot == RobotType::CYBERDOG) {
      SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);
      simulationBridge.run();
#ifdef CHEETAH3
    } else if (gMasterConfig._robot == RobotType::CHEETAH_3) {
      SimulationBridge simulationBridge(gMasterConfig._robot, ctrl);
      simulationBridge.run();
#endif
    } else {
      printf("[ERROR] unknown robot\n");
      assert(false);
    }
  } else {
#ifdef linux
    if (gMasterConfig._robot == RobotType::MINI_CHEETAH) {
      MiniCheetahHardwareBridge hw(ctrl, gMasterConfig.load_from_file);
      hw.run();
    } else if (gMasterConfig._robot == RobotType::CYBERDOG) {
      MiniCheetahHardwareBridge hw(ctrl, gMasterConfig.load_from_file);
      hw.run();
#ifdef CHEETAH3
    } else if (gMasterConfig._robot == RobotType::CHEETAH_3) {
      Cheetah3HardwareBridge hw(ctrl);
      hw.run();
#endif
    } else {
      printf("[ERROR] unknown robot\n");
      assert(false);
    }
#endif
  }

  return 0;
}
