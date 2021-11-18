/*! @file GameController.cpp
 *  @brief Code to read the Logitech F310 Game Controller
 *  Creates a DriverCommand object to be sent to the robot controller
 *  Used in the development simulator and in the robot control mode
 *
 *  NOTE: Because QT is weird, the updateDriverCommand has to be called from a
 * QT event. Running it in another thread will cause it to not work. As a
 * result, this only works if called in the update method of a QTObject
 */

#include "GameController.h"

#include <QtCore/QObject>
#ifdef QT_GAMEPAD
#include <QtGamepad/QGamepad>
#else
#include <SDL.h>
#endif

/*!
 * By default, the game controller selects the "first" joystick, printing a
 * warning if there are multiple joysticks On Linux, this is /dev/input/js0 If
 * no joystick is found, it will print an error message, and will return zero.
 * It is possible to change/add a joystick later with findNewController
 */
GameController::GameController(QObject *parent) : QObject(parent) {
#ifndef QT_GAMEPAD
  SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK);
#endif
  findNewController();
}

static inline float s16ToFloat(s16 axis)
{
  return axis >= 0 ? axis / 32767.0 : axis / 32768.0;
}

/*!
 * Re-run the joystick finding code to select the "first" joystick. This can be
 * used to set up the joystick if the simulator is started without a joystick
 * plugged in
 */
void GameController::findNewController() {
#if QT_GAMEPAD
  delete _qGamepad;
  _qGamepad = nullptr;  // in case this doesn't work!

  printf("[Gamepad] Searching for gamepads, please ignore \"Device discovery cannot open device\" errors\n");
  auto gamepadList = QGamepadManager::instance()->connectedGamepads();
  printf("[Gamepad] Done searching for gamepads.\n");
  if (gamepadList.empty()) {
    printf(
        "[ERROR: GameController] No controller was connected! All joystick "
        "commands will be zero!\n");
  } else {
    if (gamepadList.size() > 1) {
      printf(
          "[ERROR: GameController] There are %d joysticks connected.  Using "
          "the first one.\n",
          gamepadList.size());
    } else {
      printf("[GameController] Found 1 joystick\n");
    }

    _qGamepad = new QGamepad(*gamepadList.begin());
  }
#else
  for (int i = 0; i < SDL_NumJoysticks(); i++) {
    if (SDL_IsGameController(i)) {
      _controller = SDL_GameControllerOpen(i);
      if (_controller) {
        break;
      } else {
        fprintf(stderr, "[GameController] Could not open gamecontroller %i: %s\n", i, SDL_GetError());
      }
    }
  }

  if (_controller && SDL_GameControllerGetAttached(_controller)) {
    printf("[GameController] Name: %s\n", SDL_GameControllerName(_controller));
    printf("[GameController] Has Accelerometer: %s\n", SDL_GameControllerHasSensor(_controller, SDL_SENSOR_ACCEL) ? "True" : "False");
    printf("[GameController] Has Gyroscope: %s\n", SDL_GameControllerHasSensor(_controller, SDL_SENSOR_GYRO) ? "True" : "False");
    printf("[GameController] Has LED: %s\n", SDL_GameControllerHasLED(_controller) ? "True" : "False");
  } else {
    printf("[GameController] Not connected Game Controller\n");
  }
#endif
}

/*!
 * Overwrite a driverCommand with the current joystick state.  If there's no
 * joystick, sends zeros
 * TODO: what happens if the joystick is unplugged?
 */
void GameController::updateGamepadCommand(GamepadCommand &gamepadCommand) {
#ifdef QT_GAMEPAD
  if (_qGamepad) {
    gamepadCommand.leftBumper = _qGamepad->buttonL1();
    gamepadCommand.rightBumper = _qGamepad->buttonR1();
    gamepadCommand.leftTriggerButton = _qGamepad->buttonL2() != 0.;
    gamepadCommand.rightTriggerButton = _qGamepad->buttonR2() != 0.;
    gamepadCommand.back = _qGamepad->buttonSelect();
    gamepadCommand.start = _qGamepad->buttonStart();
    gamepadCommand.a = _qGamepad->buttonA();
    gamepadCommand.b = _qGamepad->buttonB();
    gamepadCommand.x = _qGamepad->buttonX();
    gamepadCommand.y = _qGamepad->buttonY();
    gamepadCommand.leftStickButton = _qGamepad->buttonL3();
    gamepadCommand.rightStickButton = _qGamepad->buttonR3();
    gamepadCommand.leftTriggerAnalog = (float)_qGamepad->buttonL2();
    gamepadCommand.rightTriggerAnalog = (float)_qGamepad->buttonR2();
    gamepadCommand.leftStickAnalog =
        Vec2<float>(_qGamepad->axisLeftX(), -_qGamepad->axisLeftY());
    gamepadCommand.rightStickAnalog =
        Vec2<float>(_qGamepad->axisRightX(), -_qGamepad->axisRightY());
  } else {
    gamepadCommand.zero();  // no joystick, return all zeros
  }
#else
  SDL_GameControllerUpdate();

  if (_controller && SDL_GameControllerGetAttached(_controller)) {

    gamepadCommand.leftBumper = false;
    gamepadCommand.rightBumper = false;

    gamepadCommand.leftTriggerButton = SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_LEFTSHOULDER) == 1 ? true : false; // L1
    gamepadCommand.rightTriggerButton = SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER) == 1 ? true : false; // R1

    gamepadCommand.start = SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_START) == 1 ? true : false; // option
    gamepadCommand.back = SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_BACK) == 1 ? true : false; // share

    gamepadCommand.a = SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_A) == 1 ? true : false;
    gamepadCommand.b = SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_B) == 1 ? true : false;
    gamepadCommand.x = SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_X) == 1 ? true : false;
    gamepadCommand.y = SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_Y) == 1 ? true : false;

    gamepadCommand.leftStickButton = SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_LEFTSTICK) == 1 ? true : false;
    gamepadCommand.rightStickButton = SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_RIGHTSTICK) == 1 ? true : false;
    gamepadCommand.logitechButton = false;

    s16 leftX = SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_LEFTX);
    s16 leftY = SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_LEFTY);
    s16 rightX = SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_RIGHTX);
    s16 rightY = SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_RIGHTY);
    gamepadCommand.leftStickAnalog = Vec2<float>(s16ToFloat(leftX), s16ToFloat(leftY) * -1.0);
    gamepadCommand.rightStickAnalog = Vec2<float>(s16ToFloat(rightX), s16ToFloat(rightY) * -1.0);

    s16 triggerLeft = SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT); // L2
    s16 triggerRight = SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT); // R2
    gamepadCommand.leftTriggerAnalog = s16ToFloat(triggerLeft);
    gamepadCommand.rightTriggerAnalog = s16ToFloat(triggerRight);

  } else {

    if (_controller != NULL) {
      SDL_GameControllerClose(_controller);
      _controller = NULL;
    }

    if (_recheckCount > 300) {
      findNewController();
      _recheckCount = 0;
    } else {
      _recheckCount++;
    }

    gamepadCommand.zero();  // no joystick, return all zeros
  }
#endif

  // printf("%s\n", gamepadCommand.toString().c_str());
}

GameController::~GameController() {
#ifdef QT_GAMEPAD
  delete _qGamepad;
#else
  if (_controller && SDL_GameControllerGetAttached(_controller)) {
    SDL_GameControllerClose(_controller);
    _controller = NULL;
  }
#endif
}