/*! @file GameController.h
 *  @brief Code to read the Logitech F310 Game Controller
 *  Creates a DriverCommand object to be sent to the robot controller
 *  Used in the development simulator and in the robot control mode
 */

#ifndef PROJECT_GAMECONTROLLER_H
#define PROJECT_GAMECONTROLLER_H

#include "SimUtilities/GamepadCommand.h"

#include <QtGlobal>

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
#define QT_GAMEPAD 1
#endif

#ifdef QT_GAMEPAD
#include <QGamepad>
#include <QGamepadManager>
#include <QGamepadKeyNavigation>
#else
#include <QObject>
#include <SDL.h>
#endif

class GameController : public QObject {
  Q_OBJECT
 public:
  explicit GameController(QObject *parent = 0);
  void updateGamepadCommand(GamepadCommand &gamepadCommand);
  void findNewController();
  ~GameController();

 private:
#ifdef QT_GAMEPAD
  QGamepad *_qGamepad = nullptr;
#else
  int _recheckCount = 0;
  SDL_GameController *_controller = NULL;
#endif
};

#endif  // PROJECT_GAMECONTROLLER_H
