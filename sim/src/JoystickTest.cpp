#include "../include/JoystickTest.h"
#include "ui_JoystickTest.h"
#include <QTimer>


JoystickTestWindow::JoystickTestWindow(GameController& gamepad, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::JoystickTestWindow),
    _gamepad(gamepad)
{
    ui->setupUi(this);

  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(update()));
  timer->start(1000 / 30);
}

JoystickTestWindow::~JoystickTestWindow()
{
    delete ui;
}


void JoystickTestWindow::update() {
  _gamepad.updateGamepadCommand(_command);
  char buffer[256];

  sprintf(buffer, "Left X: %4.2f\n", _command.leftStickAnalog[0]);
  ui->leftXLabel->setText(buffer);

  sprintf(buffer, "Left Y: %4.2f\n", _command.leftStickAnalog[1]);
  ui->leftYLabel->setText(buffer);

  sprintf(buffer, "Right X: %4.2f\n", _command.rightStickAnalog[0]);
  ui->rightXLabel->setText(buffer);

  sprintf(buffer, "Right Y: %4.2f\n", _command.rightStickAnalog[1]);
  ui->rightYLabel->setText(buffer);

  sprintf(buffer, "Left Trigger: %4.2f\n", _command.leftTriggerAnalog);
  ui->leftTriggerLabel->setText(buffer);

  sprintf(buffer, "Right Trigger: %4.2f\n", _command.rightTriggerAnalog);
  ui->rightTriggerLabel->setText(buffer);

  sprintf(buffer, "A: %d\n", _command.a);
  ui->aLabel->setText(buffer);

  sprintf(buffer, "B: %d\n", _command.b);
  ui->bLabel->setText(buffer);

  sprintf(buffer, "X: %d\n", _command.x);
  ui->xLabel->setText(buffer);

  sprintf(buffer, "Y: %d\n", _command.y);
  ui->yLabel->setText(buffer);

  sprintf(buffer, "Left Trigger: %d\n", _command.leftTriggerButton);
  ui->leftTriggerButtonLabel->setText(buffer);

  sprintf(buffer, "Right Trigger: %d\n", _command.rightTriggerButton);
  ui->rightTriggerButtonLabel->setText(buffer);

  sprintf(buffer, "Start: %d\n", _command.start);
  ui->startLabel->setText(buffer);

  sprintf(buffer, "Back: %d\n", _command.back);
  ui->backLabel->setText(buffer);
}
