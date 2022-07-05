/*! @file main.cpp
 *  @brief Main function for simulator
 */


#include "SimControlPanel.h"
#include "Utilities/SegfaultHandler.h"

#include <QApplication>

/*!
 * Setup QT and run a simulation
 */
int main(int argc, char *argv[]) {
  install_segfault_handler(nullptr);

  // Enables high-DPI scaling in Qt on supported platforms
  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

  // set up Qt
  QApplication application(argc, argv);

  // open simulator UI
  SimControlPanel controlPanel;
  controlPanel.show();

  // run the Qt program
  application.exec();

  return 0;
}
