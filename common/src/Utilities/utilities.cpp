/*!
 * @file utilities.cpp
 * @brief Common utility functions
 */


#include <ctime>
#include <iomanip>
#include <iostream>

#include "Utilities/utilities.h"


/*!
 * Write std::string to file with given name
 */
void writeStringToFile(const std::string& fileName,
                       const std::string& fileData) {
  FILE* fp = fopen(fileName.c_str(), "w");
  if (!fp) {
    printf("Failed to fopen %s\n", fileName.c_str());
    throw std::runtime_error("Failed to open file");
  }
  fprintf(fp, "%s", fileData.c_str());
  fclose(fp);
}

/*!
 * Get the current time and date as a string
 */
std::string getCurrentTimeAndDate() {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream ss;
  ss << std::put_time(&tm, "%c");
  return ss.str();
}

/*!
 * Todo: do something better to keep track of where we are relative to the
 * config directory
 */
std::string getConfigDirectoryPath(const fs::path &path) {

  fs::path p1 = fs::path("/opt/locomotion/config/") / path.filename();
  fs::path p2 = fs::path("./config/") / path.filename();
  fs::path p3 = fs::path("../config/") / path.filename();

  fs::path p = "";

  if (fs::exists(p1))
    p = p1;
  else if (fs::exists(p2))
    p = p2;
  else if (fs::exists(p3))
    p = p3;

  // std::cout << "config path: " << p << std::endl;

  return p; 
}

/*!
 * Get the LCM URL with desired TTL.
 */
std::string getLcmUrl(s64 ttl) {
  assert(ttl >= 0 && ttl <= 255);
  return "udpm://239.255.76.67:7667?ttl=" + std::to_string(ttl);
}