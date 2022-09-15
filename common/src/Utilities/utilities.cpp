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

  fs::path p = fs::path();
  fs::path opt = fs::path("/opt/locomotion/config/") / path.filename();

  if (fs::exists(opt)) {
    p = opt;
  } else {
    fs::path cwd = fs::current_path();
    fs::path config = fs::path("config");

    for (int i = 0; i < 5; i++) {
      fs::path pp = cwd / config / path.filename();
      if (fs::exists(pp)) {
        p = pp;
        break;
      } else {
        cwd = cwd.parent_path();
      }
    }
  }

  if (p.empty()) {
    throw std::runtime_error("ERROR: %s not found\n");
  }

  return p; 
}

#ifdef LCM_MSG
/*!
 * Get the LCM URL with desired TTL.
 */
std::string getLcmUrl(s64 ttl) {
  assert(ttl >= 0 && ttl <= 255);
  return "udpm://239.255.76.67:7667?ttl=" + std::to_string(ttl);
}
#endif