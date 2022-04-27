#ifndef PROJECT_RT_ETHERCAT_H
#define PROJECT_RT_ETHERCAT_H

#ifdef CHEETAH3
#include <cstdint>

void rt_ethercat_init();
void rt_ethercat_run();

struct TiBoardData;
struct TiBoardCommand;

void rt_ethercat_get_data(TiBoardData* data);
void rt_ethercat_set_command(TiBoardCommand* command);

#endif

#endif //PROJECT_RT_ETHERCAT_H
