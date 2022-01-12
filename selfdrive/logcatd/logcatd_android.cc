#include <sys/time.h>
#include <sys/resource.h>

#include <android/log.h>
#include <log/logger.h>
#include <log/logprint.h>

#include "cereal/messaging/messaging.h"
#include "selfdrive/common/util.h"

int main() {
  setpriority(PRIO_PROCESS, 0, -15);

  ExitHandler do_exit;
  PubMaster pm({"androidLog"});

  struct timespec cur_time;
  clock_gettime(CLOCK_REALTIME, &cur_time);
  log_time last_log_time(cur_time);
  logger_list *logger_list = nullptr;

  while (!do_exit) {
    // setup android logging
    if (!logger_list) {
      logger_list = android_logger_list_alloc_time(ANDROID_LOG_RDONLY | ANDROID_LOG_NONBLOCK, last_log_time, 0);
    }
    assert(logger_list);

    struct logger *main_logger = android_logger_open(logger_list, LOG_ID_MAIN);
    assert(main_logger);
    struct logger *radio_logger = android_logger_open(logger_list, LOG_ID_RADIO);
    assert(radio_logger);
    struct logger *system_logger = android_logger_open(logger_list, LOG_ID_SYSTEM);
    assert(system_logger);
    struct logger *crash_logger = android_logger_open(logger_list, LOG_ID_CRASH);
    assert(crash_logger);
    struct logger *kernel_logger = android_logger_open(logger_list, (log_id_t)5); // LOG_ID_KERNEL
    assert(kernel_logger);

    while (!do_exit) {
      log_msg log_msg;
      int err = android_logger_list_read(logger_list, &log_msg);
      if (err <= 0) break;

      AndroidLogEntry entry;
      err = android_log_processLogBuffer(&log_msg.entry_v1, &entry);
      if (err < 0) continue;
      last_log_time.tv_sec = entry.tv_sec;
      last_log_time.tv_nsec = entry.tv_nsec;

      MessageBuilder msg;
      auto androidEntry = msg.initEvent().initAndroidLog();
      androidEntry.setId(log_msg.id());
      androidEntry.setTs(entry.tv_sec * 1000000000ULL + entry.tv_nsec);
      androidEntry.setPriority(entry.priority);
      androidEntry.setPid(entry.pid);
      androidEntry.setTid(entry.tid);
      androidEntry.setTag(entry.tag);
      androidEntry.setMessage(entry.message);

      pm.send("androidLog", msg);
    }

    android_logger_list_free(logger_list);
    logger_list = NULL;
    util::sleep_for(500);
  }

  if (logger_list) {
    android_logger_list_free(logger_list);
  }

  return 0;
}
