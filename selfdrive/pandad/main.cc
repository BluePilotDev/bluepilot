#include <cassert>

#include "selfdrive/pandad/pandad.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "system/hardware/hw.h"

int main(int argc, char *argv[]) {
  LOGW("starting pandad");

  if (!Hardware::PC()) {
    int err;
    err = util::set_realtime_priority(54);
    assert(err == 0);
    err = util::set_core_affinity({3});
    assert(err == 0);
  }

  std::string serial = (argc > 1) ? argv[1] : "";
  int panda_index = (argc > 2) ? std::stoi(argv[2]) : 0;

  // Additional args: serial_1 index_1 serial_2 index_2 ...
  std::vector<std::pair<std::string, int>> secondary_pandas;
  for (int i = 3; i + 1 < argc; i += 2) {
    secondary_pandas.push_back({argv[i], std::stoi(argv[i + 1])});
  }

  pandad_main_thread(serial, panda_index, secondary_pandas);
  return 0;
}
