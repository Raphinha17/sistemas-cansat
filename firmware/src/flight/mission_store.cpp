#include "mission_store.h"

#include <Preferences.h>

namespace mission_store {

static Preferences prefs;
static bool ok_ = false;
static uint32_t writes_ = 0;
static const char *kKey = "mision";

bool begin() {
  ok_ = prefs.begin("cansat", false);
  return ok_;
}

bool load(MissionRecord &r) {
  if (!ok_ || prefs.getBytesLength(kKey) != sizeof(MissionRecord)) return false;
  MissionRecord tmp;
  if (prefs.getBytes(kKey, &tmp, sizeof tmp) != sizeof tmp) return false;
  if (tmp.magic != kMissionMagic) return false;
  r = tmp;
  return true;
}

bool save(const MissionRecord &r) {
  if (!ok_) return false;
  writes_++;
  return prefs.putBytes(kKey, &r, sizeof r) == sizeof r;
}

void clear() {
  if (ok_) prefs.remove(kKey);
}

uint32_t writes() { return writes_; }

}  // namespace mission_store
