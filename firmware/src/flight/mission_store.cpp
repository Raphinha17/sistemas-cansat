#include "mission_store.h"

#include <Preferences.h>

namespace mission_store {

static Preferences prefs;
static bool ok_ = false;
static uint32_t writes_ = 0;
static const char *kKey = "mision";
static const char *kKeyModo = "modo";  // 'A', 'B' o 'C'

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

bool loadMode(DeployMode &m) {
  if (!ok_ || !prefs.isKey(kKeyModo)) return false;
  return deployModeFromChar((char)prefs.getUChar(kKeyModo, 0), m);
}

bool saveMode(DeployMode m) {
  if (!ok_) return false;
  writes_++;
  return prefs.putUChar(kKeyModo, (uint8_t)deployModeChar(m)) == 1;
}

}  // namespace mission_store
