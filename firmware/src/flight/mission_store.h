// Registro de la misión en NVS (Preferences) para el reinicio en vuelo.
// Solo se usa desde setup() y desde la tarea de comunicaciones (la escritura en flash
// tarda milisegundos y no debe hacerse en el lazo de control).
#pragma once

#include <Arduino.h>

#include "state_machine.h"

namespace mission_store {
bool begin();
// true si hay un registro con el formato actual (magic y tamaño correctos).
bool load(MissionRecord &r);
bool save(const MissionRecord &r);
void clear();
uint32_t writes();
}  // namespace mission_store
