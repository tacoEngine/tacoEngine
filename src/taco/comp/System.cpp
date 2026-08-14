// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "System.h"

#include "taco/Entity.h"

namespace taco {
void System::UpdateEarly(Engine *, Entity) {}
void System::UpdatePrePhysics(Engine *, Entity) {}
void System::UpdatePostPhysics(Engine *, Entity) {}
void System::UpdateLate(Engine *, Entity) {}
void System::UpdateUI(Engine *, Entity) {}
}
