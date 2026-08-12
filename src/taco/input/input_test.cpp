// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

// Replays a recorded event log without a window, which is the same path a
// networked or recorded-input subclass takes.

#include <cassert>
#include <cstdio>

#include "taco/input/Input.h"

class ReplayInput : public taco::Input {
public:
    using taco::Input::KeyState;

    void Key(std::string_view ident, std::uint64_t time, KeyState state) {
        events_.emplace_back(std::hash<std::string>()(std::string(ident)), time, state);
    }

    void Mouse(std::uint64_t time, Vector2 position, float wheel = 0) {
        mouse_events_.emplace_back(time, position, wheel);
    }
};

int main() {
    ReplayInput input;
    input.RegisterKey("jump", KEY_SPACE);
    input.RegisterButton("aim", MOUSE_BUTTON_RIGHT);

    input.Key("jump", 10, ReplayInput::KeyState::Pressed);
    input.Key("jump", 20, ReplayInput::KeyState::Released);
    input.Key("aim", 15, ReplayInput::KeyState::Down);
    input.Mouse(10, {100, 100});
    input.Mouse(20, {110, 90}, 1);
    input.Mouse(30, {110, 90});

    // Before any event: registered defaults, no mouse movement.
    input.SetTime(0);
    assert(input.IsKeyUp("jump") && !input.IsKeyDown("jump"));
    assert(input.GetMouseDelta().x == 0 && input.GetMouseDelta().y == 0);

    input.SetTime(10);
    assert(input.IsKeyPressed("jump") && input.IsKeyDown("jump"));
    assert(input.IsKeyUp("aim"));

    input.SetTime(15);
    assert(input.IsKeyDown("jump") && input.IsKeyDown("aim"));

    input.SetTime(20);
    assert(input.IsKeyReleased("jump") && input.IsKeyUp("jump"));
    assert(input.GetMouseDelta().x == 10 && input.GetMouseDelta().y == -10);
    assert(input.GetMousePosition().x == 110 && input.GetMouseWheel() == 1);

    // Mouse stopped: the delta between two identical samples is zero.
    input.SetTime(30);
    assert(input.GetMouseDelta().x == 0 && input.GetMouseDelta().y == 0);
    assert(input.GetMouseWheel() == 0);

    // Seeking backwards has to give the same answer as the first time around.
    input.SetTime(15);
    assert(input.IsKeyDown("jump") && input.IsKeyDown("aim"));

    std::puts("input self-check ok");
    return 0;
}
