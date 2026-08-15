// tacoEngine (c) Nikolas Wipper 2026

#include "Input.h"

#include <algorithm>
#include <raylib.h>

namespace taco {

void Input::SetTime(std::uint64_t time) {
    time_ = time;
}

Input::RegisteredKey *Input::GetKey(std::string_view ident) {
    std::size_t ident_hash = std::hash<std::string>()(std::string(ident));

    for (auto &registered : registered_keys_) {
        if (registered.hash == ident_hash) {
            return &registered;
        }
    }
    return nullptr;
}


Input::KeyState Input::GetKeyState(std::string_view ident) {
    return GetKeyState(GetKey(ident));
}

Input::KeyState Input::GetKeyState(RegisteredKey *key) {
    // ponytail: linear scan back through the log; cheap because queries hit the
    // tail, replace with a per-key cursor if seeking far back ever gets hot.
    for (auto it = events_.rbegin(); it != events_.rend(); ++it) {
        if (it->time <= time_ && it->hash == key->hash) {
            return it->state;
        }
    }

    return key->state;
}

const Input::MouseEvent *Input::GetMouseEvent(int back) const {
    for (auto it = mouse_events_.rbegin(); it != mouse_events_.rend(); ++it) {
        if (it->time <= time_ && back-- == 0) {
            return &*it;
        }
    }
    return nullptr;
}

void Input::RegisterKey(std::string_view ident, KeyboardKey code) {
    std::size_t ident_hash = std::hash<std::string>()(std::string(ident));

    registered_keys_.emplace_back(ident_hash, code, false, KeyState::Up);
}

void Input::RegisterButton(std::string_view ident, MouseButton code) {
    std::size_t ident_hash = std::hash<std::string>()(std::string(ident));

    registered_keys_.emplace_back(ident_hash, code, true, KeyState::Up);
}

void Input::UpdateFromLocalInput() {
    for (auto &registered : registered_keys_) {
        KeyState state;
        if (registered.mouse) {
            // raylib has no repeat for mouse buttons
            if (::IsMouseButtonPressed(registered.code)) {
                state = KeyState::Pressed;
            } else if (::IsMouseButtonDown(registered.code)) {
                state = KeyState::Down;
            } else if (::IsMouseButtonReleased(registered.code)) {
                state = KeyState::Released;
            } else {
                state = KeyState::Up;
            }
        } else if (::IsKeyPressedRepeat(registered.code)) {
            state = KeyState::PressedRepeat;
        } else if (::IsKeyPressed(registered.code)) {
            state = KeyState::Pressed;
        } else if (::IsKeyDown(registered.code)) {
            state = KeyState::Down;
        } else if (::IsKeyReleased(registered.code)) {
            state = KeyState::Released;
        } else {
            state = KeyState::Up;
        }

        if (GetKeyState(&registered) != state) {
            KeyEvent event = {registered.hash, time_, state};
            auto it = std::ranges::upper_bound(events_, event.time, {}, &KeyEvent::time);
            events_.insert(it, event);
        }
    }

    MouseEvent mouse = {time_, ::GetMousePosition(), ::GetMouseWheelMove()};
    auto it = std::ranges::upper_bound(mouse_events_, mouse.time, {}, &MouseEvent::time);
    mouse_events_.insert(it, mouse);
}

bool Input::IsKeyPressed(std::string_view ident) {
    return GetKeyState(ident) == KeyState::Pressed;
}

bool Input::IsKeyPressedRepeat(std::string_view ident) {
    return GetKeyState(ident) == KeyState::PressedRepeat;
}

bool Input::IsKeyDown(std::string_view ident) {
    KeyState state = GetKeyState(ident);
    return state == KeyState::Pressed || state == KeyState::PressedRepeat || state == KeyState::Down;
}

bool Input::IsKeyReleased(std::string_view ident) {
    return GetKeyState(ident) == KeyState::Released;
}

bool Input::IsKeyUp(std::string_view ident) {
    KeyState state = GetKeyState(ident);
    return state == KeyState::Released || state == KeyState::Up;
}

Vector2 Input::GetMousePosition() const {
    const MouseEvent *current = GetMouseEvent();
    return current ? current->position : Vector2 {0, 0};
}

Vector2 Input::GetMouseDelta() const {
    const MouseEvent *current = GetMouseEvent(0), *previous = GetMouseEvent(1);
    if (!current || !previous) {
        return {0, 0};
    }
    return {current->position.x - previous->position.x, current->position.y - previous->position.y};
}

float Input::GetMouseWheel() const {
    const MouseEvent *current = GetMouseEvent();
    return current ? current->wheel : 0.0f;
}
} // taco
