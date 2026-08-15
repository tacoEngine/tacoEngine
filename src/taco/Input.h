// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef TACO_INPUT_H
#define TACO_INPUT_H

#include <raylib.h>
#include <string>
#include <vector>


namespace taco {
class Input {
protected:
    enum class KeyState {
        Pressed,
        PressedRepeat,
        Down,
        Released,
        Up
    };

    struct RegisteredKey {
        std::size_t hash;
        int code; // KeyboardKey or MouseButton, depending on `mouse`
        bool mouse;
        KeyState state;
    };

    struct KeyEvent {
        std::size_t hash;
        std::uint64_t time;
        KeyState state;
    };

    // The mouse is sampled every update, so a delta is the difference between
    // two consecutive events and stays correct when the mouse doesn't move.
    struct MouseEvent {
        std::uint64_t time;
        Vector2 position;
        float wheel;
    };

    std::vector<RegisteredKey> registered_keys_;
    std::vector<KeyEvent> events_;
    std::vector<MouseEvent> mouse_events_;
    std::uint64_t time_ = 0;

    RegisteredKey *GetKey(std::string_view ident);
    KeyState GetKeyState(std::string_view ident);
    KeyState GetKeyState(RegisteredKey *key);
    // `back` = 0 is the newest event at or before the current time, 1 the one before that.
    const MouseEvent *GetMouseEvent(int back = 0) const;
public:
    virtual ~Input() = default;

    void SetTime(std::uint64_t time);

    // Keys and mouse buttons share one ident namespace, so all IsKey* queries
    // below work for either.
    void RegisterKey(std::string_view ident, KeyboardKey code);
    void RegisterButton(std::string_view ident, MouseButton code);

    void UpdateFromLocalInput();

    bool IsKeyPressed(std::string_view ident);
    bool IsKeyPressedRepeat(std::string_view ident);
    bool IsKeyDown(std::string_view ident);
    bool IsKeyReleased(std::string_view ident);
    bool IsKeyUp(std::string_view ident);

    Vector2 GetMousePosition() const;
    Vector2 GetMouseDelta() const;
    float GetMouseWheel() const;
};
} // taco


#endif //TACO_INPUT_H
