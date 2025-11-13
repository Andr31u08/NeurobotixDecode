package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadWrapper {
    Gamepad lastGamepad, currentGamepad;

    public GamepadWrapper(Gamepad gamepad) {
        currentGamepad = new Gamepad();
        lastGamepad = new Gamepad();
        update(gamepad);
    }

    public void update(Gamepad newGamepad) {
        lastGamepad.copy(currentGamepad);
        currentGamepad.copy(newGamepad);
    }

    public boolean pressed(boolean currentButtonState, boolean lastButtonState) {
        return currentButtonState && !lastButtonState;
    }

    public boolean hold(boolean currentButtonState) {
        return currentButtonState;
    }

    public boolean isAPressed() {
        return pressed(currentGamepad.a, lastGamepad.a);
    }

    // B button
    public boolean isBPressed() {
        return pressed(currentGamepad.b, lastGamepad.b);
    }

    // X button
    public boolean isXPressed() {
        return pressed(currentGamepad.x, lastGamepad.x);
    }

    // Y button
    public boolean isYPressed() {
        return pressed(currentGamepad.y, lastGamepad.y);
    }

    // Dpad up
    public boolean isDpadUpPressed() {
        return pressed(currentGamepad.dpad_up, lastGamepad.dpad_up);
    }

    // Dpad down
    public boolean isDpadDownPressed() {
        return pressed(currentGamepad.dpad_down, lastGamepad.dpad_down);
    }

    // Dpad left
    public boolean isDpadLeftPressed() {
        return pressed(currentGamepad.dpad_left, lastGamepad.dpad_left);
    }

    // Dpad right
    public boolean isDpadRightPressed() {
        return pressed(currentGamepad.dpad_right, lastGamepad.dpad_right);
    }

    // Left bumper
    public boolean isLeftBumperPressed() {
        return pressed(currentGamepad.left_bumper, lastGamepad.left_bumper);
    }

    // Right bumper
    public boolean isRightBumperPressed() {
        return pressed(currentGamepad.right_bumper, lastGamepad.right_bumper);
    }

    // Left stick button
    public boolean isLeftStickPressed() {
        return pressed(currentGamepad.left_stick_button, lastGamepad.left_stick_button);
    }

    // Right stick button
    public boolean isRightStickPressed() {
        return pressed(currentGamepad.right_stick_button, lastGamepad.right_stick_button);
    }

    // Start button
    public boolean isStartPressed() {
        return pressed(currentGamepad.start, lastGamepad.start);
    }

    // Back button
    public boolean isBackPressed() {
        return pressed(currentGamepad.back, lastGamepad.back);
    }

    // A button hold
    public boolean isAHeld() {
        return hold(currentGamepad.a);
    }

    // B button hold
    public boolean isBHeld() {
        return hold(currentGamepad.b);
    }

    // X button hold
    public boolean isXHeld() {
        return hold(currentGamepad.x);
    }

    // Y button hold
    public boolean isYHeld() {
        return hold(currentGamepad.y);
    }

    // Dpad up hold
    public boolean isDpadUpHeld() {
        return hold(currentGamepad.dpad_up);
    }

    // Dpad down hold
    public boolean isDpadDownHeld() {
        return hold(currentGamepad.dpad_down);
    }

    // Dpad left hold
    public boolean isDpadLeftHeld() {
        return hold(currentGamepad.dpad_left);
    }

    // Dpad right hold
    public boolean isDpadRightHeld() {
        return hold(currentGamepad.dpad_right);
    }

    // Left bumper hold
    public boolean isLeftBumperHeld() {
        return hold(currentGamepad.left_bumper);
    }

    // Right bumper hold
    public boolean isRightBumperHeld() {
        return hold(currentGamepad.right_bumper);
    }

    // Left stick button hold
    public boolean isLeftStickHeld() {
        return hold(currentGamepad.left_stick_button);
    }

    // Right stick button hold
    public boolean isRightStickHeld() {
        return hold(currentGamepad.right_stick_button);
    }

    // Start button hold
    public boolean isStartHeld() {
        return hold(currentGamepad.start);
    }

    // Back button hold
    public boolean isBackHeld() {
        return hold(currentGamepad.back);
    }

    public double leftStickX() {
        return currentGamepad.left_stick_x;
    }
    public double leftStickY() {
        return currentGamepad.left_stick_y;
    }

    public double rightTrigger() {
        return currentGamepad.right_trigger;
    }
    public double leftTrigger() {
        return currentGamepad.left_trigger;
    }
}