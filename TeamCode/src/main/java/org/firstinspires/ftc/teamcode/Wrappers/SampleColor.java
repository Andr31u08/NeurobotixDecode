package org.firstinspires.ftc.teamcode.Wrappers;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

public class SampleColor {

    public enum State {
        RED, BLUE, YELLOW, GREEN, PURPLE
    }

    public State state = State.BLUE;

    public float redError, yellowError, blueError, greenError, purpleError;

    public ColorSensor distanceSensor;
    public ColorSensor colorSensor;

    public float red = 0, blue = 0, green = 0;

    public SampleColor(HardwareMap hardwareMap) {
        distanceSensor = hardwareMap.get(ColorSensor.class, "sensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor");
    }

    private void updateColor() {
        red   = colorSensor.red();
        green = colorSensor.green();
        blue  = colorSensor.blue();
    }

    public float distance(float r1, float g1, float b1,
                          float r2, float g2, float b2) {
        return (float)Math.sqrt(
                (r1 - r2) * (r1 - r2) +
                        (g1 - g2) * (g1 - g2) +
                        (b1 - b2) * (b1 - b2)
        );
    }

    private void updateState() {
        // TARGET COLOR CENTERS (tune these from telemetry)
        // These are reasonable starting points.
        float[] RED_T     = {255,   0,   0};
        float[] BLUE_T    = {  0,   0, 255};
        float[] YELLOW_T  = {230, 230,   0};
        float[] GREEN_T   = {  0, 255,   0};
        float[] PURPLE_T  = {160,   0, 160};   // red + blue mix

        redError    = distance(red, green, blue, RED_T[0],    RED_T[1],    RED_T[2]);
        yellowError = distance(red, green, blue, YELLOW_T[0], YELLOW_T[1], YELLOW_T[2]);
        blueError   = distance(red, green, blue, BLUE_T[0],   BLUE_T[1],   BLUE_T[2]);
        greenError  = distance(red, green, blue, GREEN_T[0],  GREEN_T[1],  GREEN_T[2]);
        purpleError = distance(red, green, blue, PURPLE_T[0], PURPLE_T[1], PURPLE_T[2]);

        // PICK LOWEST ERROR
        float min = Math.min(
                Math.min(Math.min(redError, yellowError),
                        Math.min(blueError, greenError)),
                purpleError
        );

        if (min == redError)    state = State.RED;
        if (min == yellowError) state = State.YELLOW;
        if (min == blueError)   state = State.BLUE;
        if (min == greenError)  state = State.GREEN;
        if (min == purpleError) state = State.PURPLE;
    }

    public boolean isGreen() {return (state == state.GREEN);}
    public boolean isPurple() {return (state == state.PURPLE);}

    public void update() {
        updateColor();
        updateState();
    }
}
