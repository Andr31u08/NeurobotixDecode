package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorDetector {
    private final NormalizedColorSensor sensor;
    private final float[] hsv = new float[3];

    public ColorDetector(NormalizedColorSensor sensor) {
        this.sensor = sensor;
    }

    public void update() {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        android.graphics.Color.colorToHSV(colors.toColor(), hsv);
    }

    public boolean isPurple() {
        // general purple
        return inRange(hsv[0], 260, 300) && hsv[1] > 0.3 && hsv[2] > 0.2;
    }

    public boolean isLightPurple() {
        // lighter (higher value)
        return inRange(hsv[0], 260, 300) && hsv[1] > 0.2 && hsv[2] > 0.6;
    }

    public boolean isDarkPurple() {
        // darker (lower value)
        return inRange(hsv[0], 260, 300) && hsv[1] > 0.4 && hsv[2] < 0.4;
    }

    public boolean isRed() {
        return (inRange(hsv[0], 0, 30) || inRange(hsv[0], 330, 360)) && hsv[1] > 0.3;
    }

    public boolean isBlue() {
        return inRange(hsv[0], 180, 240) && hsv[1] > 0.3;
    }

    public boolean isGreen() {
        return inRange(hsv[0], 90, 150) && hsv[1] > 0.3;
    }

    public float getHue() { return hsv[0]; }
    public float getSaturation() { return hsv[1]; }
    public float getValue() { return hsv[2]; }

    private boolean inRange(float value, float min, float max) {
        return value >= min && value <= max;
    }
}
