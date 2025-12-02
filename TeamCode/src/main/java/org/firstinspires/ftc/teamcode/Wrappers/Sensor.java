package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class Sensor {
    private final NormalizedColorSensor sensor;
    public static float r, g, b;

    public Sensor(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
    }

    public void updateColors() {
        NormalizedRGBA colors = sensor.getNormalizedColors();  // READ THE SENSOR HERE

        r = colors.red;
        g = colors.green;
        b = colors.blue;
    }

    public boolean isPurple() {
        return (r > 0.0014 && g < 0.003 && b > 0.003)
                && (b > r && b > g);
    }

    public boolean isGreen() {
        return (r < 0.001 && g > 0.0025 && b > 0.002)
                && (g > r && g > b);
    }

    public double RedAmount() { return r; }
    public double GreenAmount() { return g; }
    public double BlueAmount() { return b; }
}