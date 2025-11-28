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
        return (r < 0.4 && g < 0.3 && b > 0.5)
                && (b > r && b > g);
    }

    public boolean isGreen() {
        return (r < 0.3 && g > 0.5 && b < 0.4)
                && (g > r && g > b);
    }

    public double RedAmount() { return r; }
    public double GreenAmount() { return g; }
    public double BlueAmount() { return b; }
}
