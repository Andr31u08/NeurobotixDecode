package org.firstinspires.ftc.teamcode.Math;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Turret.Flywheel;

public class HoodMath {
    //TODO distance will be calculated using LimeLight
    public double distance, angle, servoPosition, n;
    public double speed = 0.1 * Flywheel.getTargetVelocity();
    public boolean isAchievable;
    //TODO Do the math for the maximum angle the hood can have. Should be around 45 degrees
    public final double MAX_ANGLE = 45;
    public final double PI = 3.1415926;
    public final double CIRCLE_FRACTION = MAX_ANGLE/360;
    public Servo hood = null;
    public double calculateAngle()
    {
        angle = Math.asin((distance*9.81)/(speed*speed)) * 90 / PI;
        if(angle < 0)
            n++;
        angle = n/2 * PI - angle;
        if (angle > MAX_ANGLE)
            isAchievable = false;
        else
            isAchievable = true;
        return angle;
    }

    public void setServoPos(double Angle) {
        servoPosition = 1 * Angle / MAX_ANGLE;
        hood.setPosition(servoPosition);
    }
}
