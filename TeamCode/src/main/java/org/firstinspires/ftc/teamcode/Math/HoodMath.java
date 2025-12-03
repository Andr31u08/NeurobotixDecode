package org.firstinspires.ftc.teamcode.Math;

public class HoodMath {
    //TODO distance will be calculated using LimeLight
    public double distance, angle, speed, RoC;
    public int n;
    public boolean isAchievable;
    //TODO Do the math for the maximum angle the hood can have. Should be around 45 degrees
    public final double MAX_ANGLE = 45;
    public final double PI = 3.1415926;
    public final double CIRCLE_FRACTION = MAX_ANGLE/360;
    public void calculateAngle()
    {
        angle = Math.asin((distance*9.81)/(speed*speed)) * 90 / PI;
        if(angle < 0)
            n++;
        if(n%2 == 0)
            angle = angle + n * PI;
        else
            angle = n * PI - angle;
        if(angle > MAX_ANGLE)
            isAchievable = false;
        else
            isAchievable = true;

    }

    // TODO Do the math for the rate of change of the angle.
    public double rateOfChange() {
        RoC = 0;
        // Distance travelled by furthest point of the hood / Distance travelled by the hood's base
        return RoC;
    }

    //TODO Do the math for needed position, after calculating required angle.
    public void updatePosition () {}
}
