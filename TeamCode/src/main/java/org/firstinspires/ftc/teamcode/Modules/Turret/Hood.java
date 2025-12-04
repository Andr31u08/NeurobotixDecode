package org.firstinspires.ftc.teamcode.Modules.Turret;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Limelight;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

public class Hood {

    private final BetterServo hood;
    private final Servo hoodServo;
    private final Limelight limelight;
    private static double startPos = 0;
    public double angle, servoPosition, n;
    public double speed = 0.1 * Flywheel.getTargetVelocity();
    public boolean isAchievable;
    public final double MAX_ANGLE = 45;
    public final double PI = 3.1415926;
    public final double CIRCLE_FRACTION = MAX_ANGLE/360;

    //TODO: derive the ecuation for the hood movement !!!

    public Hood (HardwareMap hardwareMap)
    {
        hoodServo = hardwareMap.get(Servo.class, "HoodServo");
        hood = new BetterServo("HoodServo", hoodServo, BetterServo.RunMode.PROFILE, startPos, false);
        limelight = new Limelight(hardwareMap, 0);
    }
    public double calculateAngle(double distance)
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

    public void setServoPos(double Angle, BetterServo servo) {
        servoPosition = 1 * Angle / MAX_ANGLE;
        servo.setPosition(servoPosition);
    }

    public double distanceToTarget (Limelight limelight, boolean isRedAlliance) {
        double X = 0, distance = 0;
        int latest = limelight.getFiducialId();
        if((latest == 20 && !isRedAlliance) || (latest == 24 && isRedAlliance)) {
            X = limelight.getX();
            if (X == 0)
                X+=0.1;
            distance = (1-0.315) / Math.tan(PI/2 + Math.toRadians(X));
            return distance;
        }
        else
            return 0;
    }
    public BetterServo getServo() {return hood;}
}
