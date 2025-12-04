package org.firstinspires.ftc.teamcode.Math;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Modules.Turret.Flywheel;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.Robot.Limelight;

import java.util.List;

public abstract class HoodMath {
    //TODO distance will be calculated using LimeLight
    public double angle, servoPosition, n;
    public double speed = 0.1 * Flywheel.getTargetVelocity();

    public boolean isAchievable;
    //TODO Do the math for the maximum angle the hood can have. Should be around 45 degrees
    public final double MAX_ANGLE = 45;
    public final double PI = 3.1415926;
    public final double CIRCLE_FRACTION = MAX_ANGLE/360;
    public Servo hood = null;
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

    public void setServoPos(double Angle) {
        servoPosition = 1 * Angle / MAX_ANGLE;
        hood.setPosition(servoPosition);
    }
    public double distanceToTarget (Limelight3A limelight) {

        LLResult latest = limelight.getLatestResult();
        if (latest == null) return 0;
        List<LLResultTypes.FiducialResult> fiducials = latest.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return 0;
        LLResultTypes.FiducialResult fr = fiducials.get(0);
        double X = fr.getTargetXDegrees();
        double distance = (1-0.315) / Math.tan(PI/2 + Math.toRadians(X));
        return distance;
    }
}
