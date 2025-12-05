package org.firstinspires.ftc.teamcode.Modules.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Math.PID;
import org.firstinspires.ftc.teamcode.Robot.DualMPCRServo;
import org.firstinspires.ftc.teamcode.Robot.Limelight;

public class TurretController {

    private final CRServo turretServo1;
    private final CRServo turretServo2;
    private final DcMotorEx turretEncoder;
    private final PID controller;
    private final Limelight limelight;
    //private final DualMPCRServo turretController;

    private double effectiveTargetAngle = 0.0;

    public static double kp = 0.07, ki = 0, kd = 0.01, f = 0.01;
    private double ticksPerRev = 8192.0;
    private int nr_teeth_enc_gear = 16;
    private int nr_teeth_turret_gear = 67;

    private double followingPivotPose = 0;

    private boolean following = false;
    private boolean redAllTrue;

    public TurretController(HardwareMap hardwareMap, boolean redAllTrue) {
        turretServo1 = hardwareMap.get(CRServo.class, "turretServo1");
        turretEncoder = hardwareMap.get(DcMotorEx.class, "backLeft");
        turretServo2 = hardwareMap.get(CRServo.class, "turretServo2");

        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        this.redAllTrue = redAllTrue;

        limelight = new Limelight(hardwareMap, 0);
        controller = new PID(kp, ki, kd, 0);
        //turretController = new DualMPCRServo(turretServo1, turretEncoder, turretServo2, pidController, false);
    }

    public void checkForTowerTag() {
        //TODO: Aici ai direct ce iti trebuie ca sa verifici following din llTargetAngle;
        if (limelight.patternCheck())
            if ((limelight.isRedAlliance() && redAllTrue) ||
                    (limelight.isBlueAlliance() && !redAllTrue))
                following = true;
            else following = false;
    }

    public void llTargetAngle() {
        limelight.update();
        if (!following) {
            followingPivotPose = getEncoderAngle();
            effectiveTargetAngle = 0;
            //following = true;
        }
        else {
            double error = limelight.X;
            double targetAngle = followingPivotPose + error;
            if (targetAngle > 100) effectiveTargetAngle = 100;
            else if (targetAngle < -100) effectiveTargetAngle = -100;
            else effectiveTargetAngle = targetAngle;
        }
        /*double delta = normalizeAngle(angle - getCurrentAngle());
        if (delta > 180) angle -= 360;
        else if (delta < -180) angle += 360;

        effectiveTargetAngle = normalizeAngle(angle);*/
        //turretController.setTarget(effectiveTargetAngle);
    }

    /*public void defaultTargetAngle() {
        if (!following)
            effectiveTargetAngle = 0;
    }*/

    /*public void limitTarget() {
        if (effectiveTargetAngle > 100) effectiveTargetAngle = 100;
        if (effectiveTargetAngle < -100) effectiveTargetAngle = -100;
    }*/

    public void update() {
        //turretController.update();
        checkForTowerTag();
        llTargetAngle();
        //limitTarget();
        double currpos = getEncoderAngle();
        double value = controller.update(effectiveTargetAngle, currpos, kp, ki, kd, 0);
        turretServo1.setPower(value);
        turretServo2.setPower(value);

    }

    /*public double getCurrentAngle() {
        return normalizeAngle(turretController.getEncoderAngle());
    }*/

    public double getEffectiveTargetAngle() {
        return effectiveTargetAngle;
    }

    /*public double getEncoderPosition() {
        return turretController.encoderPosition();
    }*/

    /*private double normalizeAngle(double angle) {
        while (angle >= 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }*/

    /*private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }*/

    public double getEncoderAngle() {
        return turretEncoder.getCurrentPosition() * (360.0 / ticksPerRev) * ((double)nr_teeth_enc_gear/nr_teeth_turret_gear);
    }

    public double encoderPosition() {
        return turretEncoder.getCurrentPosition();
    }
}
