package org.firstinspires.ftc.teamcode.Modules.Turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Math.PID;
import org.firstinspires.ftc.teamcode.Robot.DualMPCRServo;

public class TurretController {

    private final CRServo turretServo1;
    private final CRServo turretServo2;
    private final DcMotorEx turretEncoder;
    private final PID pidController;
    private final DualMPCRServo turretController;

    private double effectiveTargetAngle = 0.0;

    private static final double kP = 0.22, kI = 0, kD = 0.03;

    public TurretController(HardwareMap hardwareMap) {
        turretServo1 = hardwareMap.get(CRServo.class, "turretServo1");
        turretEncoder = hardwareMap.get(DcMotorEx.class, "backLeft");
        turretServo2 = hardwareMap.get(CRServo.class, "turretServo2");

        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        pidController = new PID(kP, kI, kD, 0);
        turretController = new DualMPCRServo(turretServo1, turretEncoder, turretServo2, pidController, false);
    }


    // Added a change!
    public void setTargetAngle(double angle) {
        double delta = normalizeAngle(angle - getCurrentAngle());
        if (delta > 180) angle -= 360;
        else if (delta < -180) angle += 360;

        effectiveTargetAngle = normalizeAngle(angle);
        turretController.setTarget(effectiveTargetAngle);
    }

    public void update() {
        turretController.update();
    }

    public double getCurrentAngle() {
        return normalizeAngle(turretController.getEncoderAngle());
    }

    public double getEffectiveTargetAngle() {
        return effectiveTargetAngle;
    }

    public double getEncoderPosition() {
        return turretController.encoderPosition();
    }

    private double normalizeAngle(double angle) {
        while (angle >= 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
}
