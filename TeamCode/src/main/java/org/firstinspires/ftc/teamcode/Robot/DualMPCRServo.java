package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.Math.PID;

public class DualMPCRServo {
    private final CRServo servoA;
    private final CRServo servoB;
    private final DcMotorEx enc;
    private final boolean servoBReversed;
    private final PID pid;
    private double targetAngle;
    private double ticksPerRev = 8192.0;

    public static double kp = 0.07, ki = 0, kd = 0.01, f = 0.01;
    private int nr_teeth_enc_gear = 16;
    private int nr_teeth_turret_gear = 67;

    public DualMPCRServo(CRServo a, DcMotorEx enc, CRServo b, PID pid, boolean servoBReversed) {
        this.servoA = a;
        this.servoB = b;
        this.servoBReversed = servoBReversed;
        this.pid = pid;
        this.enc = enc;
    }

    public void setTarget(double targetAngle) {
        this.targetAngle = normalizeAngle(targetAngle);
    }

    public void update() {
        double currentAngle = getEncoderAngle();
        double error = normalizeAngle(targetAngle - currentAngle);
        double power = pid.update(currentAngle + error, currentAngle, kp, ki, kd, f);
        power = clamp(power, -1, 1);

        servoA.setPower(power);
        servoB.setPower(servoBReversed ? -power : power);
    }

    private double normalizeAngle(double angle) {
        while (angle >= 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public double getEncoderAngle() {
        return enc.getCurrentPosition() * (360.0 / ticksPerRev) * ((double)nr_teeth_enc_gear/nr_teeth_turret_gear);
    }

    public double encoderPosition() {
        return enc.getCurrentPosition();
    }
}
