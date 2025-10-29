package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Wrappers.MPCRServo;

public class DualMPCRServo {
    private final MPCRServo servoA;
    private final MPCRServo servoB;
    private final boolean servoBReversed;

    public DualMPCRServo(CRServo a, DcMotorEx encA, CRServo b, DcMotorEx encB,
                         BetterMotionProfile profileA, BetterMotionProfile profileB,
                         PIDController pidA, PIDController pidB,
                         double kV, boolean servoBReversed) {
        this.servoA = new MPCRServo(a, encA, profileA, pidA, kV);
        this.servoB = new MPCRServo(b, encB, profileB, pidB, kV);
        this.servoBReversed = servoBReversed;
    }

    public void startProfile(double targetAngleDeg) {
        servoA.startProfile(targetAngleDeg);
        servoB.startProfile(servoBReversed ? -targetAngleDeg : targetAngleDeg);
    }

    public void update() {
        servoA.update();
        servoB.update();
    }

    public double getAverageAngle() {
        return (servoA.getEncoderAngle() + (servoBReversed ? -servoB.getEncoderAngle() : servoB.getEncoderAngle())) / 2.0;
    }

    public boolean isActive() {
        return servoA.isActive() || servoB.isActive();
    }
}
