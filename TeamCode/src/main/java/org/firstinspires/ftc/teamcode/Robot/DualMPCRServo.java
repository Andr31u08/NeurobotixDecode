package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Wrappers.MPCRServo;

public class DualMPCRServo {
    private final CRServo servoA;
    private final CRServo servoB;
    private final DcMotorEx enc;
    private final boolean servoBReversed;
    private final BetterMotionProfile profile;
    private final PIDController pid;
    private boolean active = false;
    private double kV;
    //TODO: complete ticks per rev
    private double ticksPerRev = 8000.0;
    public DualMPCRServo(CRServo a, DcMotorEx enc, CRServo b,
                         BetterMotionProfile profile,
                         PIDController pid,
                         double kV, boolean servoBReversed) {
        this.servoA = a;
        this.servoB = b;
        this.servoBReversed = servoBReversed;
        this.pid = pid;
        this.profile = profile;
        this.kV = kV;
        this.enc = enc;
    }

    public void startProfile(double targetAngleDeg) {
        double current = getEncoderAngle();
        profile.setMotion(current, targetAngleDeg, 0);
        active = true;
        pid.reset();
    }

    public void update() {
        if (!active) {
            servoA.setPower(0);
            servoB.setPower(0);
            return;
        }

        double currPos = getEncoderAngle();
        profile.update();
        double targetPos = profile.getPosition();
        double targetVel = profile.getVelocity() * sign(targetPos - getEncoderAngle());

        double pidOutput = pid.calculate(targetPos, getEncoderAngle());
        double power = pidOutput + kV * targetVel;

        power = clamp(power, -1, 1);

        servoB.setPower(power);
        servoA.setPower(power);

        if (profile.getTimeToMotionEnd() <= 0) {
            active = false;
            servoA.setPower(0);
            servoB.setPower(0);
        }
    }

    private double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    private double sign(double value) {
        if (value > 0) return 1.0;
        if (value < 0) return -1.0;
        return 0.0;
    }

    public double getEncoderAngle() {
        return enc.getCurrentPosition() * (360.0 / ticksPerRev);
    }

    public boolean isActive() {
        return active;
    }
}
