package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Math.PIDController;
//import org.firstinspires.ftc.teamcode.Math.PIDController;

public class MPCRServo {

    private final CRServo servo;
    private final DcMotorEx encoder;
    private final BetterMotionProfile profile;
    private final PIDController pid; // Injected PID

    private final double kV;
    private boolean active = false;

    //TODO: Find ticks per rev
    private double ticksPerRev = 8192.0;

    public MPCRServo(CRServo servo, DcMotorEx encoder,
                     BetterMotionProfile profile,
                     PIDController pid, double kV) {
        this.servo = servo;
        this.encoder = encoder;
        this.profile = profile;
        this.pid = pid;
        this.kV = kV;
    }

    public void startProfile(double targetAngleDeg) {
        double current = getEncoderAngle();
        profile.setMotion(current, targetAngleDeg, 0);
        active = true;
        pid.reset();
    }

    public void update() {
        if (!active) {
            servo.setPower(0);
            return;
        }

        double currPos = getEncoderAngle();
        profile.update();
        double targetPos = profile.getPosition();
        double targetVel = profile.getVelocity() * sign(targetPos - getEncoderAngle());

        double pidOutput = pid.calculate(targetPos, getEncoderAngle());
        double power = pidOutput + kV * targetVel;

        power = clamp(power, -1, 1);

        servo.setPower(power);

        if (profile.getTimeToMotionEnd() <= 0) {
            active = false;
            servo.setPower(0);
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
        return encoder.getCurrentPosition() * (360.0 / ticksPerRev);
    }

    public boolean isActive() {
        return active;
    }
}
