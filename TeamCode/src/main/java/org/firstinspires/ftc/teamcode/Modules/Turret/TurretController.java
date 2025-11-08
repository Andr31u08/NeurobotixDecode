package org.firstinspires.ftc.teamcode.Modules.Turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Robot.DualMPCRServo;

/**
 * TurretController
 * ----------------
 * Controls a CRServo-driven turret using hysteresis logic and PID control.
 * Prevents violent flipping at +/-180 degrees by extending the valid range.
 */
public class TurretController {

    // ----------------------- CONFIGURATION CONSTANTS -----------------------

    // Mechanical limits of turret rotation (degrees)
    private static final double MAX_ANGLE = 180.0;
    private static final double MIN_ANGLE = -180.0;

    // Hysteresis buffer (degrees) — extends effective range
    private static final double HYSTERESIS_BUFFER = 20.0;

    // Derived limits
    private static final double EXTENDED_MAX = MAX_ANGLE + HYSTERESIS_BUFFER;
    private static final double EXTENDED_MIN = MIN_ANGLE - HYSTERESIS_BUFFER;

    //TODO: REV encoder counts per revolution (REV Through Bore = 8192)
    private static final double TICKS_PER_REV = 8192.0;

    //TODO: Conversion factor: ticks → degrees
    private static final double TICKS_TO_DEGREES = 360.0 / TICKS_PER_REV;

    //TODO: PID coefficients — tune as needed
    private static final double kP1 = 0.015;
    private static final double kI1 = 0.0000;
    private static final double kD1 = 0.001;

    private static final double kP2 = 0.015;
    private static final double kI2 = 0.0000;
    private static final double kD2 = 0.001;

    // Feedforward (voltage-to-speed relationship)
    // Start around 1 / maxDegreesPerSecond (empirically measured)
    private static final double kV = 1.0 / 300.0;

    //TODO: Motion profile limits (deg/s and deg/s^2)
    private static final double MAX_VEL1 = 300.0;
    private static final double ACCEL1 = 200.0;
    private static final double DECEL1 = 250.0;

    private static final double MAX_VEL2 = 300.0;
    private static final double ACCEL2 = 200.0;
    private static final double DECEL2 = 250.0;

    // ----------------------- INSTANCE VARIABLES -----------------------

    private final CRServo turretServo1;
    private final CRServo turretServo2;
    private final DcMotorEx turretEncoder1;
    private final DcMotorEx turretEncoder2;

    private final BetterMotionProfile motionProfile1;
    private final BetterMotionProfile motionProfile2;
    private final PIDController pidController1;
    private final PIDController pidController2;
    private final DualMPCRServo turretController;

    //TODO: Get desired angle

    private double currentAngle = 0;
    private double desiredAngle = 0.0;
    private double effectiveTargetAngle = 0.0;

    // ----------------------- CONSTRUCTOR -----------------------

    public TurretController(HardwareMap hardwareMap) {
        // Initialize hardware
        turretServo1 = hardwareMap.get(CRServo.class, "turretServo1");
        turretEncoder1 = hardwareMap.get(DcMotorEx.class, "turretEncoder1");

        turretServo2 = hardwareMap.get(CRServo.class, "turretServo2");
        turretEncoder2 = hardwareMap.get(DcMotorEx.class, "turretEncoder2");

        turretEncoder1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize control components
        pidController1 = new PIDController(kP1, kI1, kD1);
        pidController2 = new PIDController(kP2, kI2, kD2);
        motionProfile1 = new BetterMotionProfile(MAX_VEL1, ACCEL1, DECEL1);
        motionProfile2 = new BetterMotionProfile(MAX_VEL2, ACCEL2, DECEL2);

        // Combined motion profile + PID + feedforward control
        turretController = new DualMPCRServo(turretServo1, turretEncoder1, turretServo2, turretEncoder2, motionProfile1, motionProfile2, pidController1, pidController2, kV, false);

        // Reset both encoders to zero and configure for raw position tracking
        turretEncoder1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        turretEncoder1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        turretEncoder2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

    }

    // ----------------------- MAIN CONTROL FUNCTION -----------------------

    public void setTargetAngle(double angle)
    {
        desiredAngle = applyHysteresis(angle);
        currentAngle = getTurretAngle();
        effectiveTargetAngle = normalizeAngle(desiredAngle);

        double targetRaw = effectiveTargetAngle;
        turretController.startProfile(targetRaw);
    }

    /**
     * Called every loop cycle to updateDistance turret power.
     * @param desiredAngle The target angle (in degrees), from vision or driver input.
     */
    /*public void update(double desiredAngle) {
        currentAngle = getTurretAngle();
        turretController.update();
    }*/

    // ----------------------- ANGLE TRACKING -----------------------

    /**
     * Returns the turret’s current angle in degrees using the REV encoder.
     */
    private double getTurretAngle() {
        return normalizeAngle(turretController.getAverageAngle());
    }

    // ----------------------- HYSTERESIS LOGIC -----------------------

    /**
     * Maps the desired angle to an equivalent target angle that avoids discontinuous flipping.
     */
    private double applyHysteresis(double targetAngle) {
        double delta = normalizeAngle(targetAngle - currentAngle);

        // Pick the closest equivalent target (handles wrap-around)
        if (delta > 180) targetAngle -= 360;
        else if (delta < -180) targetAngle += 360;

        // Keep within extended hysteresis range
        if (targetAngle > EXTENDED_MAX) targetAngle -= 360;
        if (targetAngle < EXTENDED_MIN) targetAngle += 360;

        return normalizeAngle(targetAngle);
    }

    // ----------------------- UTILITY FUNCTIONS -----------------------

    /**
     * Wraps any angle into the [-180, 180) range.
     */
    private double normalizeAngle(double angle) {
        while (angle >= 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    /**
     * Returns the current turret angle for telemetry.
     */
    public double getCurrentAngle() {
        return currentAngle;
    }

    /**
     * Returns the current desired target angle after hysteresis mapping.
     */
    public double getEffectiveTargetAngle() {
        return effectiveTargetAngle;
    }

    // ----------------------- PID CONTROLLER -----------------------

    /**
     * Lightweight, loop-stable PID implementation.
     */
}
