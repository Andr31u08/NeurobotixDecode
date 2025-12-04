package org.firstinspires.ftc.teamcode.Modules.Turret;

import static org.firstinspires.ftc.teamcode.Robot.Hardware.unlock;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;

public class Flywheel {

    private final DcMotorEx wheelMotor;
    //private final DcMotorEx encoder;
    private final BetterMotor flywheel;

    private static double power = 0; // TODO: Do I even need this?

    //TODO: set values to these !!!
    private static double onVelocity = 0;
    private static double targetVelocity = 0;

    public Flywheel (HardwareMap hardwareMap)
    {
        wheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        unlock(wheelMotor);
        //encoder = hardwareMap.get(DcMotorEx.class, "frontRight");

        flywheel = new BetterMotor(wheelMotor, BetterMotor.RunMode.PIDVELOCITY, true, wheelMotor, targetVelocity, false);
        flywheel.maxVelocityAtFullPower = 0; // TODO: Set this value !!!
        flywheel.setPidCoefficients(0, 0.5, 0);
    }

    public void updateFlyheel()
    {
        flywheel.update();
    }
    public void flywheelOn() {targetVelocity = onVelocity; flywheel.setPower(0);}
    public void flywheelOff() {targetVelocity = 0;}
    public static double getTargetVelocity () {return targetVelocity;}
    public void testFlyWheel (double powerT) {
        flywheel.setPower(powerT);
    }
    public boolean checkVelocity() {
        return true;};
}