package org.firstinspires.ftc.teamcode.Modules.turret;

import static org.firstinspires.ftc.teamcode.Robot.Hardware.unlock;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;

public class Flywheel {

    private final DcMotorEx wheelMotor;
    private final DcMotorEx encoder;
    private final BetterMotor flywheel;

    private static double power = 0.75;

    //TODO: set values to these !!!
    private static double targetVelocity = 0;

    public Flywheel (HardwareMap hardwareMap)
    {
        wheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        unlock(wheelMotor);
        encoder = hardwareMap.get(DcMotorEx.class, "encoder");

        flywheel = new BetterMotor(wheelMotor, BetterMotor.RunMode.PIDVELOCITY, false, encoder,targetVelocity, false);
        flywheel.maxVelocityAtFullPower = 0; // TODO: Set this value !!!
        flywheel.setPidCoefficients(0, 0.5, 0);
    }

    public void updateFlyheel()
    {
        flywheel.update();
    }
}