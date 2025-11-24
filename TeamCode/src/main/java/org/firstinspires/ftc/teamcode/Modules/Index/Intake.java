package org.firstinspires.ftc.teamcode.Modules.Index;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;

public class Intake {

    private final DcMotorEx intakeMotor;
    private final BetterMotor intake;
    private double power = 0;

    public Intake (HardwareMap hardwareMap)
    {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        intake = new BetterMotor(intakeMotor, BetterMotor.RunMode.RUN, false);
    }

    public void activateIntake() {
        intake.setPower(power);
    }

    public void stopIntake() {
        intake.setPower(0);
    }

    public void reverseIntake() {
        intake.setPower(-power);
    }

    public void setIntakePower(double value) {
        power = value;
    }

}
