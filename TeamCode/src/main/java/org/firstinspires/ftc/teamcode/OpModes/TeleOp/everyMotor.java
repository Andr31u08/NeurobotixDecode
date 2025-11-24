package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.apache.commons.math3.optim.linear.LinearConstraint;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Index.IndexWheel;
import org.firstinspires.ftc.teamcode.Modules.Index.Intake;
import org.firstinspires.ftc.teamcode.Modules.Robot.Robot;
import org.firstinspires.ftc.teamcode.Wrappers.GamepadWrapper;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp(name = "motoareee")
public class everyMotor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.5);

            waitForStart();
            while(opModeIsActive()) {
            }
    }
}
