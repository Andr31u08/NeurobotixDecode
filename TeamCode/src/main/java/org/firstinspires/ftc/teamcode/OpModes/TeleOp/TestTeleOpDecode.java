package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Index.Intake;
import org.firstinspires.ftc.teamcode.Modules.Robot.Robot;
import org.firstinspires.ftc.teamcode.Wrappers.GamepadWrapper;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp(name = "TestOp")
public class TestTeleOpDecode extends LinearOpMode {
    private final boolean isRedAlliance = true;

    @Override
    public void runOpMode() throws InterruptedException {
        // Robot robot = new Robot(hardwareMap, isRedAlliance);
        Intake intake = new Intake(hardwareMap);
        GamepadWrapper gamepad = new GamepadWrapper(gamepad1);
        MecanumDriveTrain driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE, hardwareMap);

        waitForStart();
        while(opModeIsActive()) {
            //robot.robotUpdate();
            double X=gamepad1.left_stick_x * gamepad1.left_stick_x * Math.signum(gamepad1.left_stick_x);
            double Y=gamepad1.left_stick_y * gamepad1.left_stick_y * -Math.signum(gamepad1.left_stick_y);
            double rotation=gamepad1.right_trigger-gamepad1.left_trigger;

            double heading =-Odo.getHeading();

            double x=X*Math.cos(heading)-Y*Math.sin(heading);
            double y=X* Math.sin(heading)+Y*Math.cos(heading);

            driveTrain.setTargetVector( x , y , rotation );
            if (gamepad.isAPressed())
                intake.activateIntake();
            if (gamepad.isBPressed())
                intake.stopIntake();
            }
        }

    }
