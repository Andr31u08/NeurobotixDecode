package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Index.IndexWheel;
import org.firstinspires.ftc.teamcode.Modules.Index.Intake;
import org.firstinspires.ftc.teamcode.Modules.Robot.Robot;
import org.firstinspires.ftc.teamcode.Wrappers.GamepadWrapper;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp(name = "TestOp")
public class TestTeleOpDecode extends LinearOpMode {
    private final boolean isRedAlliance = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, isRedAlliance);
        IndexWheel index = new IndexWheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        GamepadWrapper gamepad = new GamepadWrapper(gamepad1);
        MecanumDriveTrain driveTrain = new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE, hardwareMap);


        waitForStart();
        while (opModeIsActive()) {
            gamepad.update(gamepad1);
            robot.robotUpdate();
            double X = gamepad1.left_stick_x * gamepad1.left_stick_x * Math.signum(gamepad1.left_stick_x);
            double Y = gamepad1.left_stick_y * gamepad1.left_stick_y * -Math.signum(gamepad1.left_stick_y);
            double rotation = gamepad1.right_trigger - gamepad1.left_trigger;

            double heading = -Odo.getHeading();

            double x = X * Math.cos(heading) - Y * Math.sin(heading);
            double y = X * Math.sin(heading) + Y * Math.cos(heading);

            driveTrain.setTargetVector(x, y, rotation);
            if (gamepad.isAPressed())
                robot.loadGreen();

            if (gamepad.isBPressed())
                robot.loadPurple();

            if (gamepad.isXPressed())
                robot.startFeeder();
            if (gamepad.isYPressed())
                robot.stopFeeder();

            if (gamepad.isDpadDownPressed())
                robot.setFeederTestPosition();

            telemetry.addData("Robot node state machine state: ", robot.currentNode.name);
            telemetry.addLine("Nope");
            telemetry.addData("Feeder node state machine state: ", robot.getFeederStateName());
            telemetry.addData("Totem detected: ", robot.totemDetected());
            telemetry.addData("FiducialID: ", robot.getFiducialId());
            telemetry.addData("Limelight x: ", robot.getXll());
            telemetry.addData("Turret angle: ", robot.getTurretAngle());
            telemetry.addData("Turret target angle: ", robot.getTurretTargetAngle());
            telemetry.addData("Index current target position: ", robot.getCurrentIndexTarget());
            telemetry.addData("Index current actual position: ", robot.getCurrentIndexActualPosition());
            telemetry.addData("Index is toggled: ", robot.getToggled());
            telemetry.addData("Sensor gets purple: ", robot.isPurple());
            telemetry.addData("Sensor gets green: ", robot.isGreen());
            telemetry.addData("Red amount: ", robot.RedAmount());
            telemetry.addData("Green amount: ", robot.GreenAmount());
            telemetry.addData("Blue amount: ", robot.BlueAmount());
            telemetry.addData("Are slots full: ", robot.areSlotsFull());
            //telemetry.addData("test red amount: ", robot.testRed());
            //telemetry.addData("Turret encoder position: ", robot.getEncoderPosition());
            telemetry.update();
        }

    }
}
