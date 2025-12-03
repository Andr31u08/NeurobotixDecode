package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Robot.Robot;

@TeleOp(name="FlyWheel Test")
public class FlywheelTest extends LinearOpMode {
    public Robot robot;
    public double power;
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        waitForStart();
        while(opModeIsActive())
        {
            robot.flywheel.testFlyWheel(power);
            if (gamepad1.dpad_up && power < 1)
            {
                power = power + 0.05;
                sleep(100);
            }
            if (gamepad1.dpad_down && power > 0)
            {
                power = power - 0.05;
                sleep(100);
            }
            telemetry.addData("power: ", power);
            telemetry.update();
        }
    }
}
