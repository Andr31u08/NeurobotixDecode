package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Turret.Hood;
import org.firstinspires.ftc.teamcode.Robot.Limelight;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

@TeleOp(name= "Hood Test")

public class HoodTest extends LinearOpMode {
    Hood hood;
    Limelight limelight;
    BetterServo servo;
    double distance, angle, speed;
    public void runOpMode() {
        hood = new Hood(hardwareMap);
        limelight = new Limelight(hardwareMap, 7);
        servo = hood.getServo();
        waitForStart();
        while(opModeIsActive())
        {
            distance = hood.distanceToTarget(limelight, true);
            angle = hood.calculateAngle(distance);
            hood.setServoPos(angle, servo);
            telemetry.addData("distance: ", distance);
            telemetry.addData("angle: ", angle);
        }
    }
}