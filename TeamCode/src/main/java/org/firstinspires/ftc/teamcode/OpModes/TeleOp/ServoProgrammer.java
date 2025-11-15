package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoProgrammer")
public class ServoProgrammer extends LinearOpMode {
    private Servo servo;
    private double position = 0, lastPos = position;
    private boolean aPressed = false, bPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(position);
        waitForStart();

        while(opModeIsActive()) {
            // Increase position
            if (gamepad1.a && !aPressed) {
                position = Math.min(1.0, position + 0.1);
                aPressed = true;
            }
            if (!gamepad1.a) aPressed = false;

            // Decrease position
            if (gamepad1.b && !bPressed) {
                position = Math.max(0.0, position - 0.1);
                bPressed = true;
            }
            if (!gamepad1.b) bPressed = false;

            // Update servo if changed
            if (lastPos != position) {
                lastPos = position;
                servo.setPosition(position);
            }

            // Telemetry
            telemetry.addData("Servo Position", position);
            telemetry.update();

            sleep(50);
        }
    }
}
