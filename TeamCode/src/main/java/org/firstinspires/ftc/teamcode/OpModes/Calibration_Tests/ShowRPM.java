package org.firstinspires.ftc.teamcode.OpModes.Calibration_Tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShowRPM {
    private DcMotorEx shooter;
    public ShowRPM (HardwareMap hardwareMap){
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void RPM(){
        while (true){
            if(gamepad1.right_bumper){
                shooter.setPower(0.75);
            }
            telemetry.addData("Current RPM : ", shooter.getVelocity()/shooter.getMotorType().getTicksPerRev() * 60);
            telemetry.update();
        }
    }
}
