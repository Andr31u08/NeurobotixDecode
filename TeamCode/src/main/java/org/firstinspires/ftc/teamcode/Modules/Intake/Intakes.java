package org.firstinspires.ftc.teamcode.Modules.Intake;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class Intakes {
    private DcMotor motorIntake;
    private boolean turning = false;
    public Intakes (HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //TODO needs to check if reverse is needed
    }
    private void spinTheIntake(){
        if(gamepad2.triangle){
            turning = !turning;
        }
        while (turning)
            motorIntake.setPower(1);
        while (!turning)
            motorIntake.setPower(0);
    }
}
