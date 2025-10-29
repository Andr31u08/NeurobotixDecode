package org.firstinspires.ftc.teamcode.Modules.turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

public class Turret {

    private final CRServo turret = Hardware.sch4;
    private final DcMotorEx encoder = Hardware.meh0;

    private final TurretController turretController =  new TurretController(hardwareMap);


}
