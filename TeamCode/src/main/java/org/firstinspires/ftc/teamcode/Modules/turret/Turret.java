package org.firstinspires.ftc.teamcode.Modules.turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Limelight;

public class Turret {

    private final TurretController turretController;
    private final Flywheel flywheel;
    private final Hood hood;
    private final Limelight limelight;

    State state;

    public Turret (HardwareMap hardwareMap)
    {
        turretController = new TurretController(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);
        limelight = new Limelight(hardwareMap, 0);
    }

    public enum State {
        NOT_AIMING ,
        AIMING ,
        SHOOTING
    }

    public void turretAim() {
        state = State.AIMING;

    }
}
