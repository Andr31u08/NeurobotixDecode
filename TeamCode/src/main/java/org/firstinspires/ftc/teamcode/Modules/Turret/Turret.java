package org.firstinspires.ftc.teamcode.Modules.Turret;

import com.qualcomm.robotcore.hardware.HardwareMap;

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
