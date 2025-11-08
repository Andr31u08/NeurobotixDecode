package org.firstinspires.ftc.teamcode.Modules.Turret;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Node;

public class Turret {

    private final TurretController turretController;
    private final Flywheel flywheel;
    private final Hood hood;
    private final Limelight limelight;
    private int patternId = 0;
    private boolean isRedAliance;

    Node detectPattern, searchTowerTag, towerTagDetected, shoot;

   // State state;

    public Turret (HardwareMap hardwareMap, boolean isRedAliance)
    {
        turretController = new TurretController(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);
        limelight = new Limelight(hardwareMap, 0);

        detectPattern = new Node("detectPattern");
        searchTowerTag = new Node("searchTowerTag");
        towerTagDetected = new Node("towerTagDetected");
        shoot = new Node("shoot");

        this.isRedAliance = isRedAliance;

        detectPattern.addConditions(
                () -> {
                    limelight.update();
                }
                ,
                () -> {
                    if (limelight.patternCheck() == 1) {
                        patternId = limelight.getFiducialId();
                        return true;
                    }
                    return false;
                }
                ,
                new Node[]{searchTowerTag}
        );
        searchTowerTag.addConditions(
                () -> {
                    limelight.update();
                }
                ,
                () -> {
                    if (limelight.isBlueAlliance() && !isRedAliance ||
                        limelight.isRedAlliance() && isRedAliance) return true;
                    return false;
                }
                ,
                new Node[]{towerTagDetected}
        );
        towerTagDetected.addConditions(
                () -> {

                }
                ,
                () -> {
                    return true;
                }
                ,
                new Node[]{towerTagDetected}
        );
    }

    /*public enum State {
        NOT_AIMING ,
        AIMING ,
        SHOOTING
    }*/

    public void turretAim() {
       // state = State.AIMING;

    }
}
