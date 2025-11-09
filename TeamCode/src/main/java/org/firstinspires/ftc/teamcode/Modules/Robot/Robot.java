package org.firstinspires.ftc.teamcode.Modules.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.Index.Feeder;
import org.firstinspires.ftc.teamcode.Modules.Index.Index;
import org.firstinspires.ftc.teamcode.Modules.Index.Intake;
import org.firstinspires.ftc.teamcode.Modules.Turret.Flywheel;
import org.firstinspires.ftc.teamcode.Modules.Turret.Hood;
import org.firstinspires.ftc.teamcode.Modules.Turret.TurretController;
import org.firstinspires.ftc.teamcode.Robot.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Node;

public class Robot {

    private final TurretController turretController;
    private final Flywheel flywheel;
    private final Hood hood;
    private final Limelight limelight;
    private final Feeder feeder;
    private final Index index;
    private final Intake intake;
    private int patternId = 0;
    private boolean isRedAliance;
    private boolean shooting = false;
    private int ppgOrder[] = {2, 2, 1}, pgpOrder[] = {2, 1, 2}, gppOrder[] = {1, 2, 2};
    private int orderIndex = 0;

    Node detectPattern, searchTowerTag, towerTagDetected, shoot;

    // State state;

    public Robot (HardwareMap hardwareMap, boolean isRedAliance)
    {
        turretController = new TurretController(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);
        limelight = new Limelight(hardwareMap, 0);
        index = new Index(hardwareMap);
        feeder = new Feeder(hardwareMap);
        intake = new Intake(hardwareMap);

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
                    if ((limelight.isBlueAlliance() && !isRedAliance) ||
                            (limelight.isRedAlliance() && isRedAliance)) {
                        flywheel.flywheelOn();
                        return true;
                    }
                    return false;
                }
                ,
                new Node[]{towerTagDetected}
        );
        towerTagDetected.addConditions(
                () -> {
                    limelight.update();
                    turretController.setTargetAngle(limelight.X);
                    hood.updatePosition();
                    flywheel.updateFlyheel();
                }
                ,
                () -> {
                    if (((limelight.isBlueAlliance() && !isRedAliance) ||
                            (limelight.isRedAlliance() && isRedAliance)) || shooting)
                        return false;
                    else
                        flywheel.flywheelOff();
                    return true;
                }
                ,
                new Node[]{searchTowerTag}
        );
    }

    Node currentNode;

    /*public enum State {
        NOT_AIMING ,
        AIMING ,
        SHOOTING
    }*/

    public void startTurret() {currentNode = detectPattern;}

    public void loadArtifact() {
            if (patternId == limelight.gppPattern())
                if ((gppOrder[orderIndex] == index.green && index.fastestShiftGreen() > 0) && feeder.isStopped())
                    index.loadGreen();
                else if ((gppOrder[orderIndex] == index.purple && index.fastestShiftPurple() > 0) && feeder.isStopped())
                    index.loadPurple();
            if (patternId == limelight.pgpPattern())
                if ((pgpOrder[orderIndex] == index.green && index.fastestShiftGreen() > 0) && feeder.isStopped())
                    index.loadGreen();
                else if ((pgpOrder[orderIndex] == index.purple && index.fastestShiftPurple() > 0) && feeder.isStopped())
                    index.loadPurple();
            if (patternId == limelight.ppgPattern())
                if ((ppgOrder[orderIndex] == index.green && index.fastestShiftGreen() > 0) && feeder.isStopped())
                    index.loadGreen();
                else if ((ppgOrder[orderIndex] == index.purple && index.fastestShiftPurple() > 0) && feeder.isStopped())
                    index.loadPurple();
    }

    public void shoot() {
        if ((currentNode == towerTagDetected && flywheel.checkVelocity()) && (index.fastestShiftPurple() == 0 || index.fastestShiftGreen() == 0)) {
            shooting = true;
            feeder.startFeeder();
        }
    }

    public void stopShoot() {
        if (shooting) {
            feeder.stopFeeder();
            shooting = false;
        }
    }


}
