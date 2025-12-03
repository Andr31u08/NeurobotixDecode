package org.firstinspires.ftc.teamcode.Modules.Robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Index.Feeder;
import org.firstinspires.ftc.teamcode.Modules.Index.IndexWheel;
import org.firstinspires.ftc.teamcode.Modules.Index.Intake;
import org.firstinspires.ftc.teamcode.Modules.Turret.Flywheel;
import org.firstinspires.ftc.teamcode.Modules.Turret.Hood;
import org.firstinspires.ftc.teamcode.Modules.Turret.TurretController;
import org.firstinspires.ftc.teamcode.Robot.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Node;

public class Robot {

    private final TurretController turretController;
    public final Flywheel flywheel;
    private final Hood hood;
    private final Limelight limelight;
    public final Feeder feeder;
    private final IndexWheel index;
    private final Intake intake;
    private int patternId = 0;
    private boolean isRedAliance;
    private boolean shooting = false;
    private int ppgOrder[] = {2, 2, 1}, pgpOrder[] = {2, 1, 2}, gppOrder[] = {1, 2, 2};
    private int orderIndex = 0;

    //private final ColorSensor sensor;

    private double currentAngle;

    Node detectPattern, searchTowerTag, towerTagDetected, shoot;
    public Node currentNode;

    // State state;

    public Robot (HardwareMap hardwareMap, boolean isRedAliance)
    {
        turretController = new TurretController(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        hood = new Hood(hardwareMap);
        limelight = new Limelight(hardwareMap, 7);
        index = new IndexWheel(hardwareMap);
        feeder = new Feeder(hardwareMap);
        intake = new Intake(hardwareMap);

        //sensor = hardwareMap.get(ColorSensor.class, "sensor");

        detectPattern = new Node("detectPattern");
        searchTowerTag = new Node("searchTowerTag");
        towerTagDetected = new Node("towerTagDetected");
        shoot = new Node("shoot");

        currentNode = detectPattern;

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
                        currentAngle = getTurretAngle();
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
                    turretController.setTargetAngle(currentAngle+limelight.X);
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

    /*public enum State {
        NOT_AIMING ,
        AIMING ,
        SHOOTING
    }*/

    public void update() {
        currentNode.run();
        if (currentNode.transition()) {
            currentNode = currentNode.next[0];
        }
    }

    public void startTurret() {currentNode = detectPattern;}
    public void funcLoadArtifact() {
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

            orderIndex++;
            if (orderIndex > 2) orderIndex = 0;
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

    public void loadGreen() {
        index.loadGreen();
    }

    public void loadPurple() {
        index.loadPurple();
    }

    public void fakeLoadGreen() {
        index.artifactGreenIn();
    }
    public void fakeLoadPurple() {
        index.artifactPurpleIn();
    }
    public void checkRunIntake() {if (index.emptySlots()) intake.activateIntake(); else if (index.fullSlots()) intake.stopIntake();}
    public void loadArtifact() {if (!index.emptySlots()) funcLoadArtifact();}

    public void noSensorLoadPurple() {index.loadPurple();}
    public void noSensorLoadGreen() {index.loadGreen();}

    //TODO: Debug methods
    public void activeFlywheel() {flywheel.flywheelOn();}
    public void startFeeder() {feeder.startFeeder();}
    public void stopFeeder() {feeder.stopFeeder();}
    public String getFeederStateName() {return feeder.currentNode.name;}
    public void setFeederTestPosition() {feeder.setTestPosition();}
    public boolean totemDetected() {return (limelight.patternCheck() == 1);}
    public int getFiducialId() {return limelight.getFiducialId();}
    public double getXll() {return limelight.X;}
    public double getTurretAngle() {return turretController.getCurrentAngle();}
    public double getTurretTargetAngle() {return turretController.getEffectiveTargetAngle();}

    public int getCurrentIndexTarget() {return index.getCurrentIndexTarget();}
    public int getCurrentIndexActualPosition() {return index.getCurrentIndexActualPosition();}
    public boolean getToggled() {return index.getToggled();}
    public boolean isPurple() {return index.isPurple();}
    public boolean isGreen() {return index.isGreen();}
    public double RedAmount() {return index.RedAmount();}
    public double GreenAmount() {return index.GreenAmount();}
    public double BlueAmount() {return index.BlueAmount();}
    //public double getEncoderPosition() {return turretController.getEcoderPosition();}
    public boolean areSlotsFull() {return index.areSlotsFull();}

    //public int testRed() {return sensor.green();}

    public void robotUpdate() {
        feeder.update();
        update();
        checkRunIntake();
        loadArtifact();
        index.update();
        turretController.update();
    }
}
