package org.firstinspires.ftc.teamcode.Modules.Index;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Math.PID;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Sensor;

@Config
public class IndexWheel {

    private final DcMotorEx index;
    private final Sensor sensor;
    private final PID pid;
    private final FtcDashboard dashboard;
    public static double kp = 0.03, ki = 0.004, kd = 0.0013, f = 0.01, kp2 = 0.009, ki2, kd2 = 1, f2;

    //private int loadingPosOffset = 500; // Set offset between regular index to intake position to turret loading position
    //private int posOffset = 100; // Set offset between actual positions
    private int currPosition;

    public static int nlPos1 = 0, nlPos2 = 60, nlPos3 = 120, lPos1 = 20, lPos2 = 80, lPos3 = 140;

    private final int[] slots = {0, 0, 0};

    private final int[] notLoadPositions = {nlPos1, nlPos2, nlPos3};
    private final int[] loadedPositions = {lPos1, lPos2, lPos3};
    private final int[] currPositions = {nlPos1, nlPos2, nlPos3};

    private int slotIndexer = 0;
    private int positionIndexer = 0;

    public final int green = 1;
    public final int purple = 2;
    public final int empty = 0;

    private boolean turretLoaded = false;

    public IndexWheel(HardwareMap hardwareMap)
    {
        dashboard = FtcDashboard.getInstance();
        index = hardwareMap.get(DcMotorEx.class, "index");
        index.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //index.setPidCoefficients(0.5, 0 , 0);
        sensor = new Sensor(hardwareMap);
        pid = new PID(kp, kd, ki, f);
        pid.setOutputRange(-1, 1);
        pid.setIntegralLimit(0.25);
    }

    public void cycleIndexer() {if (slotIndexer > 2) slotIndexer = 0;}
    public void cyclePositions() {if (positionIndexer > 2) positionIndexer = 0;}

    public void initIndex() {index.setTargetPosition(currPositions[positionIndexer]);}

    public void artifactGreenIn() {
        slots[slotIndexer] = green;
        slotIndexer++;
        cycleIndexer();
        //index.setPosition(currPosition + posOffset);
        currPosition = currPositions[++positionIndexer];
        //index.update();
        //index.setPosition(currPosition);
        index.setTargetPosition(currPosition);
    }

    public void artifactPurpleIn() {
        slots[slotIndexer] = purple;
        slotIndexer++;
        cycleIndexer();
        //index.setPosition(currPosition + posOffset);
        //currPosition += posOffset;
        currPosition = currPositions[++positionIndexer];
        //index.update();
        //index.setPosition(currPosition);
        index.setTargetPosition(currPosition);
    }

    public int fastestShiftGreen() {
        int imIndexer = slotIndexer;
        boolean hasGreen = false;
        for (int i = 0; i < 3; i++)
            if (slots[i] == green) {hasGreen = true;break;}
        if (hasGreen)
            while (slots[imIndexer] != green) {imIndexer++;if (imIndexer > 2) imIndexer = 0;}
        else return 3;
        int difSlots = slotIndexer - imIndexer;
        if (difSlots == 2) return -1;
        return difSlots;
    }

    public int fastestShiftPurple() {
        int imIndexer = slotIndexer;
        boolean hasPurple = false;
        for (int i = 0; i < 3; i++)
            if (slots[i] == purple) {hasPurple = true;break;}
        if (hasPurple)
            while (slots[imIndexer] != purple) {imIndexer++;if (imIndexer > 2) imIndexer = 0;}
        else return 3;
        int difSlots = slotIndexer - imIndexer;
        if (difSlots == 2) return -1;
        return difSlots;
    }

    public void loadTurretToggle() {
        int i;
        if (turretLoaded) {
            turretLoaded = false;
            //index.setPosition(currPosition + loadingPosOffset);
            for (i = 0; i < 3; i++)
                currPositions[i] = notLoadPositions[i];
        }
        else {
            turretLoaded = true;
            //index.setPosition(currPosition - loadingPosOffset);
            //currPosition -= loadingPosOffset;
            for (i = 0; i < 3; i++)
                currPositions[i] = loadedPositions[i];
        }
        //index.update();
        //index.setPosition(currPosition);
        index.setTargetPosition(currPositions[positionIndexer]);
    }

    public void loadPurple() {
        int sSlots = fastestShiftPurple();
        if (sSlots == 3) return;

        positionIndexer += sSlots;
        currPosition = currPositions[positionIndexer];

        //int posD = sSlots * posOffset;
        //index.setPosition(currPosition + posD);
        //currPosition += posD;

        slotIndexer = (slotIndexer - sSlots) % 3;
        if (slotIndexer < 0) slotIndexer += 3;
        //index.update();
        //index.setPosition(currPosition);
        index.setTargetPosition(currPosition);
    }

    public void loadGreen() {
        int sSlots = fastestShiftGreen();
        if (sSlots == 3) return;

        positionIndexer += sSlots;
        currPosition = currPositions[positionIndexer];

        //int posD = sSlots * posOffset;
        //index.setPosition(currPosition + posD);
        //currPosition += posD;

        slotIndexer = (slotIndexer - sSlots) % 3;
        if (slotIndexer < 0) slotIndexer += 3;
        //index.update();
        //index.setPosition(currPosition);
        index.setTargetPosition(currPosition);
    }

    public void checkLoading() {
        if ((!loadingMotion()) && sensor.isGreen())
            loadGreen();
        if ((!loadingMotion()) && sensor.isPurple())
            loadPurple();
    }

    public boolean loadingMotion() {
        if ((sensor.isGreen() || sensor.isPurple()) &&
                Math.abs(index.getCurrentPosition()-currPosition) > 70)
            return true;
        return false;
    }

    public boolean emptySlots() {
        for (int i = 0; i < 3; i++)
            if (slots[i] != empty) return false;
        return true;
    }

    public boolean fullSlots() {
        for (int i = 0; i < 3; i++)
            if (slots[i] == empty) return false;
        return true;
    }

    public void checkToggleable() {
        if (fullSlots()) loadTurretToggle();
        if (emptySlots() && (!turretLoaded)) loadTurretToggle();
    }

    public void launchArtifact() {
        currPosition = currPositions[++positionIndexer];
        index.setTargetPosition(currPosition);
    }

    public void update() {
        checkLoading();
        checkToggleable();
        int currpos = index.getCurrentPosition();
        double value = pid.update(currPosition, currpos, kp, kd, ki, f);
        index.setPower(value);
    }



    //TODO: -------------------WARNING------------ THE SETTING TO POSITION WITH GET POSITION +- SOMETHING COULD LEAD TO PROBLEMS??
    //TODO: -------------------WARNING------------ ADD TO FASTEST ROUTE CHECK FOR SLOTS WITH 0 AND ALWAYS HAVE THEM SWITCHED TO THE EMPTY SLOT

    public int getCurrentIndexTarget() {return currPosition;}
    public int getCurrentIndexActualPosition() {return index.getCurrentPosition();}
    public boolean getToggled() {return turretLoaded;}
    public boolean isPurple() {return sensor.isPurple();}
    public double RedAmount() {return sensor.RedAmount();}
    public double GreenAmount() {return sensor.GreenAmount();}
    public double BlueAmount() {return sensor.BlueAmount();}
}