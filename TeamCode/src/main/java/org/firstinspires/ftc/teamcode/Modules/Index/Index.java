package org.firstinspires.ftc.teamcode.Modules.Index;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.Limelight;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;

public class Index {

    private final BetterMotor index;
    private final DcMotorEx indexMotor;
    private final ColorSensor sensor;
    private int loadingPosOffset = 0; //TODO: Set offset between regular index to intake position to turret loading position
    private int posOffset = 0; //TODO: Set offset between actual positions
    private int currPosition = 0;
    private int slots[] = {0, 0, 0};
    private int slotIndexer = 0;
    public final int green = 1;
    public final int purple = 2;
    private boolean turretLoaded = false;
    public Index(HardwareMap hardwareMap)
    {
        indexMotor = hardwareMap.get(DcMotorEx.class, "index");
        indexMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        index = new BetterMotor(indexMotor, BetterMotor.RunMode.PID, false, indexMotor, false);
        sensor = hardwareMap.get(ColorSensor.class, "sensor");
    }

    public void cycleIndexer() {if (slotIndexer > 2) slotIndexer = 0;}

    public void artifactGreenIn() {
        slots[slotIndexer] = green;
        slotIndexer++;
        cycleIndexer();
        index.setPosition(currPosition + posOffset);
        currPosition += posOffset;
    }

    public void artifactPurpleIn() {
        slots[slotIndexer] = purple;
        slotIndexer++;
        cycleIndexer();
        index.setPosition(currPosition + posOffset);
        currPosition += posOffset;
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
        if (turretLoaded) {
            turretLoaded = false;
            index.setPosition(currPosition + loadingPosOffset);
            currPosition += loadingPosOffset;
        }
        else {
            turretLoaded = true;
            index.setPosition(currPosition - loadingPosOffset);
            currPosition -= loadingPosOffset;
        }
    }

    public void loadPurple() {
        int sSlots = fastestShiftPurple();
        if (sSlots == 3) return;

        int posD = sSlots * posOffset;
        index.setPosition(currPosition + posD);
        currPosition += posD;

        slotIndexer = (slotIndexer - sSlots) % 3;
        if (slotIndexer < 0) slotIndexer += 3;
    }

    public void loadGreen() {
        int sSlots = fastestShiftGreen();
        if (sSlots == 3) return;

        int posD = sSlots * posOffset;
        index.setPosition(currPosition + posD);
        currPosition += posD;

        slotIndexer = (slotIndexer - sSlots) % 3;
        if (slotIndexer < 0) slotIndexer += 3;
    }

    //TODO: -------------------WARNING------------ THE SETTING TO POSITION WITH GET POSITION +- SOMETHING COULD LEAD TO PROBLEMS??
    //TODO: -------------------WARNING------------ ADD TO FASTEST ROUTE CHECK FOR SLOTS WITH 0 AND ALWAYS HAVE THEM SWITCHED TO THE EMPTY SLOT
}