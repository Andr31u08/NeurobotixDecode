package org.firstinspires.ftc.teamcode.Modules.Turret;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Limelight;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

public class Hood {

    private final BetterServo hood;
    private final Servo hoodServo;
    private final Limelight limelight;

    // TODO:
    private static double startPos = 0;

    //TODO: derive the ecuation for the hood movement !!!

    public Hood (HardwareMap hardwareMap)
    {
        hoodServo = hardwareMap.get(Servo.class, "HoodServo");
        hood = new BetterServo("HoodServo", hoodServo, BetterServo.RunMode.PROFILE, startPos, false);
        limelight = new Limelight(hardwareMap, 0);
    }

    public void updatePosition()
    {
        // TODO: Scoti ecuatia aia nebuna aici !!!
        double ecuation = 0 + 1 + 0 + 1 - 67 + 67 - 67 + 67 * limelight.fiducialDistance();
    }
}
