package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Config
public class Limelight {

    //public static double distanceLimeLight=74.5;
    //public static double r=27;
    //public static double k=0.9;
    //public static double distanceIntake=93;
    //public static double lateralDistance=-103;
    // public static double extendoPosition;

    public static Limelight3A limelight;
    public static double cameraAngle = 30;
    public static double cameraHeight = 285;
    public static double targetHeight = 30;

    public static LLResult result;

    public static double Distance = 0;
    public static double targetAngle;

    public static double X;
    public static double Y;
    public static int fiducialId;

    //TODO: Set alliance fiducial tower ids !!!
    public static int redAllianceId = 24;
    public static int blueAllianceId = 20;
    public static int patternPPGId = 23;
    public static int patternPGPId = 22;
    public static int patternGPPId = 21;

    public Limelight(HardwareMap hardwareMap , int pipeline){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public void update() {
        LLResult latest = limelight.getLatestResult();
        if (latest == null) return;
        List<LLResultTypes.FiducialResult> fiducials = latest.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return;
        LLResultTypes.FiducialResult fr = fiducials.get(0);
        X = fr.getTargetXDegrees();
        Y = fr.getTargetYDegrees();
        fiducialId = fr.getFiducialId();
    }

    public double getY() {

        return Y;
    }
    public int getFiducialId() {return fiducialId;}

    public double fiducialDistance() {
        //result=limelight.getLatestResult();
        // if(result==null)return;

        //Y= cameraHeight *Math.tan(Math.toRadians(result.getTy()+90-cameraAngle));

       // X=Math.sqrt( Y*Y + cameraHeight * cameraHeight)*Math.tan(Math.toRadians(result.getTx()));

        double angleToGoalDegrees = cameraAngle + Y;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        Distance = (targetHeight - cameraHeight) / Math.tan(angleToGoalRadians);


        //targetAngle=Math.atan((X+lateralDistance)/(Y+distanceLimeLight));

        //Distance = Math.sqrt( (X+lateralDistance)*(X+lateralDistance) +(Y+distanceLimeLight)*(Y+distanceLimeLight));
        //extendoPosition=(Distance-distanceIntake)/(2*Math.PI*r)*(8194/50);
        return Distance;
    }

    public static boolean isRedAlliance() {return (fiducialId == redAllianceId);}
    public static boolean isBlueAlliance() {return (fiducialId == blueAllianceId);}

    public int patternCheck() {
        if (getFiducialId() == patternGPPId ||
            getFiducialId() == patternPGPId ||
            getFiducialId() == patternPPGId)
            return 1;
        return 0;
    }
    public int ppgPattern() {return patternPPGId;};
    public int pgpPattern() {return patternPGPId;};
    public int gppPattern() {return patternGPPId;};

}

