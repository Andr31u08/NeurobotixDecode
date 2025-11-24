package org.firstinspires.ftc.teamcode.Wrappers;


import static androidx.core.math.MathUtils.clamp;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;

public class BetterServo {

    public enum RunMode{
        PROFILE,
        Time,
        GoToPosition;
    }

    private static final double POSITION_EPS = 0.002;   // tolerance
    private static final double MIN_POS = 0.0;
    private static final double MAX_POS = 1.0;

    public ElapsedTime timer=new ElapsedTime();
    public double time;

    public RunMode runMode;

    public BetterMotionProfile profile;

    public double position;

    public boolean INIT=false;
    public boolean reachedPosition=true;

    public Servo servo;
    public String name;

    public BetterServo(String name,   Servo servo , RunMode runMode , double initialPosition , boolean reversed)
    {
        this.name=name;
        this.servo=servo;
        this.runMode=runMode;
        if(reversed)this.servo.setDirection(Servo.Direction.REVERSE);
        else this.servo.setDirection(Servo.Direction.FORWARD);
        position=initialPosition;
        timer.startTime();
    }
    public BetterServo(String name,   Servo servo , RunMode runMode , double initialPosition , boolean reversed , double time)
    {
        this.name=name;
        this.servo=servo;
        this.runMode=runMode;
        this.servo.setPosition(initialPosition);
        if(reversed)this.servo.setDirection(Servo.Direction.REVERSE);
        else this.servo.setDirection(Servo.Direction.FORWARD);
        position=initialPosition;
        if(runMode==RunMode.Time)
        {
            timer.startTime();
            this.time=time;
        }
    }
    public void setProfileCoefficients(double maxVelocity , double acceleration , double deceleration)
    {
        if(runMode!=RunMode.PROFILE)return;

        if(!INIT)
        {profile=new BetterMotionProfile(maxVelocity , acceleration , deceleration);INIT=true;}
        else {profile.maxVelocity=maxVelocity;
        profile.deceleration=deceleration;
        profile.acceleration=acceleration;}

        profile.setMotion(position , position , 0);
    }

    public void setPosition(double position)
    {
        if(runMode==RunMode.Time)if(this.position==position)return;
        if(runMode==RunMode.PROFILE)if(position==profile.finalPosition)return;



        switch (runMode)
        {
            case PROFILE:
                reachedPosition=false;
                profile.setMotion(profile.getPosition() , position , profile.getVelocity());
                break;
            case GoToPosition:
                reachedPosition=true;
                this.position=position;
                servo.setPosition(clamp(position));
                break;
            case Time:
                double target = clamp(position);
                if(servo.getPosition()==position)return;
                reachedPosition=false;
                servo.setPosition(target);
                timer.reset();
                this.position=position;
                break;
        }
        update();
    }
    public void setMODE(RunMode mode)
    {
        runMode=mode;
    }



    public double getPosition()
    {
        return position;
    }

    public boolean inPosition()
    {
        if(runMode==RunMode.PROFILE)
            return reachedPosition;
        if(runMode==RunMode.Time)
            return timer.seconds()>time;
        return true;
    }

    public void update()    /* Add this function to the loop if you use motionprofile */
    {
        if(runMode==RunMode.PROFILE){
        profile.update();
        position=profile.getPosition();

        double out = clamp(position);
        //servo.setPosition(profile.getPosition());
            servo.setPosition(out);

        reachedPosition = profile.isAtTarget();}
    }

    private double clamp(double v) {
        return Math.max(MIN_POS, Math.min(MAX_POS, v));
    }

}