package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Math.PIDController;

public class BetterMotor {

    public enum RunMode{
        RUN ,
        PID ,
        PIDVELOCITY
    }
    RunMode runMode;

    PIDController pidController;
    public DcMotorEx motor;
    boolean power;
    DcMotorEx encoder;

    public double targetPosition;
    public double targetVelocity;
    //TODO: Set this value !!!
    public double maxVelocityAtFullPower = 0;
    public int encoderReversed=-1;

    public BetterMotor(DcMotorEx motor , RunMode runMode , boolean reversed)
    {
        this.motor=motor;
        this.runMode=runMode;

        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(reversed)this.motor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public BetterMotor(DcMotorEx motor , RunMode runMode , boolean reversed , DcMotorEx encoder)
    {
        this.motor=motor;
        this.runMode=runMode;

        this.encoder=encoder;
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(reversed)this.motor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public BetterMotor(DcMotorEx motor , RunMode runMode , boolean reversed , DcMotorEx encoder , boolean encoderReversed)
    {
        this.motor=motor;
        this.runMode=runMode;

        this.encoder=encoder;
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(encoderReversed)this.encoderReversed=-1;
        else this.encoderReversed=1;

        if(reversed)this.motor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public BetterMotor(DcMotorEx motor, RunMode runMode, boolean reversed, DcMotorEx encoder, double targetVelocity)
    {
        this.motor=motor;
        this.runMode=runMode;

        this.targetVelocity = targetVelocity;
        this.encoder=encoder;
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(reversed)this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public BetterMotor(DcMotorEx motor, RunMode runMode, boolean reversed, DcMotorEx encoder, double targetVelocity, boolean encoderReversed)
    {
        this.motor=motor;
        this.runMode=runMode;

        this.targetVelocity = targetVelocity;
        this.encoder=encoder;
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(encoderReversed)this.encoderReversed=-1;
        else this.encoderReversed=1;

        if(reversed)this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPidCoefficients(double kp , double ki , double kd)
    {
        pidController=new PIDController(kp , ki , kd);
    }
    public void setPidCoefficients(double kp , double ki , double kd , double ff1 , double ff2)
    {
        pidController=new PIDController(kp , ki , kd , ff1 , ff2);
    }

    public void setPosition(double position)
    {
        power=false;
        targetPosition=position;
    }

    public void setPower(double power)
    {
        this.power=true;
        motor.setPower(power);
    }
    public void setRunMode(RunMode runMode)
    {
        this.runMode=runMode;
    }

    public double getPosition()
    {
        return encoder.getCurrentPosition() * encoderReversed;
    }

    public double getVelocity()
    {
        return encoder.getVelocity() * encoderReversed;
    }

    public void resetPosition()
    {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void update()
    {
        if(runMode==RunMode.RUN || power)return;

        double power = 0;

        if(runMode == RunMode.PID)
            power=pidController.calculate(targetPosition , encoder.getCurrentPosition()*encoderReversed);

        if(runMode == RunMode.PIDVELOCITY)
        {
            double feedforward = targetVelocity / maxVelocityAtFullPower; // base motor power
            double error = targetVelocity - getVelocity();
            double dt = pidController.timer.seconds();
            pidController.timer.reset();

            pidController.integralSum += error * dt;
            double pidCorrection = pidController.ki * pidController.integralSum + pidController.kp * error;

            power = feedforward + pidCorrection;

            double min = 0;
            double max = 0;

            if ( 1.0 < power)
                min = 1.0;
            else min = power;

            if (-1.0 > min)
                max = -1.0;
            else max = min;

            power = max;
        }

        motor.setPower(power);
    }
}