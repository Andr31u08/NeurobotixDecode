package org.firstinspires.ftc.teamcode.Modules.Index;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

@Config
public class Feeder {

    FtcDashboard dashboard;
    private final Servo wheelServo;
    private final DcMotorEx feederMotor;
    //private final BetterMotor feeder;
    //private final BetterServo wheel;

    Node placeWheel;
    Node startFeeder;
    Node retractWheel;
    Node stopFeeder;
    Node runningNode;
    Node stoppedNode;
    public Node currentNode;
    //TODO: Ale lui Bogdan
    private Servo extractoServo;
    private DcMotorEx extractoMotor;
    //TODO: -------------------

    public static double retractPos = 0, loadPosition = 0.5, testPos = 0;
    //public static double MaxVelocoty=20 , Acceleration=32  , Deceleration=32;
    public double power = 0.5;

    public Feeder(HardwareMap hardwareMap)
    {
        wheelServo = hardwareMap.get(Servo.class, "wheel");
        feederMotor = hardwareMap.get(DcMotorEx.class, "feeder");
        extractoMotor = hardwareMap.get(DcMotorEx.class, "feeder");
        extractoServo = hardwareMap.get(Servo.class, "wheel");
        extractoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feederMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        dashboard = FtcDashboard.getInstance();

        //feeder = new BetterMotor(feederMotor, BetterMotor.RunMode.RUN, false);
        //wheel = new BetterServo("wheel", wheelServo, BetterServo.RunMode.PROFILE, retractPos, false);

        //wheel.setProfileCoefficients(MaxVelocoty, Acceleration, Deceleration);

        placeWheel = new Node("placeWheel");
        startFeeder = new Node("startFeeder");
        retractWheel = new Node("retractWheel");
        stopFeeder = new Node("stopFeeder");
        runningNode = new Node("runningNode");
        stoppedNode = new Node("stoppedNode");

        currentNode = runningNode;

        placeWheel.addConditions(
                () -> {
                    wheelServo.setPosition(loadPosition);
                }
                ,
                () -> {
                    if(Math.abs(loadPosition-wheelServo.getPosition()) < 0.02) return true;
                    return false;
                }
                ,
                new Node[]{startFeeder}
        );
        startFeeder.addConditions(
                () -> {
                    feederMotor.setPower(power);
                }
                ,
                () -> {
                    return true;
                }
                ,
                new Node[]{runningNode}
        );
        runningNode.addConditions(
                () -> {}
                ,
                () -> {return true;
                }
                ,
                new Node[]{runningNode}
        );
        stopFeeder.addConditions(
                () -> {
                    feederMotor.setPower(0);
                }
                ,
                () -> {
                    return true;
                }
                ,
                new Node[]{retractWheel}
        );
        retractWheel.addConditions(
                () -> {
                    wheelServo.setPosition(retractPos);
                }
                ,
                () -> {
                    if(Math.abs(wheelServo.getPosition()-retractPos) < 0.02) return true;
                    return false;
                }
                ,
                new Node[]{stoppedNode}
        );
        stoppedNode.addConditions(
                () -> {}
                ,
                () -> {
                    return true;
                }
                ,
                new Node[]{stoppedNode}
        );
    }
    /* TODO Ale lui Bogdan
    public void shift(){
        extractoMotor.setPower(1);
        extractoServo.setPosition(0.7);
    }
    public void retract(){
        extractoServo.setPosition(0);
        extractoMotor.setPower(0);
    }
    public void shoot(Gamepad gamepad){
        if(gamepad.triangle){
            shift();
            try{
                sleep(1000);
            } catch (InterruptedException e){
                throw new RuntimeException(e);
            }
            retract();


        }
    }TODO ----------------------------*/
    public void update() {
        currentNode.run();
        if (currentNode.transition()) {
            currentNode = currentNode.next[0];
        }
    }

    public void startFeeder() {
        currentNode = placeWheel;
    }

    public void stopFeeder() {
        currentNode = stopFeeder;
    }
    public boolean isRunning() {return (currentNode == runningNode);}
    public boolean isStopped() {return (currentNode == stoppedNode);}

    //TODO: Debugging methods

    public void setTestPosition() {wheelServo.setPosition(testPos);}

}
