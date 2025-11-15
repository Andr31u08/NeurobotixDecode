package org.firstinspires.ftc.teamcode.Modules.Index;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

public class Feeder {

    private final Servo wheelServo;
    private final DcMotorEx feederMotor;
    private final BetterMotor feeder;
    private final BetterServo wheel;

    Node placeWheel, startFeeder, retractWheel, stopFeeder, runningNode, stoppedNode, currentNode;

    private static double retractPos = 0, loadPosition = 0;
    private double power = 0;

    public Feeder(HardwareMap hardwareMap)
    {
        wheelServo = hardwareMap.get(Servo.class, "wheel");
        feederMotor = hardwareMap.get(DcMotorEx.class, "feeder");
        feederMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feederMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        feeder = new BetterMotor(feederMotor, BetterMotor.RunMode.RUN, false);
        wheel = new BetterServo("wheel", wheelServo, BetterServo.RunMode.PROFILE, retractPos, false);

        placeWheel = new Node("placeWheel");
        startFeeder = new Node("startFeeder");
        retractWheel = new Node("retractWheel");
        stopFeeder = new Node("stopFeeder");
        runningNode = new Node("runningNode");
        stoppedNode = new Node("stoppedNode");

        currentNode = runningNode;

        placeWheel.addConditions(
                () -> {
                    wheel.setPosition(loadPosition);
                }
                ,
                () -> {
                    if(wheel.inPosition()) return true;
                    return false;
                }
                ,
                new Node[]{startFeeder}
        );
        startFeeder.addConditions(
                () -> {
                    feeder.setPower(power);
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
                    feeder.setPower(0);
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
                    wheel.setPosition(retractPos);
                }
                ,
                () -> {
                    if (wheel.inPosition()) return true;
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

}
