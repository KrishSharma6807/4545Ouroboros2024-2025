package org.firstinspires.ftc.teamcode.Teleop;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ThreadHandler;

import java.util.HashMap;

public abstract class ThreadTeleLib extends OpMode {
    public DcMotor br;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor fl;
    /*public CRServo horizSlideLeft;
    public CRServo horizSlideRight;
    public Servo intakeTilt;
    public DcMotor intake;

    public Servo wrist;
    public Servo claw;*/

    public void init() {

        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
       /* wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        horizSlideLeft = hardwareMap.get(CRServo.class, "leftSlide");
        horizSlideRight = hardwareMap.get(CRServo.class, "rightSlide");
        intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
        intake = hardwareMap.get(DcMotor.class, "intake");*/

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
//        horizSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        horizSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        intakeTilt.setDirection(Servo.Direction.REVERSE);
//        intake.setDirection(DcMotorSimple.Direction.FORWARD);


        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    // add threads here
}