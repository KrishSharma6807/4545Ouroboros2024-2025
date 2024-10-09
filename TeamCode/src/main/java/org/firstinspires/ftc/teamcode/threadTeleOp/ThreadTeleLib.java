package org.firstinspires.ftc.teamcode.threadTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ThreadHandler;

public abstract class ThreadTeleLib extends OpMode {
    public DcMotor br;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor fl;
    public DcMotor intake;
    public Servo intakeTilt;
    public ThreadHandler th_intake;
    public ThreadHandler th_intakeTilt;
    public Servo claw;
    public Servo arm;
    public ThreadHandler th_outtake;
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
        intake = hardwareMap.get(DcMotor.class, "intake");
        Servo intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
        th_intake = new ThreadHandler();
        th_intakeTilt = new ThreadHandler();
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo arm = hardwareMap.get(Servo.class, "arm");
        th_outtake = new ThreadHandler();
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

    Thread intake_in = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

                intake.setPower(1);

            }

    });

    Thread intake_out = new Thread(new Runnable() {
        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

                intake.setPower(-1);

            }
    });

    Thread intake_TiltUp = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }


            intakeTilt.setPosition(1);

        }

    });

    Thread intake_TiltDown = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

            intakeTilt.setPosition(0);

            }

    });

    Thread outtake_front = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

                claw.setPosition(1);
                arm.setPosition(1);

            }

    });

    Thread outtake_back = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

            claw.setPosition(0);
            arm.setPosition(0);

        }

    });
}

// fdsfdsfssdddddddsdff
