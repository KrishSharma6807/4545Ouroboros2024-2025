package org.firstinspires.ftc.teamcode.threadTeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ThreadHandler;
@Config
public abstract class ThreadTeleLib extends OpMode {
    FtcDashboard dashboard;

    public DcMotor br;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor fl;
    public DcMotor intake;
    public DcMotor outtakeSlidesRight;
    public DcMotor outtakeSlidesLeft;

    public Servo intakeTilt;
    public Servo claw;
    public CRServo intakeSlidesLeft;
    public CRServo intakeSlidesRight;
    public Servo armLeft;
    public Servo armRight;
    public Servo wristRight;
    public Servo wristLeft;

    public ThreadHandler th_intake;
    public ThreadHandler th_intakeTilt;
    public ThreadHandler th_outtake;
    //TelemetryPacket
    private static  double kP = 0.0000174;
    private static  double kI = 0;
    private static  double kD = 0.0000012;

    private static  double kPh = 0.00174;
    private static  double kIh = 0;
    private static  double kDh = 0.000012;

    private static  double kF = 0.01;

    private double lowPass = 0;
    private final double a = 0.1;

    private double targetPosition = 0;  // Target encoder position for PIDF
    private double integralSum = 0;
    private double previousError = 0;
    private ElapsedTime timer = new ElapsedTime();
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
        outtakeSlidesRight = hardwareMap.get(DcMotor.class, "outtakeSlidesRight");
        outtakeSlidesLeft = hardwareMap.get(DcMotor.class, "outtakeSlidesLeft");

        intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
        claw = hardwareMap.get(Servo.class, "claw");
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        wristRight = hardwareMap.get(Servo.class, "wristRight");
        intakeSlidesLeft = hardwareMap.get(CRServo.class, "intakeSlidesLeft");
        intakeSlidesRight = hardwareMap.get(CRServo.class, "intakeSlidesRight");

        th_intake = new ThreadHandler();
        th_intakeTilt = new ThreadHandler();
        th_outtake = new ThreadHandler();


        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);

        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeSlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowPass = outtakeSlidesLeft.getCurrentPosition();
    }
    // add threads here

    Thread outtake_up_high_bucket = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

                targetPosition = -2500;
                RunTOPos();
                armLeft.setPosition(.6);
                wristLeft.setPosition(1);

            }

    });

    Thread outtake_down = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

            targetPosition = -10;
            RunTOPos();
            claw.setPosition(1);
            armLeft.setPosition(.15);
            wristLeft.setPosition(0);


        }

    });

    Thread outtake_down_partial = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

            armLeft.setPosition(0);
            armRight.setPosition(0);
            outtakeSlidesLeft.setPower(-1);
            outtakeSlidesRight.setPower(-1);

        }

    });

    Thread intake_TiltUp = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 10) {

            }

            intakeTilt.setPosition(.65);

        }

    });

    Thread intake_TiltDown = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 10) {

            }

            intakeTilt.setPosition(.27);

        }

    });

    

    public void telem(){
        telemetry.addData("fr:", fr.getPower());
        telemetry.addData("br:", br.getPower());
        telemetry.addData("fl:", fl.getPower());
        telemetry.addData("bl:", bl.getPower());

        telemetry.addData("rStickx:", gamepad1.right_stick_x);
        telemetry.addData("lStickx:", gamepad1.left_stick_x);
        telemetry.addData("lSticky:", gamepad1.left_stick_x);

        telemetry.addData("intakeTilt:", intakeTilt.getPosition());

        telemetry.addData("outtakeLeft",outtakeSlidesLeft.getPower());
        telemetry.addData("outtakeRight",outtakeSlidesRight.getPower());

        telemetry.addData("outtakeLeft",outtakeSlidesLeft.getPower());
        telemetry.addData("outtakeRight",outtakeSlidesRight.getPower());

        telemetry.addData("targ", targetPosition);

        telemetry.addData("CurrPosFiltered", lowPass);
        telemetry.addData("CurrPos", outtakeSlidesLeft.getCurrentPosition());

        telemetry.addData("intakeSlidesLeft", intakeSlidesLeft.getPower());
        telemetry.addData("intakeSlidesRight", intakeSlidesRight.getPower());

        telemetry.addData("armLeftPos", armLeft.getPosition());
        telemetry.addData("armRightPos", armRight.getPosition());
        telemetry.addData("wristLeftPos", wristLeft.getPosition());
        telemetry.addData("2a", gamepad2.a);
        telemetry.update();
    }
    public void ArcadeDrive() {

        double left_stick_x = gamepad1.left_stick_x;
        double left_stick_y = gamepad1.left_stick_y;
        double right_stick_x = gamepad1.right_stick_x;

        if (Math.abs(left_stick_x) > 0.1 ||
                Math.abs(left_stick_y) >.1|| Math.abs(right_stick_x) > 0.1){
            fr.setPower((left_stick_y + left_stick_x) + right_stick_x);
            fl.setPower((left_stick_y - left_stick_x) - right_stick_x);
            br.setPower((left_stick_y - left_stick_x) + right_stick_x);
            bl.setPower((left_stick_y + left_stick_x) - right_stick_x);
        }
        else{
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
            bl.setPower(0);
        }
    }

    public void horizSlides() {

         if (gamepad2.left_trigger > .2) {
            intakeSlidesLeft.setPower(1);
            intakeSlidesRight.setPower(-1);
            //intakeTilt.setPosition(.65);
        } else if (gamepad2.right_trigger > .2) {
            intakeSlidesLeft.setPower(-1);
            intakeSlidesRight.setPower(1);
        }
        else {
            intakeSlidesLeft.setPower(0);
            intakeSlidesRight.setPower(0);
        }

    }

    public void intakeTilt() {
        if (gamepad2.x && intakeTilt.getPosition() != .65) {
           th_intake.queue(intake_TiltUp);
            //intakeTilt.setPosition(.65);
//
        } else if (gamepad2.x) {
            th_intake.queue(intake_TiltDown);
            //intakeTilt.setPosition(.27);
        }
    }

    public void verticalSlides(){
        if(gamepad2.right_stick_y > .2){
            outtakeSlidesLeft.setPower(1);
            outtakeSlidesRight.setPower(-1);
        }
        else if (gamepad2.right_stick_y < -.2){
            outtakeSlidesLeft.setPower(-1);
            outtakeSlidesRight.setPower(1);
        }
        else if (gamepad2.dpad_up){
            th_outtake.queue(outtake_up_high_bucket);
        }
        else if (gamepad2.dpad_down){
            th_outtake.queue(outtake_down);
        }
        else{
            targetPosition = -outtakeSlidesRight.getCurrentPosition();
            //holdPos();
            outtakeSlidesLeft.setPower(0);
            outtakeSlidesRight.setPower(0);
        }
    }



    public void claw (){
        boolean bumperPressed = false;
        if(gamepad2.left_bumper ){
            claw.setPosition(0);
        }
        else if (gamepad2.right_bumper){
            claw.setPosition(1);
        }
    }

    public void arm(){
        if(gamepad2.a && armLeft.getPosition() != 0.1 /*armRight.getPosition() != 0*/){
            armLeft.setPosition(.15); //is down
            //armRight.setPosition(0);

            wristLeft.setPosition(0);
            //wristRight.setPosition(0);
        }
        else if (gamepad2.a){
            armLeft.setPosition(.8); //is up
            //armRight.setPosition(1);

            wristLeft.setPosition(1);
        }
    }

    public void wrist(){
        if(gamepad2.dpad_down && wristLeft.getPosition() != 0){
            wristLeft.setPosition(0);
        }
        else if (gamepad2.dpad_down){
            wristLeft.setPosition(1);
        }
    }

    public void intake(){
        if(gamepad2.b){
            intake.setPower(-1);
//            intake.setPower(1);
        }
        else if (gamepad2.y){
            intake.setPower(1);
//            intake.setPower(-1);
        }
        else{
            intake.setPower(0);
        }
    }

    public void holdPos(){

        double elapsedTime = timer.seconds();
        timer.reset();

        if (elapsedTime <= 0) {
            outtakeSlidesLeft.setPower(0);
            outtakeSlidesRight.setPower(0);
            return;
        }

        double position = outtakeSlidesRight.getCurrentPosition();

        lowPass = Math.abs(a * position + (1 - a) * lowPass);

        double error = targetPosition - lowPass;
        integralSum += error * timer.seconds();
        integralSum = Math.max(-1.0, Math.min(1.0, integralSum));
        double derivative = (error - previousError) / timer.seconds();
        double feedforward = kF * targetPosition;

        // Compute motor power
        double power = (kPh * error) + (kIh * integralSum) + (kDh * derivative) + feedforward;
        power = Math.max(-1.0, Math.min(1.0, power));  // Clamp power to [-1, 1]

        outtakeSlidesLeft.setPower(power);
        outtakeSlidesRight.setPower(-power);

        previousError = error;
        timer.reset();

        telemetry.addData("error", error);
        telemetry.addData("outtakeLeft",outtakeSlidesLeft.getPower());
        telemetry.addData("outtakeRight",outtakeSlidesRight.getPower());

        telemetry.addData("targ", targetPosition);

        telemetry.addData("CurrPosFiltered", lowPass);
        telemetry.addData("CurrPos", outtakeSlidesLeft.getCurrentPosition());
        telemetry.update();

    }

    public void RunTOPos(){
        double error = targetPosition - outtakeSlidesLeft.getCurrentPosition();
        while(Math.abs(error) > 5) {
            telemetry.addData("error", error);
            telemetry.addData("outtakeLeft",outtakeSlidesLeft.getPower());
            telemetry.addData("outtakeRight",outtakeSlidesRight.getPower());

            telemetry.addData("targ", targetPosition);

            telemetry.addData("CurrPosFiltered", lowPass);
            telemetry.addData("CurrPos", outtakeSlidesLeft.getCurrentPosition());
            telemetry.update();
            double elapsedTime = timer.seconds();
            timer.reset();

            if (elapsedTime <= 0) {
                outtakeSlidesLeft.setPower(0);
                outtakeSlidesRight.setPower(0);
                return;
            }

            double position = outtakeSlidesLeft.getCurrentPosition();

            lowPass = a * position + (1 - a) * lowPass;

            error = targetPosition - lowPass;
            integralSum += error * timer.seconds();
            integralSum = Math.max(-1.0, Math.min(1.0, integralSum));
            double derivative = (error - previousError) / timer.seconds();

            // Compute motor power
            double power = (kP * error) + (kI * integralSum) + (kD * derivative);
            power = Math.max(-1.0, Math.min(1.0, power));  // Clamp power to [-1, 1]

            outtakeSlidesLeft.setPower(power);
            outtakeSlidesRight.setPower(-power);

            previousError = error;
            timer.reset();
        }
    }

    public void stop(){
        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);
    }


}

// fdsfdsfssdddddddsdff
