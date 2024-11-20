package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class TeleLib extends OpMode {
    public DcMotor br;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor fl;
//    public CRServo horizSlideLeft;
//    public CRServo horizSlideRight;
//    public Servo intakeTilt;
//    public DcMotor intake;

    public Servo wrist;
    public Servo claw;

    public void init() {

        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
//        horizSlideLeft = hardwareMap.get(CRServo.class, "leftSlide");
//        horizSlideRight = hardwareMap.get(CRServo.class, "rightSlide");
//        intakeTilt = hardwareMap.get(Servo.class, "intakeTilt");
//        intake = hardwareMap.get(DcMotor.class, "intake");

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

    public void ArcadeDrive() {

        double left_stick_x = gamepad1.left_stick_x;
        double left_stick_y = gamepad1.left_stick_y;
        double right_stick_x = gamepad1.right_stick_x;

        if(Math.abs(left_stick_x) > 0.1 ||
                Math.abs(left_stick_y) >.1|| Math.abs(right_stick_x) > 0.1) {
            fr.setPower(((left_stick_y + left_stick_x) + right_stick_x));
            fl.setPower(((left_stick_y - left_stick_x) - right_stick_x));
            br.setPower(((left_stick_y - left_stick_x) + right_stick_x));
            bl.setPower(((left_stick_y + left_stick_x) - right_stick_x));
        } else if (Math.abs(left_stick_x) > 0.1 ||
                Math.abs(left_stick_y) >.1|| Math.abs(right_stick_x) > 0.1){
            fr.setPower(((left_stick_y + left_stick_x) + right_stick_x));
            fl.setPower(((left_stick_y - left_stick_x) - right_stick_x));
            br.setPower(((left_stick_y - left_stick_x) + right_stick_x));
            bl.setPower(((left_stick_y + left_stick_x) - right_stick_x));
        }
        else{
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
            bl.setPower(0);
        }telemetry.addData("fr:", fr.getPower());
        telemetry.addData("br:", br.getPower());
        telemetry.addData("fl:", fl.getPower());
        telemetry.addData("bl:", bl.getPower());
    }
//      public void horizSlides(){
//        if (Math.abs(gamepad2.left_stick_y) > 0.2){
//            horizSlideLeft.setPower(gamepad2.left_stick_y);
//            horizSlideRight.setPower(gamepad2.left_stick_y);
//        }
//    }
//
//    public void intake(){
//        if (gamepad2.right_trigger > 0.1){
//            intakeTilt.setPosition(gamepad2.right_trigger);
//        } else if (gamepad2.left_trigger > 0.1){
//            intakeTilt.setPosition(-gamepad2.right_trigger);
//        }
//
//        if (gamepad2.right_bumper){
//            intake.setPower(1);
//        } else if (gamepad2.left_bumper) {
//            intake.setPower(-1);
//        } else {
//            intake.setPower(0);
//        }
//    }

    public void Claw(){
        wrist.setPosition(-1);
        if (wrist.getPosition() == 1 && (gamepad2.x)) {
            wrist.setPosition(-1);
        }
        if (wrist.getPosition() == -1 && (gamepad2.x)) {
            wrist.setPosition(1);
        }

        if (gamepad2.a && claw.getPosition() == 1) {
            claw.setPosition(-1);
        }else if (gamepad2.a && claw.getPosition() == -1 || claw.getPosition() != -1 || claw.getPosition() != 1) {
            claw.setPosition(1);
        }
    }
     public void kill() {
         bl.setPower(0);
         br.setPower(0);
         fr.setPower(0);
         fl.setPower(0);
    }

}
