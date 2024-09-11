package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class TeleLib extends OpMode {
    public DcMotor br;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor fl;
    public CRServo horizSlideLeft;
    public CRServo horizSlideRight;
    public CRServo intakeTilt;
    public DcMotor intake;

    public Servo wrist;
    public Servo claw;

    public void init() {

        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        horizSlideLeft = hardwareMap.get(CRServo.class, "leftSlide");
        horizSlideRight = hardwareMap.get(CRServo.class, "rightSlide");
        intakeTilt = hardwareMap.get(CRServo.class, "intakeTilt");
        intake = hardwareMap.get(DcMotor.class, "intake");

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        horizSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        horizSlideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeTilt.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);


        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void ArcadeDrive() {

        if (gamepad1.left_stick_y > 0.1 && gamepad1.left_stick_x > 0.1) {
            fl.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
            br.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
        } else if (gamepad1.left_stick_y < 0.1 && gamepad1.left_stick_x > 0.1) {
            fl.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
            br.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
        } else if (gamepad1.left_stick_y > 0.1 && gamepad1.left_stick_x < 0.1) {
            fr.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
            bl.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
        } else if (gamepad1.left_stick_y < 0.1 && gamepad1.left_stick_x < 0.1) {
            fl.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
            br.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
        }
        if (gamepad1.right_trigger > 0.1 ) {
            fl.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
    }

    public void horizSlides(){
        if (Math.abs(gamepad2.left_stick_y) > 0.2){
            horizSlideLeft.setPower(gamepad2.left_stick_y);
            horizSlideRight.setPower(gamepad2.left_stick_y);
        }
    }

    public void intake(){
        if (gamepad2.right_trigger > 0.1){
            intakeTilt.setPower(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger > 0.1){
            intakeTilt.setPower(-gamepad2.right_trigger);
        } else {
            intakeTilt.setPower(0);
        }

        if (gamepad2.right_bumper){
            intake.setPower(1);
        } else if (gamepad2.left_bumper) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
    }

    public void Claw(){
        if (gamepad2.x) {
            wrist.setPosition(-1);
        }
        if (gamepad2.y) {
            wrist.setPosition(1);
        }

        if (gamepad2.a) {
            claw.setPosition(-1);
        }
        if (gamepad2.b) {
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
