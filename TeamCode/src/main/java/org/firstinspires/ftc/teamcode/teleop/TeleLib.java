package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class TeleLib extends OpMode {
    public DcMotor br;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor fl;

    public Servo wrist;

    public Servo claw;

    public void init() {

        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        wrist = hardwareMap.get(Servo.class, "wrist");

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);


        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
    }
     public void kill() {
         bl.setPower(0);
         br.setPower(0);
         fr.setPower(0);
         fl.setPower(0);
    }

}
