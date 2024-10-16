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
    public DcMotor outtakeSlidesRight;
    public DcMotor outtakeSlidesLeft;

    public Servo intakeTilt;
    public Servo claw;
    public Servo intakeSlidesLeft;
    public Servo intakeSlidesRight;
    public Servo armLeft;
    public Servo armRight;

    public ThreadHandler th_intake;
    public ThreadHandler th_intakeTilt;
    public ThreadHandler th_outtake;

    private final double kP = 0.01;
    private final double kI = 0;
    private final double kD = 0.002;
    private final double kF = 0.1;

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
        intakeSlidesLeft = hardwareMap.get(Servo.class, "intakeSlidesLeft");
        intakeSlidesRight = hardwareMap.get(Servo.class, "intakeSlidesRight");

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
    Thread intakeExtend = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

            intakeSlidesLeft.setPosition(1);
            intakeSlidesRight.setPosition(-1);

        }

    });

    Thread intakeRetract = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

            intakeSlidesLeft.setPosition(-1);
            intakeSlidesRight.setPosition(1);

        }

    });

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

    Thread outtake_up = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

                armLeft.setPosition(1);
                armRight.setPosition(1);
                outtakeSlidesLeft.setPower(1);
                outtakeSlidesRight.setPower(1);

            }

    });

    Thread outtake_down_full = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

            armLeft.setPosition(-1);
            armRight.setPosition(-1);
            outtakeSlidesLeft.setPower(-1);
            outtakeSlidesRight.setPower(-1);

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

    Thread outtake_down_specimen = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

            armLeft.setPosition(.5);
            armRight.setPosition(.5);
            outtakeSlidesLeft.setPower(-1);
            outtakeSlidesRight.setPower(-1);

        }

    });

    Thread outtake_up_specimen = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

            armLeft.setPosition(.5);
            armRight.setPosition(.5);
            outtakeSlidesLeft.setPower(1);
            outtakeSlidesRight.setPower(1);

        }

    });

    Thread clawOpen = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

            claw.setPosition(1);

        }

    });

    Thread clawClose = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

            claw.setPosition(0);

        }

    });


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

        telemetry.addData("fr:", fr.getPower());
        telemetry.addData("br:", br.getPower());
        telemetry.addData("fl:", fl.getPower());
        telemetry.addData("bl:", bl.getPower());

        telemetry.addData("rStickx:", right_stick_x);
        telemetry.addData("lStickx:", left_stick_x);
        telemetry.addData("lSticky:", left_stick_x);
        telemetry.update();
    }

    public void horizSlides() {
        if (gamepad2.left_bumper) {
            th_intake.queue(intakeExtend);
            th_intake.queue(intake_TiltDown);
        } else if (gamepad2.right_bumper) {
            th_intake.queue(intake_TiltUp);
            th_intake.queue(intakeRetract);
        }
    }

    public void verticalSlides(){
           if (gamepad2.left_stick_y > .2) {
               th_outtake.queue(outtake_up);
           }
           else if (intakeSlidesRight.getPosition() == 0 && gamepad2.left_stick_y < .2 ){
               th_outtake.queue(outtake_down_partial);
           }
           else if (gamepad2.left_stick_y < .2){
               th_outtake.queue(outtake_down_full);
           }
           else if (gamepad2.left_bumper){
               th_outtake.queue(outtake_up_specimen);
           }
           else if (gamepad2.right_bumper){
               th_outtake.queue(outtake_down_specimen);
           }
           else {
               holdPos();
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

        double position = outtakeSlidesLeft.getCurrentPosition();

        lowPass = a * position + (1 - a) * lowPass;

        double error = targetPosition - lowPass;
        integralSum += error * timer.seconds();
        integralSum = Math.max(-1.0, Math.min(1.0, integralSum));
        double derivative = (error - previousError) / timer.seconds();
        double feedforward = kF * targetPosition;

        // Compute motor power
        double power = (kP * error) + (kI * integralSum) + (kD * derivative) + feedforward;
        power = Math.max(-1.0, Math.min(1.0, power));  // Clamp power to [-1, 1]

        outtakeSlidesLeft.setPower(power);
        outtakeSlidesRight.setPower(power);

        previousError = error;
        timer.reset();
    }

    public void claw (){
        if(gamepad2.a && claw.getPosition() != 0){
            th_outtake.queue(clawOpen);
        }
        else if (gamepad2.a){
            th_outtake.queue(clawClose);
        }
    }

    public void intake(){
        if(gamepad2.b){
            //th_intake.queue(intake_in);
            intake.setPower(1);
        }
        else if (gamepad2.y){
           // th_intake.queue(intake_out);
            intake.setPower(-1);
        }
        else{
            intake.setPower(0);
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
