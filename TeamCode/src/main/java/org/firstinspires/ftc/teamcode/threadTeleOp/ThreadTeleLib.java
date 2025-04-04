package org.firstinspires.ftc.teamcode.threadTeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ThreadHandler;
import org.firstinspires.ftc.teamcode.robotControl.CustomPID;

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
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    CustomPID pidController = new CustomPID(kP, kI, kD, 0.0);

    public static  double kPh = .00001;
    public static  double kIh = 0;
    public static  double kDh = 0;
    CustomPID pidControllerHold = new CustomPID(kPh, kIh, kDh, kF);

    public static  double kF = 0.005;

    private double lowPass = 0;
    private final double a = 0.1;

    private double targetPosition = 0;  // Target encoder position for PIDF
    private double integralSum = 0;
    private double previousError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timeOut = new ElapsedTime();

    boolean isArmDown = true;
    boolean armTogglePressed = false;

    boolean isIntakeTiltUp = false;
    boolean intakeTiltTogglePressed = false;

    boolean isClawClosed = false;
    boolean clawTogglePressed = false;

    private boolean targetPositionSet = false;

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
            armLeft.setPosition(.6);
            wristLeft.setPosition(.7);
            targetPosition = -2500;
            RunTOPos();


        }

    });

    Thread outtake_down = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

            armLeft.setPosition(.15);
            wristLeft.setPosition(0);
            targetPosition = -10;
            RunTOPos();
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

        telemetry.addData("claw", claw.getPosition());

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

        double denominator = Math.max(Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_x), 1);

        if (Math.abs(left_stick_x) > 0.1 ||
                Math.abs(left_stick_y) >.1|| Math.abs(right_stick_x) > 0.1){
            fr.setPower(((left_stick_y + left_stick_x) + right_stick_x) / denominator);
            fl.setPower(((left_stick_y - left_stick_x) - right_stick_x) / denominator);
            br.setPower(((left_stick_y - left_stick_x) + right_stick_x) / denominator);
            bl.setPower(((left_stick_y + left_stick_x) - right_stick_x) / denominator);
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

        if (gamepad2.x && !intakeTiltTogglePressed) {
            intakeTiltTogglePressed = true;

            if (isIntakeTiltUp) {
                intakeTilt.setPosition(0.27);
            } else {
                intakeTilt.setPosition(0.65);
            }

            isIntakeTiltUp = !isIntakeTiltUp;
        } else if (!gamepad2.x) {
            intakeTiltTogglePressed = false;
        }

    }

    public void verticalSlides(){

        if (gamepad2.left_stick_y > .2) {
            outtakeSlidesLeft.setPower(1);
            outtakeSlidesRight.setPower(-1);
            targetPositionSet = false;
        }
        else if (gamepad2.left_stick_y < -.2) {
            outtakeSlidesLeft.setPower(-1);
            outtakeSlidesRight.setPower(1);
            targetPositionSet = false;
        }
        else if (gamepad2.dpad_up) {
//                armLeft.setPosition(.6);
//                wristLeft.setPosition(.7);
//                targetPositionSet = false;
            th_outtake.queue(outtake_up_high_bucket);
        }
        else if (gamepad2.dpad_down) {
            th_outtake.queue(outtake_down);
            targetPositionSet = false;
        }
        else {
            if (!targetPositionSet) {
                targetPosition = outtakeSlidesLeft.getCurrentPosition();
                targetPositionSet = true;
            }
            holdPos();
        }

    }



    public void claw (){

        if (gamepad2.right_bumper) {
            claw.setPosition(.9); // Close claw
        } else if (gamepad2.left_bumper) {
            claw.setPosition(0.7); // Open claw
        }
    }

    public void arm(){
        if (gamepad2.a && !armTogglePressed) {
            armTogglePressed = true;

            if (isArmDown) {
                armLeft.setPosition(0.8);
                wristLeft.setPosition(1);
            } else {
                claw.setPosition(0);
                armLeft.setPosition(0);
//                wristLeft.getController().pwmEnable();
                wristLeft.setPosition(0);
            }

            isArmDown = !isArmDown;
        } else if (!gamepad2.a) {
            armTogglePressed = false;
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
        }
        else if (gamepad2.y){
            intake.setPower(1);
        }
        else{
            intake.setPower(0);
        }
    }

    public void holdPos(){

        pidController.timer.reset();  // Reset the PID controller's internal timer


        // Get current position
        double currentPos = outtakeSlidesLeft.getCurrentPosition();
        double error = targetPosition - currentPos;

        // Update telemetry
        telemetry.addData("Error", error);
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", currentPos);
        telemetry.addData("Filtered Position", pidController.lowPass);
        telemetry.update();

        // Calculate power to hold position using CustomPID controller
        double power = pidControllerHold.calculatePower(targetPosition, currentPos);

        power = Math.max(-1.0, Math.min(1.0, power));

        // Set motor power to hold position
        outtakeSlidesLeft.setPower(power);
        outtakeSlidesRight.setPower(-power);
    }

    public void RunTOPos(){
        timeOut.reset();
        pidController.timer.reset();  // Reset the PID controller's internal timer

        while (timeOut.seconds() < 2) {
            // Current position and target error
            double currentPos = outtakeSlidesLeft.getCurrentPosition();
            double error = targetPosition - currentPos;

            // Update telemetry
            telemetry.addData("Error", error);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPos);
            telemetry.addData("Filtered Position", pidController.lowPass);
            telemetry.update();

            // Calculate power using CustomPID controller
            double power = pidController.calculatePower(targetPosition, currentPos);

            power = Math.max(-1.0, Math.min(1.0, power));

            // Set motor power
            outtakeSlidesLeft.setPower(power);
            outtakeSlidesRight.setPower(-power);

            // Check if error is within tolerance
            if (Math.abs(error) <= 5) {
                break;  // Exit loop if within tolerance
            }
        }
        targetPosition = outtakeSlidesLeft.getCurrentPosition();
        holdPos();
    }



    public void stop(){
        th_intake.th_kill();
        th_outtake.th_kill();
        th_intakeTilt.th_kill();

        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);
        intake.setPower(0);
        outtakeSlidesLeft.setPower(0);
        outtakeSlidesRight.setPower(0);
    }


}