package org.firstinspires.ftc.teamcode.threadTeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.ThreadHandler;
import org.firstinspires.ftc.teamcode.robotControl.CustomPID;

import java.util.List;

@Config
public abstract class ThreadTeleLibV2 extends OpMode {
    FtcDashboard dashboard;

    public DcMotorEx br;
    public DcMotorEx bl;
    public DcMotorEx fr;
    public DcMotorEx fl;
    public DcMotorEx intake;
    public DcMotorEx outtakeSlidesRight;
    public DcMotorEx outtakeSlidesLeft;
    public DcMotorEx intakeSlides;

    public Servo intakeTiltLeft;
    public Servo intakeTiltRight;
    public Servo claw;
    public Servo armLeft;
    public Servo armRight;
    public Servo wrist;
    public Servo clawSpin;

    public ThreadHandler th_outtake;
    public ThreadHandler th_intake;

    //TelemetryPacket
    public static double kP = .004;
    public static double kI = 0;
    public static double kD = .001;
    CustomPID pidController = new CustomPID(kP, kI, kD, 0.0);

    public static double kPe = 0.023 ;
    public static double kIe = 0;
    public static double kDe= 0;
    public static double kFe = 0;
    CustomPID pidControllerHorizontal = new CustomPID(kPe, kIe, kDe, kFe);

    public static  double kPh = 0;
    public static  double kIh = 0;
    public static  double kDh = 0;
    CustomPID pidControllerHold = new CustomPID(kPh, kIh, kDh, kF);

    public static  double kF = 0.005;

    private double lowPass = 0;
    private final double a = 0.1;

    private double targetPosition = 0;  // Target encoder position for PIDF
    private double targetPositionHorizontal = 0;
    private double integralSum = 0;
    private double previousError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime timeOut = new ElapsedTime();

    boolean isArmDown = true;
    boolean armTogglePressed = false;

    boolean isIntakeTiltUp = false;
    boolean intakeTiltTogglePressed = false;

    boolean isClawOpen = false;
    boolean clawTogglePressed = false;

    private boolean targetPositionSet = false;

    double outtakeSlidesLeftPosition = 0.0;
    double outtakeSlidesRightPosition = 0.0;
    double intakeSlidesPosition = 0.0;

    double voltage = 0;

    public static double armLeft1Sample = .4;
    public static double armLeft2Sample = .91;//1 is max, .25 is min

    public static double armRight1Specimen = 1;
    public static double armRight2Specimen = .4;//.2 is max, 1 is min (tune min more)

    public static double armLeft1Specimen = .25;
    public static double armLeft2Specimen = .75;

    public static double armRight1Sample = .8;
    public static double armRight2Sample = .29;

    public static double closeClawSpecimen = .5;
    public static double openClawSpecimen = .67;

    public static double closeClawSample = .5;
    public static double openClawSample = .67;

    public static double wrist1Specimen = .31;
    public static double wrist2Specimen = .85;

    public static double wrist1Sample = 1;
    public static double wrist2Sample = .5;

    public static double clawSpin1 = .38;
    public static double clawSpin2 = .38;

    private FtcDashboard dash;
    TelemetryPacket telemetryPacket;
    public enum OuttakeLiftState {
        LIFT_SAMPLE_HIGH,
        LIFT_SAMPLE_LOW,
        LIFT_SPECIMEN_PICKUP,
        LIFT_SPECIMEN_HIGH,
        LIFT_SPECIMEN_LOW
    };
    public void init() {
        dash = FtcDashboard.getInstance();
        br = hardwareMap.get(DcMotorEx.class, "br");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtakeSlidesRight = hardwareMap.get(DcMotorEx.class, "outtakeSlidesRight");
        outtakeSlidesLeft = hardwareMap.get(DcMotorEx.class, "outtakeSlidesLeft");
        intakeSlides = hardwareMap.get(DcMotorEx.class, "intakeSlides");

        intakeTiltLeft = hardwareMap.get(Servo.class, "intakeTiltLeft");
        intakeTiltRight = hardwareMap.get(Servo.class, "intakeTiltRight");
        claw = hardwareMap.get(Servo.class, "claw");
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        wrist = hardwareMap.get(Servo.class, "wrist");
        clawSpin = hardwareMap.get(Servo.class, "clawSpin");

        th_intake = new ThreadHandler();
        th_outtake = new ThreadHandler();

        armRight.setDirection(Servo.Direction.REVERSE);
        armLeft.setDirection(Servo.Direction.REVERSE);

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);

        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeSlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        outtakeSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        outtakeSlidesLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeSlidesRight.setDirection(DcMotorSimple.Direction.REVERSE);


        lowPass = outtakeSlidesRightPosition;

        telemetryPacket = new TelemetryPacket();

        clawSpin.setPosition(clawSpin1);
    }
    // add threads here
    Thread outtake_up_high_specimen = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 150) {

            }
            armLeft.setPosition(armLeft2Specimen);
            armRight.setPosition(armRight2Specimen);

            wrist.setPosition(wrist2Specimen);

            targetPosition = 150;
            RunTOPos();

            clawSpin.setPosition(clawSpin2);


        }

    });

    Thread outtake_down_high_specimen = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }
            armLeft.setPosition(armLeft1Specimen);
            armRight.setPosition(armRight1Specimen);
            wrist.setPosition(wrist1Specimen);

            targetPosition = 330;
            RunTOPos();
            clawSpin.setPosition(clawSpin1);


        }

    });

    Thread outtake_down = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

            armLeft.setPosition(armLeft1Sample);
            armRight.setPosition(armRight1Sample);
            wrist.setPosition(wrist1Sample);
            targetPosition = 50;
            RunTOPos();
        }

    });

    Thread outtake_up_high_bucket = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while(time.milliseconds() < 300) {

            }

            armLeft.setPosition(armLeft2Sample);
            armRight.setPosition(armRight2Sample);
            wrist.setPosition(wrist2Sample);
            targetPosition = 1500;
            RunTOPos();
        }

    });

    Thread hold = new Thread(new Runnable() {

        @Override
        public void run() {

            ElapsedTime time = new ElapsedTime();
            time.reset();
//            while(time.milliseconds() < 300) {
//
//            }

            holdPos();
        }

    });


    public void update(){
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        outtakeSlidesLeftPosition = outtakeSlidesLeft.getCurrentPosition();
        outtakeSlidesRightPosition = -outtakeSlidesRight.getCurrentPosition();
        intakeSlidesPosition = intakeSlides.getCurrentPosition();

        telemetryPacket.put("error", targetPosition - outtakeSlidesRightPosition);
        telemetryPacket.put("power", outtakeSlidesRight.getPower());

        dash.sendTelemetryPacket(telemetryPacket);

    }

    public void telem(){
        telemetry.addData("fr:", fr.getPower());
        telemetry.addData("br:", br.getPower());
        telemetry.addData("fl:", fl.getPower());
        telemetry.addData("bl:", bl.getPower());

        telemetry.addData("rStickx:", gamepad1.right_stick_x);
        telemetry.addData("lStickx:", gamepad1.left_stick_x);
        telemetry.addData("lSticky:", gamepad1.left_stick_x);

        telemetry.addData("intakeTiltLeft:", intakeTiltLeft.getPosition());
        telemetry.addData("intakeTiltRight:", intakeTiltRight.getPosition());

        telemetry.addData("outtakeLeft",outtakeSlidesLeft.getPower());
        telemetry.addData("outtakeRight",outtakeSlidesRight.getPower());

        telemetry.addData("targ", targetPosition);

        telemetry.addData("CurrPosFiltered", lowPass);

        telemetry.addData("intakeSlides", intakeSlides.getPower());
        telemetry.addData("intakeSlidesPos", intakeSlides.getCurrentPosition());

        telemetry.addData("intake", intake.getPower());
        telemetry.addData("intakeCurrent", intake.getCurrent(CurrentUnit.MILLIAMPS));

        telemetry.addData("claw", claw.getPosition());

        telemetry.addData("armLeftPos", armLeft.getPosition());
        telemetry.addData("armRightPos", armRight.getPosition());
        telemetry.addData("wristPos", wrist.getPosition());

        telemetry.addData("clawSpin", clawSpin.getPosition());

        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", outtakeSlidesRightPosition);
        telemetry.addData("Filtered Position", lowPass);

        telemetry.update();
    }
    public void ArcadeDrive() {

        double left_stick_x = gamepad1.left_stick_x;
        double left_stick_y = gamepad1.left_stick_y;
        double right_stick_x = gamepad1.right_stick_x;

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        double denominator = Math.max(Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_x), 1);

        if (Math.abs(left_stick_x) > 0.1 ||
                Math.abs(left_stick_y) >.1|| Math.abs(right_stick_x) > 0.1){
            fr.setPower(voltage/13.5 * ((left_stick_y + left_stick_x) + right_stick_x) / denominator);
            fl.setPower(voltage/13.5 * ((left_stick_y - left_stick_x) - right_stick_x) / denominator);
            br.setPower(voltage/13.5 * ((left_stick_y - left_stick_x) + right_stick_x) / denominator);
            bl.setPower(voltage/13.5 * ((left_stick_y + left_stick_x) - right_stick_x) / denominator);
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
            intakeSlides.setPower(-gamepad2.left_trigger);
        } else if (gamepad2.right_trigger > .2) {
            intakeSlides.setPower(gamepad2.right_trigger);
        }
        else {
            intakeSlides.setPower(0);
//            targetPositionHorizontal = intakeSlides.getCurrentPosition();
//            horizPID();
        }

    }

    public void intakeTilt() {

        if (gamepad2.x && !intakeTiltTogglePressed) {
            intakeTiltTogglePressed = true;

            if (isIntakeTiltUp) {
                intakeTiltLeft.setPosition(1);
                intakeTiltRight.setPosition(1);
            } else {
                intakeTiltLeft.setPosition(.6);
                intakeTiltRight.setPosition(0.6);
            }

            isIntakeTiltUp = !isIntakeTiltUp;
        } else if (!gamepad2.x) {
            intakeTiltTogglePressed = false;
        }

    }

    public void verticalSlidesSample(){

            if (gamepad2.left_stick_y < -.2) {
                outtakeSlidesLeft.setPower(gamepad2.left_stick_y);
                outtakeSlidesRight.setPower(gamepad2.left_stick_y);
                targetPositionSet = false;
            }
            else if (gamepad2.left_stick_y > .2) {
                outtakeSlidesLeft.setPower(gamepad2.left_stick_y);
                outtakeSlidesRight.setPower(gamepad2.left_stick_y);
                targetPositionSet = false;
            }
            else if (gamepad2.dpad_up) {
                th_outtake.queue(outtake_up_high_bucket);
            }
            else if (gamepad2.dpad_down) {
                th_outtake.queue(outtake_down);
                targetPositionSet = false;
            }
            else {
                if (!targetPositionSet) {
                    targetPosition = outtakeSlidesRightPosition;
                    targetPositionSet = true;
                }
                holdPos();
            }

    }

    public void verticalSlidesSpecimen() {

        if (gamepad2.left_stick_y < -.2) {
            outtakeSlidesLeft.setPower(gamepad2.left_stick_y);
            outtakeSlidesRight.setPower(gamepad2.left_stick_y);
            targetPositionSet = false;
        } else if (gamepad2.left_stick_y > .2) {
            outtakeSlidesLeft.setPower(gamepad2.left_stick_y);
            outtakeSlidesRight.setPower(gamepad2.left_stick_y);
            targetPositionSet = false;
        } else if (gamepad2.dpad_up) {
            th_outtake.queue(outtake_up_high_specimen);
        } else if (gamepad2.dpad_down) {
            th_outtake.queue(outtake_down_high_specimen);
            targetPositionSet = false;
        } else {
            if (!targetPositionSet) {
                targetPosition = outtakeSlidesRightPosition;
                targetPositionSet = true;
            }
            th_outtake.queue(hold);
        }
    }

    public void clawSpecimen (){
        if (gamepad2.right_bumper && !clawTogglePressed) {
            clawTogglePressed = true;

            if (isClawOpen) {
                claw.setPosition(openClawSpecimen);
            } else {
                claw.setPosition(closeClawSpecimen);
            }

            isClawOpen = !isClawOpen;
        } else if (!gamepad2.right_bumper) {
            clawTogglePressed = false;
        }
    }

    public void clawSample (){
        if (gamepad2.right_bumper && !clawTogglePressed) {
            clawTogglePressed = true;

            if (isClawOpen) {
                claw.setPosition(openClawSpecimen);
            } else {
                claw.setPosition(closeClawSpecimen);
            }

            isClawOpen = !isClawOpen;
        } else if (!gamepad2.right_bumper) {
            clawTogglePressed = false;
        }
    }



    public void armSpecimen(){
        if (gamepad2.a && !armTogglePressed) {
            armTogglePressed = true;

            if (isArmDown) {
                armLeft.setPosition(armLeft1Specimen);
                armRight.setPosition(armRight1Specimen);
                wrist.setPosition(wrist1Specimen);
            } else {
                armLeft.setPosition(armLeft2Specimen);
                armRight.setPosition(armRight2Specimen);
                wrist.setPosition(wrist2Specimen);
            }

            isArmDown = !isArmDown;
        } else if (!gamepad2.a) {
            armTogglePressed = false;
        }

    }

    public void armSample(){
        if (gamepad2.a && !armTogglePressed) {
            armTogglePressed = true;

            if (isArmDown) {
                armLeft.setPosition(armLeft1Sample);
                armRight.setPosition(armRight1Sample);
                wrist.setPosition(wrist1Sample);
            } else {
                armLeft.setPosition(armLeft2Sample);
                armRight.setPosition(armRight2Sample);
//                wrist.getController().pwmEnable();
                wrist.setPosition(wrist2Sample);
            }

            isArmDown = !isArmDown;
        } else if (!gamepad2.a) {
            armTogglePressed = false;
        }

    }

    public void wrist(){
        if(gamepad2.dpad_left){
            wrist.setPosition(0);
        }
        else if (gamepad2.dpad_right){
            wrist.setPosition(1);
        }
    }

    public void intake(){
        if(gamepad2.y){
            intake.setPower(-1);
        }
        else if (gamepad2.b){
            intake.setPower(1);
        }
        else{
            intake.setPower(0);
        }
    }

    public void holdPos(){

        pidController.timer.reset();  // Reset the PID controller's internal timer


        // Get current position
        double currentPos = outtakeSlidesRightPosition;
        double error = targetPosition - currentPos;


        // Calculate power to hold position using CustomPID controller
        double power = pidControllerHold.calculatePower(targetPosition, currentPos);

        power = Math.max(-1.0, Math.min(1.0, power));

        // Set motor power to hold position
        outtakeSlidesLeft.setPower(-power);
        outtakeSlidesRight.setPower(-power);
    }

    public void RunTOPos(){
        timeOut.reset();
        pidController.timer.reset();  // Reset the PID controller's internal timer
        double error = targetPosition - outtakeSlidesRightPosition;
        while (timeOut.seconds() < 2) {
            // Current position and target error
            double currentPos = outtakeSlidesRightPosition;
            error = targetPosition - currentPos;

            // Calculate power using CustomPID controller
            double power = pidController.calculatePower(targetPosition, currentPos);

            power = Math.max(-1.0, Math.min(1.0, power));

            // Set motor power
            outtakeSlidesLeft.setPower(-power);
            outtakeSlidesRight.setPower(-power);

            // Check if error is within tolerance
            if (Math.abs(error) <= 5) {
                break;  // Exit loop if within tolerance
            }
        }
        targetPosition = outtakeSlidesRightPosition;
        holdPos();
    }

    public void horizPID(){
        pidControllerHorizontal.timer.reset();  // Reset the PID controller's internal timer


        // Get current position
        double currentPos = intakeSlidesPosition;
        double error = targetPosition - currentPos;


        // Calculate power to hold position using CustomPID controller
        double power = pidController.calculatePower(targetPositionHorizontal, currentPos);

        power = Math.max(-1.0, Math.min(1.0, power));

        // Set motor power to hold position
        intakeSlides.setPower(power);
    }


    public void stop(){
        th_intake.th_kill();
        th_outtake.th_kill();
//        th_intakeTilt.th_kill();

        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);
        intake.setPower(0);
        outtakeSlidesLeft.setPower(0);
        outtakeSlidesRight.setPower(0);
        intakeSlides.setPower(0);
    }


}