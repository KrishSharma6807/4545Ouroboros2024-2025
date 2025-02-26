package org.firstinspires.ftc.teamcode.threadTeleOp;

import static org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.OuttakeSlides.rightEncoder;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.OuttakeSlides;
import org.firstinspires.ftc.teamcode.ThreadHandler;
import org.firstinspires.ftc.teamcode.robotControl.CustomPID;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;

import java.util.List;

@Config
public abstract class ThreadTeleLibBLUE extends OpMode {
    FtcDashboard dashboard;

    private NormalizedColorSensor colorSensor;
    private String detectedColor = "unknown";
    private float[] hsv = new float[3];
    private boolean autoRetractEnabled = true;
    private ElapsedTime colorCheckTimer = new ElapsedTime();

    private boolean intakeActive = false;


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

    public static double kPe = .0009 ;
    public static double kIe = 0;
    public static double kDe= 0;
    public static double kFe = 0;
    CustomPID pidControllerHorizontal = new CustomPID(kPe, kIe, kDe, kFe);

    public static double kPeH = .0009 ;
    public static double kIeH = 0;
    public static double kDeH= 0;
    public static double kFeH = 0;
    CustomPID pidControllerHorizontalHold = new CustomPID(kPeH, kIeH, kDeH, kFeH);

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
    private ElapsedTime timeOutH = new ElapsedTime();

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

    public static double armLeft1Sample = .95;
    public static double armLeft2Sample = .32;//1 is max, .25 is min

    public static double armRight1Specimen = .15;
    public static double armRight2Specimen = 1;//.2 is max, 1 is min (tune min more)
    public static double armRight3Specimen = .55;

    public static double armLeft1Specimen = .83;
    public static double armLeft2Specimen = .3;
    public static double armLeft3Specimen = .8;

    public static double armRight1Sample = 0;
    public static double armRight2Sample = .9;

    public static double closeClawSpecimen = .5;
    public static double openClawSpecimen = 1;

    public static double intakeTiltLeftUp = .4;
    public static double intakeTiltRightUp = .02;

    public static double intakeTiltLeftDown = .2;
    public static double intakeTiltRightDown = .37;

    public static double closeClawSample = .5;
    public static double openClawSample = .85;

    public static double wrist1Specimen = .85;
    public static double wrist2Specimen = .085;
    public static double wrist3Specimen = .63;

    public static double wrist1Sample = .925;
    public static double wrist2Sample = .5;

    public static double clawSpin1 = .38;
    public static double clawSpin2 = .38;
    boolean armStateChange = false;
    private FtcDashboard dash;
    TelemetryPacket telemetryPacket;
    public enum OuttakeLiftState {
        LIFT_SAMPLE_HIGH,
        LIFT_SAMPLE_LOW,
        LIFT_SPECIMEN_PICKUP,
        LIFT_SPECIMEN_HIGH,
        LIFT_SPECIMEN_LOW
    };
    private enum OuttakeState {
        IDLE,
        MOVING_UP_INITIAL,
        CHECKING_SAMPLE,
        CHECKING_SAMPLE_WHILE_RAISING,
        RETRYING_LOWER,
        RETRYING_GRAB,
        MOVING_UP_RETRY,
        FINAL_CHECK,
        COMPLETE
    }

    private OuttakeState currentOuttakeState = OuttakeState.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();
    public enum ArmStateSpecimen {
        POSITION_1,
        POSITION_2,
        POSITION_3
    }
    private ArmStateSpecimen currentArmState = ArmStateSpecimen.POSITION_1;
    public void init() {
        dash = FtcDashboard.getInstance();

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(4);

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
        intakeSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

            targetPosition = 380;
            RunTOPos();
            clawSpin.setPosition(clawSpin1);


        }

    });

//    Thread outtake_down = new Thread(new Runnable() {
//
//        @Override
//        public void run() {
//
//            ElapsedTime time = new ElapsedTime();
//            time.reset();
//            while(time.milliseconds() < 300) {
//
//            }
//
//            armLeft.setPosition(armLeft1Sample);
//            armRight.setPosition(armRight1Sample);
//            wrist.setPosition(wrist1Sample);
//            claw.setPosition(openClawSample);
//            intakeTiltLeft.setPosition(intakeTiltLeftDown);
//            intakeTiltRight.setPosition(intakeTiltRightDown);
//            targetPosition = 285;
//            RunTOPos();
//        }
//
//    });

    Thread outtake_up_high_bucket = new Thread(new Runnable() {
        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            time.reset();
            // Initial delay for arm movement
            while (time.milliseconds() < 300) {}

            // Move arm, wrist, and slides to high position
            armLeft.setPosition(armLeft2Sample);
            armRight.setPosition(armRight2Sample);
            wrist.setPosition(wrist2Sample);
            targetPosition = 1600;
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

    Thread holdHorizontal = new Thread(new Runnable() {

        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            time.reset();
            horizPID();
        }

    });

    Thread horizontal = new Thread(new Runnable() {

        @Override
        public void run() {
            ElapsedTime time = new ElapsedTime();
            time.reset();
            runToHorizPID();
        }

    });


    public void update() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Only check color when intake is active and 100ms has elapsed
        if(intakeActive && colorCheckTimer.milliseconds() >= 100) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsv);
            detectedColor = classifyColor(hsv[0], hsv[1], hsv[2]);
            colorCheckTimer.reset();
        }

        // Existing update logic
        outtakeSlidesLeftPosition = outtakeSlidesLeft.getCurrentPosition();
        outtakeSlidesRightPosition = -outtakeSlidesRight.getCurrentPosition();
        intakeSlidesPosition = intakeSlides.getCurrentPosition();

        telemetryPacket.put("Detected Color", detectedColor);
        telemetryPacket.put("Intake Active", intakeActive);
        dash.sendTelemetryPacket(telemetryPacket);
    }

    public void telem(){
        telemetry.addData("Intake Active", intakeActive);
        telemetry.addData("Last Color", detectedColor);
        telemetry.addData("Color Check Interval", colorCheckTimer.milliseconds());

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

        telemetry.addData("Target Position", targetPositionHorizontal);

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
    public void disableArm(){
        if(gamepad1.x){
            armLeft.getController().pwmDisable();
            armRight.getController().pwmDisable();
        }
        if(gamepad1.y){
            armLeft.getController().pwmEnable();
            armRight.getController().pwmEnable();
        }
    }
    public void ArcadeDriveBar() {

        double left_stick_x = gamepad1.left_stick_x * -1;
        double left_stick_y = gamepad1.left_stick_y * -1;
        double right_stick_x = gamepad1.right_stick_x;

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        double denominator = Math.max(Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_x), 1);

        if (Math.abs(left_stick_x) > 0.1 ||
                Math.abs(left_stick_y) >.1|| Math.abs(right_stick_x) > 0.1){
            fr.setPower(/*voltage/13.5 **/ ((left_stick_y + left_stick_x) + right_stick_x) );
            fl.setPower(((left_stick_y - left_stick_x) - right_stick_x));
            br.setPower(((left_stick_y - left_stick_x) + right_stick_x) );
            bl.setPower(((left_stick_y + left_stick_x) - right_stick_x) );
        }
        else{
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
            bl.setPower(0);
        }
    }

    public void ArcadeDriveIntake() {

        double left_stick_x = gamepad1.left_stick_x;
        double left_stick_y = gamepad1.left_stick_y;
        double right_stick_x = gamepad1.right_stick_x;

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        double denominator = Math.max(Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_x), 1);

        if (Math.abs(left_stick_x) > 0.1 ||
                Math.abs(left_stick_y) >.1|| Math.abs(right_stick_x) > 0.1){
            fr.setPower(/*voltage/13.5 **/ ((left_stick_y + left_stick_x) + right_stick_x) );
            fl.setPower(((left_stick_y - left_stick_x) - right_stick_x) );
            br.setPower(((left_stick_y - left_stick_x) + right_stick_x) );
            bl.setPower(((left_stick_y + left_stick_x) - right_stick_x) );
        }
        else{
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
            bl.setPower(0);
        }
    }

    public void horizSlides() {
        // Existing manual controls
        if (gamepad2.left_trigger > .2) {
            intakeSlides.setPower(-gamepad2.left_trigger);
            targetPositionHorizontal = intakeSlidesPosition;
        } else if (gamepad2.right_trigger > .2) {
            intakeSlides.setPower(gamepad2.right_trigger);
            targetPositionHorizontal = intakeSlidesPosition;
        }
        else if (gamepad2.dpad_right){
            targetPositionHorizontal = -1000;
            th_intake.queue(horizontal);
        }
        else if (gamepad2.dpad_left){
            targetPositionHorizontal = 0;
            th_intake.queue(horizontal);
        }
        else {
            th_intake.queue(holdHorizontal);
        }

        // Auto-retract only when intake is active
        if(intakeActive) {
            if(detectedColor.equals("blue") && intakeSlidesPosition < -100) {
                intakeTiltLeft.setPosition(intakeTiltLeftUp);
                intakeTiltRight.setPosition(intakeTiltRightUp);
                targetPositionHorizontal = 0;
                th_intake.queue(horizontal);
                detectedColor = "unknown"; // Reset to prevent repeated triggers
            }
        }
    }

    public void intakeTilt() {

        if (gamepad2.x && !intakeTiltTogglePressed) {
            intakeTiltTogglePressed = true;

            if (isIntakeTiltUp) {
                intakeTiltLeft.setPosition(intakeTiltLeftUp);
                intakeTiltRight.setPosition(intakeTiltRightUp);
            } else {
                intakeTiltLeft.setPosition(intakeTiltLeftDown);
                intakeTiltRight.setPosition(intakeTiltRightDown);
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
                    claw.setPosition(openClawSample);
                    intakeTiltLeft.setPosition(intakeTiltLeftDown);
                    intakeTiltRight.setPosition(intakeTiltRightDown);
                    targetPosition = 285;
                    RunTOPos();
                }

            });
            if(gamepad2.start){
                outtakeSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtakeSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
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
            th_outtake.queue(outtake_up_high_bucket);
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
                claw.setPosition(openClawSample);
            } else {
                claw.setPosition(closeClawSample);
            }

            isClawOpen = !isClawOpen;
        } else if (!gamepad2.right_bumper) {
            clawTogglePressed = false;
        }
    }

    public void armSpecimen(){
        if (gamepad2.a && !armTogglePressed) {
            armTogglePressed = true;
            switch (currentArmState) {
                case POSITION_1:
                    armLeft.setPosition(armLeft1Specimen);
                    armRight.setPosition(armRight1Specimen);
                    wrist.setPosition(wrist1Specimen);
                    claw.setPosition(closeClawSpecimen);
                    if(!armStateChange){
                        armStateChange = true;
                        currentArmState = ArmStateSpecimen.POSITION_2;
                    }

                    break;
                case POSITION_2:
                    armLeft.setPosition(armLeft2Specimen);
                    armRight.setPosition(armRight2Specimen);
                    wrist.setPosition(wrist2Specimen);
                    claw.setPosition(closeClawSpecimen);
                    if(!armStateChange){
                        armStateChange = true;
                        currentArmState = ArmStateSpecimen.POSITION_1;
                    }
                    break;
//                case POSITION_3:
//                    armLeft.setPosition(armLeft3Specimen);
//                    armRight.setPosition(armRight3Specimen);
//                    wrist.setPosition(wrist3Specimen);
//                    claw.setPosition(closeClawSpecimen);
//                    currentArmState = ArmStateSpecimen.POSITION_1;
//                    break;
            }
        } else if (!gamepad2.a) {
            armStateChange = false;
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
                claw.setPosition(closeClawSpecimen);
            } else {
                armLeft.setPosition(armLeft2Sample);
                armRight.setPosition(armRight2Sample);
//                wrist.getController().pwmEnable();
                wrist.setPosition(wrist2Sample);
                claw.setPosition(closeClawSpecimen);
            }

            isArmDown = !isArmDown;
        } else if (!gamepad2.a) {
            armTogglePressed = false;
        }

    }

//    public void wrist(){
//        if(gamepad2.dpad_left){
//            wrist.setPosition(0);
//        }
//        else if (gamepad2.dpad_right){
//            wrist.setPosition(1);
//        }
//    }

    public void intake() {
        if(gamepad2.y) {
            intake.setPower(1);
            intakeActive = false;
        } else if (gamepad2.b) {
            intake.setPower(-1);
            intakeActive = true;
        } else {
            intake.setPower(0);
            intakeActive = false;
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

            power = Math.max(-1.0, Math.min(1.0, power)) + .1;

            // Set motor power
            outtakeSlidesLeft.setPower(-power);
            outtakeSlidesRight.setPower(-power);

            // Check if error is within tolerance
            if (Math.abs(error) <= 20) {
                break;  // Exit loop if within tolerance
            }
        }
        targetPosition = outtakeSlidesRightPosition;
        holdPos();
    }

    public void horizPID(){
        pidControllerHorizontalHold.timer.reset();
        // Reset the PID controller's internal timer


        // Get current position
        double currentPos = intakeSlidesPosition;
        double error = targetPositionHorizontal - currentPos;

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // Calculate power to hold position using CustomPID controller
        double power = pidControllerHorizontalHold.calculatePower(targetPositionHorizontal, currentPos);

        power = Math.max(-1.0, Math.min(1.0, power));

        // Set motor power to hold position
        intakeSlides.setPower(-(voltage/13.5) * power);
    }

    public void runToHorizPID(){
        timeOutH.reset();
        while(timeOutH.seconds() < 2) {

            // Get current position
            double currentPos = intakeSlidesPosition;
            double error = targetPositionHorizontal - currentPos;


            // Calculate power to hold position using CustomPID controller
            double power = pidControllerHorizontal.calculatePower(targetPositionHorizontal, currentPos);

            power = Math.max(-1.0, Math.min(1.0, power));


            // Set motor power to hold position
            intakeSlides.setPower(-(voltage/13.5)* power);

            if (Math.abs(error) <= 5) {
                break;  // Exit loop if within tolerance
            }
        }
        targetPositionHorizontal = intakeSlidesPosition;
    }

    private String classifyColor(float hue, float saturation, float value) {
        // (Same classification logic as before)
        if (saturation < 0.3) return "unknown";
        if ((hue >= 0 && hue <= 64) || (hue >= 330 && hue <= 360)) return "red";
        else if (hue >= 200 && hue <= 260) return "blue";
        else if (hue >= 65 && hue <= 100) return "yellow";
        return "unknown";
    }

    private boolean isSampleDetected() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);
        String color = classifyColor(hsv[0], hsv[1], hsv[2]);
        return !color.equals("unknown");
    }

    public void updateOuttakeStateMachine() {
        switch (currentOuttakeState) {
            case IDLE:
                break;

            case MOVING_UP_INITIAL:
                targetPosition = 1500; // Start raising
                currentOuttakeState = OuttakeState.CHECKING_SAMPLE_WHILE_RAISING;
                stateTimer.reset();
                break;

            case CHECKING_SAMPLE_WHILE_RAISING:
                if (stateTimer.milliseconds() > 200) { // Allow sensor time to stabilize
                    if (!isSampleDetected()) {
                        // No sample detected, assume successful transfer
                        currentOuttakeState = OuttakeState.COMPLETE;
                    } else {
                        // Sample still in intake, lower before retrying
                        targetPosition = 285;
                        currentOuttakeState = OuttakeState.RETRYING_LOWER;
                    }
                }
                break;

            case RETRYING_LOWER:
                if (Math.abs(outtakeSlidesRightPosition - targetPosition) <= 20) {
                    claw.setPosition(openClawSample); // Open claw
                    stateTimer.reset();
                    currentOuttakeState = OuttakeState.RETRYING_GRAB;
                }
                break;

            case RETRYING_GRAB:
                if (stateTimer.milliseconds() > 500) {
                    claw.setPosition(closeClawSample); // Close claw
                    targetPosition = 1500; // Move up again
                    currentOuttakeState = OuttakeState.MOVING_UP_RETRY;
                }
                break;

            case MOVING_UP_RETRY:
                if (Math.abs(outtakeSlidesRightPosition - targetPosition) <= 20) {
                    stateTimer.reset();
                    currentOuttakeState = OuttakeState.FINAL_CHECK;
                }
                break;

            case FINAL_CHECK:
                if (stateTimer.milliseconds() > 200) {
                    if (!isSampleDetected()) {
                        // If sample is confirmed in claw, don't go back down
                        currentOuttakeState = OuttakeState.COMPLETE;
                    } else {
                        // If sample is missing, retry the process
                        targetPosition = 285;
                        claw.setPosition(openClawSample);
                        currentOuttakeState = OuttakeState.COMPLETE;
                    }
                }
                break;

            case COMPLETE:
                // Ensure the arm only moves after confirming sample presence
                if (!isSampleDetected()) {
                    armLeft.setPosition(armLeft2Sample);
                    armRight.setPosition(armRight2Sample);
                    wrist.setPosition(wrist2Sample);
                }
                // Reset or prepare for next cycle
                break;
        }

        // Ensure PID runs continuously
        updateVerticalPID();
    }

    public void updateVerticalPID() {
        double power = pidController.calculatePower(targetPosition, outtakeSlidesRightPosition);
        outtakeSlidesLeft.setPower(-power);
        outtakeSlidesRight.setPower(-power);
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