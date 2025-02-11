package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotControl.CustomPID;

import page.j5155.expressway.core.actions.InitLoopAction;

public class OuttakeSlides {
    private DcMotorEx outtakeSlidesLeft;
    private DcMotorEx outtakeSlidesRight;
    public DcMotorEx intakeSlides;

    public static double kP = .0047;
    public static double kI = 0.0;
    public static double kD = .0;
    public static double kF = 0.15;
    CustomPID pidController = new CustomPID(kP, kI, kD, kF);



    public OuttakeSlides(HardwareMap hardwareMap) {
        outtakeSlidesLeft = hardwareMap.get(DcMotorEx.class, "outtakeSlidesLeft");
        outtakeSlidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSlidesRight = hardwareMap.get(DcMotorEx.class, "outtakeSlidesRight");
        outtakeSlidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSlidesLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeSlidesRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeSlides = hardwareMap.get(DcMotorEx.class, "intakeSlides");


        outtakeSlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        outtakeSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    class PIDActionUpSpecimen extends InitLoopAction {
        private final int target = 1370;  // Consistent target

        @Override
        public void init() {

        }

        @Override
        public boolean loop(TelemetryPacket p) {
            int position = -outtakeSlidesRight.getCurrentPosition();
            double power = pidController.calculatePower(target, position);

            // Clamp power output
            power = Math.max(-1, Math.min(power, 1));

            outtakeSlidesLeft.setPower(-power);
            outtakeSlidesRight.setPower(-power);

            // Allow small tolerance
            return Math.abs(target - position) > 15;
        }
    }
    public Action liftUpPID() {
        return new PIDActionUpSpecimen();
    }

    class PIDActionGrab extends InitLoopAction {
        private int target;

        public PIDActionGrab() {
            this.target = target;
        }

        @Override
        public void init() {

        }

        @Override
        public boolean loop(TelemetryPacket p) {
            int position = -outtakeSlidesRight.getCurrentPosition();
            double power = pidController.calculatePower(450, -outtakeSlidesRight.getCurrentPosition());
            target = 450;
            p.put("Motor Info", "Target: " + target + "; Error " + (target - position) + "; Power: " + power + "; currentPos" + position);

            outtakeSlidesLeft.setPower(-power);
            outtakeSlidesRight.setPower(-power);
            if (!(position < (target - 20) || position > (target + 20))){
                outtakeSlidesLeft.setPower(0);
                outtakeSlidesRight.setPower(0);
            }
            return Math.abs(target - position) > 25;
        }
    }
    public Action PIDGrab() {
        return new PIDActionGrab();
    }

    class liftUpPartialPID extends InitLoopAction {
        private int target;

        public liftUpPartialPID() {
            this.target = target;
        }

        @Override
        public void init() {
        }

        @Override
        public boolean loop(TelemetryPacket p) {
            int position = -outtakeSlidesRight.getCurrentPosition();
            double power = pidController.calculatePower(580, -outtakeSlidesRight.getCurrentPosition());
            target = 580;
            p.put("Motor Info", "Target: " + target + "; Error " + (target - position) + "; Power: " + power);

            outtakeSlidesLeft.setPower(-power);
            outtakeSlidesRight.setPower(-power);
            if (!(position < (target - 20) || position > (target + 20))){
                outtakeSlidesLeft.setPower(-0.005);
                outtakeSlidesRight.setPower(-0.005);
            }
            return Math.abs(target - position) > 35;
        }
    }
    public Action liftUpPartialPID() {
        return new liftUpPartialPID();
    }

    class PIDActionDownFull extends InitLoopAction {
        private int target;

        public PIDActionDownFull() {
            this.target = target;
        }

        @Override
        public void init() {

        }

        @Override
        public boolean loop(TelemetryPacket p) {
            int position = -outtakeSlidesRight.getCurrentPosition();
            double power = pidController.calculatePower(0, -outtakeSlidesRight.getCurrentPosition());
            target = 0;
            p.put("Motor Info", "Target: " + target + "; Error " + (target - position) + "; Power: " + power);

            outtakeSlidesLeft.setPower(-power);
            outtakeSlidesRight.setPower(-power);
            if (!(position < (target - 20) || position > (target + 20))){
                outtakeSlidesLeft.setPower(-0.005);
                outtakeSlidesRight.setPower(-0.005);
            }
            return Math.abs(target - position) > 25;
        }
    }
    public Action PIDDownFull() {
        return new PIDActionDownFull();
    }

    public class LiftUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                double power = pidController.calculatePower(1560, -outtakeSlidesRight.getCurrentPosition());
                outtakeSlidesLeft.setPower(-power);
                outtakeSlidesRight.setPower(-power);
                initialized = true;
            }

            double pos = -outtakeSlidesRight.getCurrentPosition();
            packet.put("liftPos", pos);
            packet.put("power", outtakeSlidesLeft.getPower());
            if (pos < 1450) {
                return true;
            } else {
                outtakeSlidesLeft.setPower(0);
                outtakeSlidesRight.setPower(0);
                return false;
            }
        }
    }
    public Action liftUp() {
        return new LiftUp();
    }

    public class LiftUpPartial implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                double power = pidController.calculatePower(1560, -outtakeSlidesRight.getCurrentPosition());
                outtakeSlidesLeft.setPower(-power);
                outtakeSlidesRight.setPower(-power);
                initialized = true;
            }

            double pos = -outtakeSlidesRight.getCurrentPosition();
            packet.put("liftPos", pos);
            packet.put("power", outtakeSlidesLeft.getPower());
            if (pos < 700) {
                return true;
            } else {
                outtakeSlidesLeft.setPower(0);
                outtakeSlidesRight.setPower(0);
                return false;
            }
        }
    }
    public Action liftUpPartial() {
        return new LiftUpPartial();
    }

    public class LiftDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {

                double power = pidController.calculatePower(0, -outtakeSlidesRight.getCurrentPosition());
                outtakeSlidesLeft.setPower(-power);
                outtakeSlidesRight.setPower(-power);


                initialized = true;
            }

            double pos = -outtakeSlidesRight.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos > 630) {
                return true;
            } else {
                outtakeSlidesLeft.setPower(0);
                outtakeSlidesRight.setPower(0);
                return false;
            }
        }
    }
    public Action liftDown() {
        return new LiftDown();
    }

    public class LiftDownFull implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {

                double power = pidController.calculatePower(0, -outtakeSlidesRight.getCurrentPosition());
                outtakeSlidesLeft.setPower(-power);
                outtakeSlidesRight.setPower(-power);


                initialized = true;
            }

            double pos = -outtakeSlidesRight.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos > 10) {
                return true;
            } else {
                outtakeSlidesLeft.setPower(0);
                outtakeSlidesRight.setPower(0);
                return false;
            }
        }
    }
    public Action liftDownFull() {
        return new LiftDownFull();
    }

    public class returnIntakeSlides implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeSlides.setPower(-.5);
            return false;
        }
    }

    public Action returnIntakeSlides() {
        return new returnIntakeSlides();
    }
}

