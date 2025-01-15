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

public class OuttakeSlides {
    private DcMotorEx outtakeSlidesLeft;
    private DcMotorEx outtakeSlidesRight;
    public DcMotorEx intakeSlides;

    public static double kP = 0.033;
    public static double kI = 0;
    public static double kD = 0.0;
    CustomPID pidController = new CustomPID(kP, kI, kD, 0.0);



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

    public class LiftUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                double power = pidController.calculatePower(1560, -outtakeSlidesRight.getCurrentPosition());
                outtakeSlidesLeft.setPower(-power);
                outtakeSlidesRight.setPower(-power);
                intakeSlides.setPower(-1);
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
                intakeSlides.setPower(-1);
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
            if (pos > 650) {
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
}

