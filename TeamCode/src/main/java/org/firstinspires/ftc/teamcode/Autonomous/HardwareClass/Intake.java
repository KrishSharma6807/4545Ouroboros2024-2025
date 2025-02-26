package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    private NormalizedColorSensor colorSensor;
    private String detectedColor = "unknown";
    private float[] hsv = new float[3];
    private boolean autoRetractEnabled = true;
    private ElapsedTime colorCheckTimer = new ElapsedTime();

    ElapsedTime timer;

    private DcMotorEx intake;
    public Servo intakeTiltLeft;
    public Servo intakeTiltRight;

    public static double intakeTiltLeftUp = .4;
    public static double intakeTiltRightUp = .02;

    public static double intakeTiltLeftDown = .2;
    public static double intakeTiltRightDown = .35;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeTiltLeft = hardwareMap.get(Servo.class, "intakeTiltLeft");
        intakeTiltRight = hardwareMap.get(Servo.class, "intakeTiltRight");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(4);

        timer = new ElapsedTime();
    }
    //--------------------------------------------------------
    public class IntakeIn implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setPower(1);
                initialized = true;
            }
            double vel = intake.getVelocity();
            packet.put("IntakeVel", vel);
            intake.setPower(1);
            if(intake.getCurrent(CurrentUnit.MILLIAMPS) > 10){

            }
            return false;
        }
    }
    public Action intakeIn() {
        return new IntakeIn();
    }
    //--------------------------------------------------------
    public class IntakeOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double vel = intake.getVelocity();
            packet.put("IntakeVel", vel);
            intake.setPower(-1);
            return false;
        }
    }
    public Action intakeOut(){
        return new IntakeOut();
    }
    //--------------------------------------------------------
    public class IntakeStop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double vel = intake.getVelocity();
            packet.put("IntakeVel", vel);
            intake.setPower(0);
            return false;
        }
    }
    public Action intakeStop(){
        return new IntakeStop();
    }
    //--------------------------------------------------------
    public class IntakeTiltUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeTiltLeft.setPosition(intakeTiltLeftUp);
            intakeTiltRight.setPosition(intakeTiltRightUp);
            return false;
        }
    }
    public Action intakeTiltUp(){
        return new IntakeTiltUp();
    }
    //--------------------------------------------------------
    public class IntakeTiltDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeTiltLeft.setPosition(intakeTiltLeftDown);
            intakeTiltRight.setPosition(intakeTiltRightDown);
            return false;
        }
    }
    public Action intakeTiltDown(){
        return new IntakeTiltDown();
    }

    //--------------------------------------------------------
    public class IntakeInSample implements Action {
        private final ElapsedTime timer = new ElapsedTime();

        public IntakeInSample() {
            timer.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!isSampleDetected() && timer.seconds() < 2) {
                intake.setPower(1);
                return true; // Keep running
            } else {
                intake.setPower(0);
                return false; // Stops the action
            }
        }
    }

    public Action intakeInSample() {
        return new IntakeInSample();
    }

    private boolean isSampleDetected() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);
        String color = classifyColor(hsv[0], hsv[1], hsv[2]);
        return !color.equals("unknown");
    }

    private String classifyColor(float hue, float saturation, float value) {
        // (Same classification logic as before)
        if (saturation < 0.3) return "unknown";
        if ((hue >= 0 && hue <= 64) || (hue >= 330 && hue <= 360)) return "red";
        else if (hue >= 200 && hue <= 260) return "blue";
        else if (hue >= 65 && hue <= 100) return "yellow";
        return "unknown";
    }
}