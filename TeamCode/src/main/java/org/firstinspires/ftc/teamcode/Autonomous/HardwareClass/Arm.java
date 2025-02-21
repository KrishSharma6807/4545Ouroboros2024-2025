package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Arm {
    public Servo armLeft;
    public Servo armRight;
    public Servo wrist;
    public Servo claw;
    public static double armRight1Specimen = .15;
    public static double armRight2Specimen = 1;//.2 is max, 1 is min (tune min more)


    public static double armLeft1Specimen = .83;
    public static double armLeft2Specimen = .3;

    public static double wrist1Specimen = .85;
    public static double wrist2Specimen = .085;

    public static double armLeft1Sample = .95;
    public static double armLeft2Sample = .32;

    public static double armRight1Sample = 0;
    public static double armRight2Sample = .9;

    public static double wrist1Sample = .925;
    public static double wrist2Sample = .5;

    private ElapsedTime timer = new ElapsedTime();

    public Arm(HardwareMap hardwareMap) {
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        armRight.setDirection(Servo.Direction.REVERSE);
        armLeft.setDirection(Servo.Direction.REVERSE);
    }

    public class UpSpecimenArm implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            timer.reset();
            while(timer.seconds() < .3){

            }
            armLeft.setPosition(armLeft2Specimen);
            armRight.setPosition(armRight2Specimen);
            wrist.setPosition(wrist2Specimen);
            return false;
        }
    }
    public Action upSpecimenArm() {
        return new UpSpecimenArm();
    }

    public class UpSpecimenArmFull implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
//            armLeft.setPosition(armLeft3Specimen);
//            armRight.setPosition(armRight3Specimen);
//            wrist.setPosition(wrist3Specimen);
            timer.reset();
            while(timer.seconds() < 1){

            }
            return false;
        }
    }
    public Action upSpecimenArmFull() {
        return new UpSpecimenArmFull();
    }

    public class DownSpecimenArm implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(1);
            timer.reset();
            while(timer.seconds() < 0.15){

            }
            wrist.setPosition(wrist1Specimen);
            armLeft.setPosition(armLeft1Specimen);
            armRight.setPosition(armRight1Specimen);
            return false;
        }
    }
    public Action downSpecimenArm() {
        return new DownSpecimenArm();
    }

    public class UpSampleArm implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armLeft.setPosition(armLeft2Sample);
            armRight.setPosition(armRight2Sample);
            wrist.setPosition(wrist2Sample);
            return false;
        }
    }
    public Action upSampleArm() {
        return new UpSampleArm();
    }

    public class DownSampleArm implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armLeft.setPosition(armLeft1Sample);
            armRight.setPosition(armRight1Sample);
            wrist.setPosition(wrist1Sample);
            return false;
        }
    }
    public Action downSampleArm() {
        return new DownSampleArm();
    }

}
