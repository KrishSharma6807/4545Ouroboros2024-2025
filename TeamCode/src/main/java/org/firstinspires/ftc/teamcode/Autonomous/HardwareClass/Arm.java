package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Arm {
    public Servo armLeft;
    public Servo armRight;
    public Servo wrist;
    public static double armRight1Specimen = 1;
    public static double armRight2Specimen = .65;//.2 is max, 1 is min (tune min more)
    public static double armRight3Specimen = .45;

    public static double armLeft1Specimen = .25;
    public static double armLeft2Specimen = .7;
    public static double armLeft3Specimen = .9;

    public static double wrist1Specimen = .31;
    public static double wrist2Specimen = .64;
    public static double wrist3Specimen = .64;

    public Arm(HardwareMap hardwareMap) {
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");
        wrist = hardwareMap.get(Servo.class, "wrist");

        armRight.setDirection(Servo.Direction.REVERSE);
        armLeft.setDirection(Servo.Direction.REVERSE);
    }

    public class UpSpecimenArm implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
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
            armLeft.setPosition(armLeft3Specimen);
            armRight.setPosition(armRight3Specimen);
            wrist.setPosition(wrist3Specimen);
            return false;
        }
    }
    public Action upSpecimenArmFull() {
        return new UpSpecimenArmFull();
    }

    public class DownSpecimenArm implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armLeft.setPosition(armLeft1Specimen);
            armRight.setPosition(armRight1Specimen);
            wrist.setPosition(wrist1Specimen);
            return false;
        }
    }
    public Action downSpecimenArm() {
        return new DownSpecimenArm();
    }

}
