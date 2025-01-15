package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public Servo armLeft;
    public Servo armRight;
    public Servo wrist;
    public static double armRight1Specimen = 1;
    public static double armRight2Specimen = .4;

    public static double armLeft1Specimen = .25;
    public static double armLeft2Specimen = .75;

    public static double wrist1Specimen = .31;
    public static double wrist2Specimen = .85;;

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
