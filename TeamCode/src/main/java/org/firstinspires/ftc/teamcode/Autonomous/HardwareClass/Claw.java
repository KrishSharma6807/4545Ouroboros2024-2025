package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {
    public static double clawSpin1 = 0;
    public static double clawSpin2 = .75;

    public static double closeClaw = 0;
    public static double openClaw = 1;
    public Servo claw;
    public Servo clawSpin;
    ElapsedTime timer = new ElapsedTime();

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
    }

    public class OpenClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(openClaw);
                return false;
        }
    }
    public Action openClaw() {
        return new OpenClaw();
    }

    public class OpenClawWait implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            timer.reset();
            while(timer.seconds() < 2){

            }
            claw.setPosition(openClaw);
            return false;
        }
    }
    public Action openClawWait() {
        return new OpenClawWait();
    }

    public class OpenClawWaitShort implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            timer.reset();
            while(timer.seconds() < .1){
            }
            claw.setPosition(openClaw);
            return false;
        }
    }
    public Action openClawWaitShort() {
        return new OpenClawWaitShort();
    }

    public class CloseClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(closeClaw);
                return false;
        }
    }


    public Action closeClaw() {
        return new CloseClaw();
    }
}
