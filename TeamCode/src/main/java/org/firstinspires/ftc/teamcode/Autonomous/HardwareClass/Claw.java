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
    private ElapsedTime timer = new ElapsedTime();

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

    public class OpenClawSample implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(openClaw-.2);
            return false;
        }
    }
    public Action openClawSample() {
        return new OpenClawSample();
    }

    public class OpenClawWait implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            timer.reset();
            while(timer.seconds() < 1.5){

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
            while(timer.seconds() < 1.2){
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

    public class CloseClawWait implements Action {
        private boolean isFinished = false;
        private Thread closeClawThread;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (closeClawThread == null) { // Start the thread only once
                closeClawThread = new Thread(() -> {
                    ElapsedTime timer = new ElapsedTime();
                    timer.reset();

                    while (timer.seconds() < 0.5) {
                        // Do nothing, just wait
                    }

                    claw.setPosition(closeClaw);
                    isFinished = true;
                });
                closeClawThread.start();
            }

            // If thread has finished, return false to signal completion
            return !isFinished;
        }
    }


    public Action closeClawWait() {
        return new CloseClawWait();
    }
}
