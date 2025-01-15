package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotControl.CustomPID;

public class IntakeAuto {
    private DcMotorEx intake;
    public Servo intakeTiltLeft;
    public Servo intakeTiltRight;
    public IntakeAuto(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeTiltLeft = hardwareMap.get(Servo.class, "intakeTiltLeft");
        intakeTiltRight = hardwareMap.get(Servo.class, "intakeTiltRight");
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
            intakeTiltLeft.setPosition(.8);
            intakeTiltRight.setPosition(0.47);
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
            intakeTiltLeft.setPosition(.6);
            intakeTiltRight.setPosition(0.85);
            return false;
        }
    }
    public Action intakeTiltDown(){
        return new IntakeTiltDown();
    }
}