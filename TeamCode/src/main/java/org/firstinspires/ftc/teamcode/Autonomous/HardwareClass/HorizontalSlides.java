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

public class HorizontalSlides {
    public DcMotorEx intakeSlides;

    public HorizontalSlides(HardwareMap hardwareMap) {

        intakeSlides = hardwareMap.get(DcMotorEx.class, "intakeSlides");

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

    public class returnIntakeSlidesInfinite implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeSlides.setPower(-.5);
            return true;
        }
    }

    public Action returnIntakeSlidesInfinite() { return new returnIntakeSlides(); }

    public class extendIntakeSlidesInfinite implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeSlides.setPower(.5);
            return true;
        }
    }

    public Action extendIntakeSlidesInfinite() {
        return new extendIntakeSlidesInfinite();
    }
}

