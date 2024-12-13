package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotControl.CustomPID;

public class IntakeAuto {
    private DcMotorEx intake;

    public IntakeAuto(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public class IntakeIn implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setPower(0);
                initialized = true;
            }

            double vel = intake.getVelocity();
            packet.put("IntakeVel", vel);
            intake.setPower(1);
            return true;
        }
    }
    public Action intakeIn() {
        return new IntakeIn();
    }

    public class IntakeOut implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                intake.setPower(0);
                initialized = true;
            }

            double vel = intake.getVelocity();
            packet.put("IntakeVel", vel);
            intake.setPower(-1);
            return true;
        }
    }
    public Action intakeOut(){
        return new IntakeOut();
    }
}