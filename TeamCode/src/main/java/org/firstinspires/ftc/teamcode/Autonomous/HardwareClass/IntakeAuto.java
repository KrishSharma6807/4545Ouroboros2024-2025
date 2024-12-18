package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeAuto {
    private DcMotorEx intake;
    private DcMotorEx horizSlidesLeft;
    private DcMotorEx horizSlidesRight;

    public IntakeAuto(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizSlidesLeft = hardwareMap.get(DcMotorEx.class, "horizSlidesLeft");
        horizSlidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizSlidesRight = hardwareMap.get(DcMotorEx.class, "horizSlidesRight");
        horizSlidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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


    public class horizSlidesOut implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                horizSlidesLeft.setPower(0);
                horizSlidesRight.setPower(0);
                initialized = true;
            }

            double vel = horizSlidesLeft.getVelocity();
            packet.put("horizSlidesVel", vel);
            horizSlidesLeft.setPower(1);
            horizSlidesRight.setPower(1);
            return true;
        }
    }
    public Action horizSlidesOut() {
        return new IntakeIn();
    }

    public class horizSlidesIn implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                horizSlidesRight.setPower(0);
                horizSlidesLeft.setPower(0);
                initialized = true;
            }

            double vel = horizSlidesRight.getVelocity();
            packet.put("horizSlidesVel", vel);
            horizSlidesRight.setPower(-1);
            horizSlidesLeft.setPower(-1);
            return true;
        }
    }

    public Action horizSlidesIn() {
        return new IntakeIn();
    }

}