package org.firstinspires.ftc.teamcode.Autonomous.HardwareClass;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotControl.CustomPID;

public class OuttakeSlides {
    private DcMotorEx outtakeSlidesLeft;
    private DcMotorEx outtakeSlidesRight;

    public static double kP = 0.004;
    public static double kI = 0;
    public static double kD = 0.00011;
    CustomPID pidController = new CustomPID(kP, kI, kD, 0.0);

    public OuttakeSlides(HardwareMap hardwareMap) {
        outtakeSlidesLeft = hardwareMap.get(DcMotorEx.class, "outtakeSlidesLeft");
        outtakeSlidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSlidesRight = hardwareMap.get(DcMotorEx.class, "outtakeSlidesLeft");
        outtakeSlidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSlidesLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeSlidesRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class LiftUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                double power = pidController.calculatePower(3000, outtakeSlidesRight.getCurrentPosition());
                outtakeSlidesLeft.setPower(power);
                outtakeSlidesRight.setPower(power);
                initialized = true;
            }

            double pos = outtakeSlidesRight.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos < 3000.0) {
                return true;
            } else {
                outtakeSlidesLeft.setPower(0);
                outtakeSlidesRight.setPower(0);
                return false;
            }
        }
    }
    public Action liftUp() {
        return new LiftUp();
    }

    public class LiftDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                outtakeSlidesLeft.setPower(-0.8);
                outtakeSlidesRight.setPower(-0.8);
                initialized = true;
            }

            double pos = outtakeSlidesRight.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos > 100.0) {
                return true;
            } else {
                outtakeSlidesLeft.setPower(0);
                outtakeSlidesRight.setPower(0);
                return false;
            }
        }
    }
    public Action liftDown(){
        return new LiftDown();
    }
}

