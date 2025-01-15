package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robotControl.CustomPID;

public class OuttakeSlides {

    private DcMotor outtakeSlidesLeft;
    private DcMotor outtakeSlidesRight;
    private int targetPosition;
    private CustomPID pidController;

    // Constants for slide positions
    public static final int POSITION_HIGH = 1000; // Replace with actual values
    public static final int POSITION_MID = 500;
    public static final int POSITION_LOW = 0;

    // Constructor
    public OuttakeSlides(DcMotor leftMotor, DcMotor rightMotor, double kP, double kI, double kD, double kF) {
        this.outtakeSlidesLeft = leftMotor;
        this.outtakeSlidesRight = rightMotor;

        // Reset encoders and set mode
        this.outtakeSlidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.outtakeSlidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.outtakeSlidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.outtakeSlidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.outtakeSlidesLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        this.outtakeSlidesRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize the PID controller
        this.pidController = new CustomPID(kP, kI, kD, kF);
    }

    // Set target position
    public void setTargetPosition(int position) {
        this.targetPosition = position;
    }

    // Run slides to position using PID control
    public void runToPosition() {
        int currentPositionLeft = outtakeSlidesLeft.getCurrentPosition();
        int currentPositionRight = outtakeSlidesRight.getCurrentPosition();

        double correctionLeft = pidController.calculatePower(targetPosition, currentPositionLeft);
        double correctionRight = pidController.calculatePower(targetPosition, currentPositionRight);

        outtakeSlidesLeft.setPower(correctionLeft);
        outtakeSlidesRight.setPower(correctionRight);
    }

    // Hold the current position
    public void holdPosition() {
        targetPosition = outtakeSlidesLeft.getCurrentPosition(); // Assume both motors are synchronized
        runToPosition();
    }

    // Stop motors
    public void stop() {
        outtakeSlidesLeft.setPower(0);
        outtakeSlidesRight.setPower(0);
    }

    // Check if the slides are at the target position (within a tolerance)
    public boolean isAtTarget() {
        int currentPositionLeft = outtakeSlidesLeft.getCurrentPosition();
        int currentPositionRight = outtakeSlidesRight.getCurrentPosition();

        return Math.abs(targetPosition - currentPositionLeft) < 10
                && Math.abs(targetPosition - currentPositionRight) < 10; // 10 is tolerance, adjust as needed
    }
}