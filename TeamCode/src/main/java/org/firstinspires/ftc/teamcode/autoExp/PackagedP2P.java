package org.firstinspires.ftc.teamcode.autoExp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Geom.Pose;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotControl.CustomPID;
@Config
public class PackagedP2P {
    private DcMotorEx br, bl, fr, fl;
    private PinpointDrive pinpointDrive;
    private FtcDashboard dashboard;
    private ElapsedTime timer, stable;

    private double targetX, targetY, targetH;
    public static double pX = 0.22, iX = 0, dX = 0, xF = 0;
    public static double pY = 0.22, iY = 0, dY = 0, yF = 0;
    public static double pH = 0.22, iH = 0, dH = 0, hF = 0.1;

    private CustomPID xController = new CustomPID(pX, iX, dX, xF);
    private CustomPID yController = new CustomPID(pY, iY, dY, yF);
    private CustomPID hController = new CustomPID(pH, iH, dH, hF);

    private final double MAX_TRANSLATIONAL_SPEED = 0.5;
    private final double MAX_ROTATIONAL_SPEED = 1;
    private final double lateralMultiplier = 1.0;
    private final double d = 1.0;

    public PackagedP2P(HardwareMap hardwareMap) {
        pinpointDrive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        dashboard = FtcDashboard.getInstance();

        br = hardwareMap.get(DcMotorEx.class, "br");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");

        timer = new ElapsedTime();
        stable = new ElapsedTime();
    }

    public void setTarget(double x, double y, double h) {
        this.targetX = x;
        this.targetY = y;
        this.targetH = h;
    }

    public void runToPosition(LinearOpMode opMode) {
        while (opMode.opModeIsActive() && !isAtTarget()) {
            pinpointDrive.updatePoseEstimate();
            xController.setPID(pX, iX, dX, xF);
            yController.setPID(pY, iY, dY, yF);
            hController.setPID(pH, iH, dH, hF);
            Pose2d robotPose = pinpointDrive.getPoseEstimate();
            Pose powers = getPower(robotPose);

            fl.setPower(powers.getX() + lateralMultiplier * powers.getY() + d * powers.heading);
            bl.setPower(powers.getX() - lateralMultiplier * powers.getY() + d * powers.heading);
            br.setPower(powers.getX() + lateralMultiplier * powers.getY() - d * powers.heading);
            fr.setPower(powers.getX() - lateralMultiplier * powers.getY() - d * powers.heading);

            sendTelemetry(robotPose);
        }
        stopMotors();
    }

    private void sendTelemetry(Pose2d robotPose) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", robotPose.position.x);
        packet.put("y", robotPose.position.y);
        packet.put("h", robotPose.heading.toDouble());

        double currentX = robotPose.position.x;
        double currentY = robotPose.position.y;
        double currentH = robotPose.heading.toDouble();
        double errorH = normalizeAngle(targetH - currentH);

        packet.put("errorH", errorH);
        packet.put("errorY", currentY - targetY);
        packet.put("errorX", currentX-targetX);

        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStroke("#FF0000");
        fieldOverlay.strokeCircle(currentX, currentY, 9);
        fieldOverlay.strokeLine(currentX, currentY, currentX + Math.cos(currentH) * 9, currentY + Math.sin(currentH) * 9);

        fieldOverlay.setStroke("#0000FF");
        fieldOverlay.strokeCircle(targetX, targetY, 9);
        fieldOverlay.strokeLine(targetX, targetY, targetX + Math.cos(targetH) * 9, targetY + Math.sin(targetH) * 9);


        dashboard.sendTelemetryPacket(packet);
    }

    private boolean isAtTarget() {
        Pose2d robotPose = pinpointDrive.getPoseEstimate();
        double currentX = robotPose.position.x;
        double currentY = robotPose.position.y;
        double currentH = robotPose.heading.toDouble();
        double errorH = normalizeAngle(targetH - currentH);

        return Math.abs(targetX - currentX) <= 0.3 &&
                Math.abs(targetY - currentY) <= 0.3 &&
                Math.abs(errorH) <= 0.2;
    }

    private void stopMotors() {
        fl.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fr.setPower(0);
    }

    private Pose getPower(Pose2d robotPose) {
        double currentX = robotPose.position.x;
        double currentY = robotPose.position.y;
        double currentHeading = robotPose.heading.toDouble();

        // Calculate X and Y powers without negation
        double xPower = xController.calculatePower(targetX, currentX);
        double yPower = -yController.calculatePower(targetY, currentY);

        // Compute normalized heading error
        double error = normalizeAngle(targetH - currentHeading);
        double virtualTargetH = currentHeading + error;

        double positionError = Math.hypot(targetX - currentX, targetY - currentY);
       // double headingWeight = Range.clip(positionError / 5.0, 0.2, 1.0); // Scale heading influence dynamically

        double hPower = hController.calculatePower(0, error) + Math.signum(hController.calculatePower(0, error)) * hF;
        //double hPower = -hController.calculatePower(error, 0) + Math.signum(-hController.calculatePower(error, 0)) * hF;

        // Check stopping condition with error
        if (Math.abs(targetX - currentX) <= .3 &&
                Math.abs(targetY - currentY) <= .3 &&
                Math.abs(error) <= 0.2) {
            return new Pose(0, 0, 0);
        }

        double x_rotated = xPower * Math.cos(-currentHeading) - yPower * Math.sin(-currentHeading);
        double y_rotated = xPower * Math.sin(-currentHeading) + yPower * Math.cos(-currentHeading);

        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);
        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);
        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);

        return new Pose(x_rotated, y_rotated, hPower);
    }

    private double normalizeAngle(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle > Math.PI) {
            angle -= 2 * Math.PI;
        } else if (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}
