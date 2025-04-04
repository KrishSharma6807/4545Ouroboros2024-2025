package org.firstinspires.ftc.teamcode.autoExp;

import static org.firstinspires.ftc.teamcode.PinpointDrive.PARAMS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Geom.Pose;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.robotControl.CustomPID;

@Config
@Autonomous
public class PIDToPoint extends OpMode {
    public DcMotorEx br;
    public DcMotorEx bl;
    public DcMotorEx fr;
    public DcMotorEx fl;

    private PinpointDrive pinpointDrive;
    private FtcDashboard dashboard;

    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetH = Math.PI;

    public static double currentX = 0;
    public static double currentY = 0;
    public static double currentH = 0;

    public static double pX = 0.22, iX = 0, dX = 0, xF = 0;
    public static double pY = 0.22, iY = 0, dY = 0, yF = 0;
    public static double pH = 0.22, iH = 0, dH = 0, hF = 0.1;

    public Pose2d targetPose = new Pose2d(targetX, targetY, targetH);
    public Pose2d robotPose2 = new Pose2d(0, 0, 0);

    public static CustomPID xController = new CustomPID(pX, iX, dX, xF);
    public static CustomPID yController = new CustomPID(pY, iY, dY, yF);
    public static CustomPID hController = new CustomPID(pH, iH, dH, hF);

    private final double MAX_TRANSLATIONAL_SPEED = .5;
    private final double MAX_ROTATIONAL_SPEED = 1;
    private final double lateralMultiplier = 1.0;
    private final double d = 1.0;

    public void init() {
        pinpointDrive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        dashboard = FtcDashboard.getInstance();
        br = hardwareMap.get(DcMotorEx.class, "br");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
    }

    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        // Update target pose with current values
        targetPose = new Pose2d(targetX, targetY, targetH);
        hController.setPID(pH,iH,dH,0);
        xController.setPID(pX,iX,dX,xF);
        yController.setPID(pY,iY,dY,yF);

        pinpointDrive.updatePoseEstimate();
        robotPose2 = pinpointDrive.getPoseEstimate();
        currentX = robotPose2.position.x;
        currentY = robotPose2.position.y;
        currentH = robotPose2.heading.toDouble();

        fieldOverlay.setStroke("#FF0000");
        fieldOverlay.strokeCircle(currentX, currentY, 9);
        fieldOverlay.strokeLine(currentX, currentY, currentX + Math.cos(currentH) * 9, currentY + Math.sin(currentH) * 9);

        fieldOverlay.setStroke("#0000FF");
        fieldOverlay.strokeCircle(targetX, targetY, 9);
        fieldOverlay.strokeLine(targetX, targetY, targetX + Math.cos(targetH) * 9, targetY + Math.sin(targetH) * 9);

        packet.put("x", currentX);
        packet.put("y", currentY);
        packet.put("h", currentH);
        Pose powers = getPower(robotPose2);
        packet.put("errorH", normalizeAngle(currentH - targetH));
        packet.put("errorY", currentY - targetY);
        packet.put("errorX", currentX-targetX);
        dashboard.sendTelemetryPacket(packet);

        execute();
    }

    public void execute() {
        Pose powers = getPower(robotPose2);
        //pinpointDrive.setDrivePowers();
        fl.setPower(powers.getX() + lateralMultiplier * powers.getY() + d * powers.heading);
        bl.setPower(powers.getX() - lateralMultiplier * powers.getY() + d * powers.heading);
        br.setPower(powers.getX() + lateralMultiplier * powers.getY() - d * powers.heading);
        fr.setPower(powers.getX() - lateralMultiplier * powers.getY() - d * powers.heading);
    }

    public Pose getPower(Pose2d robotPose) {
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
        double headingWeight = Range.clip(positionError / 5.0, 0.2, 1.0); // Scale heading influence dynamically

        double hPower = hController.calculatePower(0, error) * headingWeight + Math.signum(hController.calculatePower(0, error)) * hF;
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