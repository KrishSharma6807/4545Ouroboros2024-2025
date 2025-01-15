package org.firstinspires.ftc.teamcode.autoExp;

import static org.firstinspires.ftc.teamcode.PinpointDrive.PARAMS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Geom.Pose;
import org.firstinspires.ftc.teamcode.robotControl.CustomPID;

@Config
@Autonomous
public class PIDToPoint extends OpMode {
    public DcMotorEx br;
    public DcMotorEx bl;
    public DcMotorEx fr;
    public DcMotorEx fl;
    GoBildaPinpointDriverRR localizer;
    private FtcDashboard dashboard;

    public static double targetX= 0;
    public static double targetY= 0;
    public static double targetH= Math.PI;

    public static double pX = 0;
    public static double iX = 0;
    public static double dX = 0;
    public static double xF = 0;

    public static double pY = 0;
    public static double iY = 0;
    public static double dY = 0;
    public static double yF = 0;

    public static double pH = 0;
    public static double iH = 0;
    public static double dH = 0;
    public static double hF = 0;

    public Pose targetPose = new Pose(targetX, targetY, targetH);
    public Pose robotPose2 = new Pose(0, 0, 0);

    public static CustomPID xController = new CustomPID(pX, iX, dX, xF);
    public static CustomPID yController = new CustomPID(pY, iY, dY, yF);
    public static CustomPID hController = new CustomPID(pH, iH, dH, hF);

    private final double  MAX_TRANSLATIONAL_SPEED = 0.5;
    private final double  MAX_ROTATIONAL_SPEED = 0.4;
    private final double X_GAIN = 2.00;

    private ElapsedTime timer;
    private ElapsedTime stable;
    public static double lateralMultiplier = 1.0;

    public static double trackWidth = 1;
    public static double wheelBase = 1;
    double d = (trackWidth + wheelBase) / 2.0;

    public void init() {
        br = hardwareMap.get(DcMotorEx.class, "br");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);

        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        localizer = hardwareMap.get(GoBildaPinpointDriverRR.class, PARAMS.pinpointDeviceName);
        dashboard = FtcDashboard.getInstance();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setStroke("#0000FF");
        fieldOverlay.strokeCircle(targetPose.x, targetPose.y, 9); // 9 is the radius of the robot, adjust as needed
        fieldOverlay.strokeLine(targetPose.x, targetPose.y, targetPose.x + Math.cos(targetPose.heading) * 9, targetPose.y + Math.sin(targetPose.heading) * 9);
        dashboard.sendTelemetryPacket(packet);
    }
    public void loop() {
        // Update the PurePursuitCommand
        // Get the current position of the robot
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        robotPose2 = new Pose(localizer.getPosX(), localizer.getPosY(), localizer.getHeading());

        fieldOverlay.setStroke("#FF0000");
        fieldOverlay.strokeCircle(robotPose2.x, robotPose2.y, 9); // 9 is the radius of the robot, adjust as needed
        fieldOverlay.strokeLine(robotPose2.x, robotPose2.y, robotPose2.x + Math.cos(robotPose2.heading) * 9, robotPose2.y + Math.sin(robotPose2.heading) * 9);

        fieldOverlay.setStroke("#0000FF");
        fieldOverlay.strokeCircle(targetPose.x, targetPose.y, 9); // 9 is the radius of the robot, adjust as needed
        fieldOverlay.strokeLine(targetPose.x, targetPose.y, targetPose.x + Math.cos(targetPose.heading) * 9, targetPose.y + Math.sin(targetPose.heading) * 9);

        fieldOverlay.strokeCircle(targetPose.x, targetPose.y,2);
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);

        execute();
    }

    public void execute() {
        if (timer == null) timer = new ElapsedTime();
        if (stable == null) stable = new ElapsedTime();

        Pose robotPose = new Pose(localizer.getPosX(), localizer.getPosY(), localizer.getHeading());

        Pose powers = getPower(robotPose);
        fl.setPower(powers.getX() - lateralMultiplier * powers.getY() - d * powers.heading);
        fr.setPower(powers.getX() + lateralMultiplier * powers.getY() + d * powers.heading);
        bl.setPower(powers.getX() + lateralMultiplier * powers.getY() - d * powers.heading);
        br.setPower(powers.getX() - lateralMultiplier * powers.getY() + d * powers.heading);
    }

    public Pose getPower(Pose robotPose) {
        double currentX = robotPose.x;
        double xPower = -xController.calculatePower(targetPose.x, currentX);
        if((Math.abs(targetPose.x - currentX) <= 0.08)){
            xPower=0;
        }

        double currentHeading = robotPose.heading;
        double error = AngleUnit.normalizeRadians(targetPose.heading - currentHeading);
        double hPower = hController.calculatePower(0, error);
        if((Math.abs(error) <= 0.017453)){
            hPower=0;
        }

        double currentY = robotPose.y;
        double yPower = -yController.calculatePower(targetPose.y, currentY);
        if((Math.abs(targetPose.y-robotPose.y) <= 0.09)){
            yPower=0;
        }

        double x_rotated = xPower * Math.cos(-robotPose.heading) - yPower * Math.sin(-robotPose.heading);
        double y_rotated = xPower * Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);

        // technically i dont think this is normalized correctly
        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);
        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);

        return new Pose(x_rotated, y_rotated, hPower);
    }
}