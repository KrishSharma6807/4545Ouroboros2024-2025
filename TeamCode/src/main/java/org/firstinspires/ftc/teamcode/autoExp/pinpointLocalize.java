package org.firstinspires.ftc.teamcode.autoExp;

import static org.firstinspires.ftc.teamcode.PinpointDrive.PARAMS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
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
public class pinpointLocalize extends OpMode {

    private PinpointDrive pinpointDrive;
    private FtcDashboard dashboard;

    public static double currentX = 0;
    public static double currentY = 0;
    public static double currentH = 0;

    public Pose2d robotPose2 = new Pose2d(0, 0, 0);


    public void init() {
        pinpointDrive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));
        dashboard = FtcDashboard.getInstance();
    }

    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        pinpointDrive.updatePoseEstimate();
        robotPose2 = pinpointDrive.getPoseEstimate();
        currentY = pinpointDrive.getY();
        currentH = pinpointDrive.getHeading();
        currentX = pinpointDrive.getX();

        fieldOverlay.setStroke("#FF0000");
        fieldOverlay.strokeCircle(currentX, currentY, 9);
        fieldOverlay.strokeLine(currentX, currentY, currentX + Math.cos(currentH) * 9, currentY + Math.sin(currentH) * 9);

        packet.put("x", currentX);
        dashboard.sendTelemetryPacket(packet);
    }
}