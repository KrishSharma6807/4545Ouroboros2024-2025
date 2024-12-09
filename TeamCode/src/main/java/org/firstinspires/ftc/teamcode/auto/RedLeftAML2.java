package org.firstinspires.ftc.teamcode.auto;
// RR-specific imports
import com.acmerobotics.dashboard.config.Config;

// Non-RR imports
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;


@Config
@Autonomous(name = "RedLeftAML2", group = "Autonomous")
public class RedLeftAML2 extends LinearOpMode {
    public static double xPosBucket = 54;
    public static double yPosBucket = 51;
    public static double xSpike1 = 43;
    public static double xSpike2 = 54;
    public static double xSpike3 = 54;
    public static double ySpike1 = 38;
    public static double ySpike2 = ySpike1;
    public static double ySpike3 = 25;
    public static double bucketHeading = -135;

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(37, 60, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(40),
                new AngularVelConstraint(Math.PI)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 25.0);

        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder toBucket = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xPosBucket,yPosBucket, Math.toRadians(bucketHeading)), Math.toRadians(0))
                .waitSeconds(1);
        TrajectoryActionBuilder spike1 = toBucket.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xSpike1, ySpike1, Math.toRadians(-90)), Math.toRadians(190))
                .waitSeconds(1);
        TrajectoryActionBuilder toBucket2 = spike1.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(xPosBucket,yPosBucket, Math.toRadians(bucketHeading)), Math.toRadians(100))
                .waitSeconds(1);
        TrajectoryActionBuilder spike2 = toBucket2.endTrajectory().fresh()
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(xSpike2, ySpike2, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1);
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(54,51, Math.toRadians(-135)), Math.toRadians(100))
        TrajectoryActionBuilder toBucket3 = spike2.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(xPosBucket,yPosBucket, Math.toRadians(-135)), Math.toRadians(100))
                .waitSeconds(1);
        TrajectoryActionBuilder spike3 = toBucket2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xSpike3, ySpike3, Math.toRadians(0)), Math.toRadians(-90))
                .waitSeconds(1);
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(54,51, Math.toRadians(-135)), Math.toRadians(100))
        TrajectoryActionBuilder toBucket4 = spike3.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(xPosBucket, yPosBucket, Math.toRadians(-135)), Math.toRadians(100))
                .waitSeconds(1);
        TrajectoryActionBuilder park = toBucket2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(22,4.5, Math.toRadians(0)), Math.toRadians(180))
                .waitSeconds(1);



        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        toBucket.build(),
                        spike1.build(),
                        toBucket2.build(),
                        spike2.build(),
                        toBucket3.build(),
                        spike3.build(),
                        toBucket4.build(),
                        park.build()
                )
        );
    }
}