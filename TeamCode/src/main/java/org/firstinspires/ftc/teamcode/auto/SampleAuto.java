package org.firstinspires.ftc.teamcode.auto;
// RR-specific imports
import com.acmerobotics.dashboard.config.Config;

// Non-RR imports
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.Arm;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.Claw;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.IntakeAuto;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.OuttakeSlides;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.Arrays;


@Config
@Autonomous(name = "SampleAuto", group = "Autonomous")
public class SampleAuto extends LinearOpMode {
    public static double baseAccelMin = -10;
    public static double baseAccelMax = 25;
    public static double baseTransVel = 40;
    public static double baseAngularVel = Math.PI;

    public static double xPosBar = 2;
    public static double yPosBar = 35;
    public static double yFinal = 30;

    public static double bucketX = 56;
    public static double bucketY = 56;
    public static double bucketHeading = -135;

    public static double spikeX1 = 47;
    public static double spikeY1 = 38;

    public static double spikeX2 = 57;
    public static double spikeY2 = 38;

    public static double spikeX3 = 56;
    public static double spikeY3 = 25;

    public static double parkX = 22;
    public static double parkY = 10;

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(14, 63, Math.toRadians(-90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        OuttakeSlides slides = new OuttakeSlides(hardwareMap);
        IntakeAuto intake = new IntakeAuto(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(baseTransVel),
                new AngularVelConstraint(baseAngularVel)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(baseAccelMin, baseAccelMax);

        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder toBar = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(xPosBar,yPosBar, Math.toRadians(-90)), Math.toRadians(-90))
                .lineToY(yFinal);
        TrajectoryActionBuilder toSpike1 = toBar.endTrajectory().fresh()
                .lineToY(yFinal+1)
                .splineToLinearHeading(new Pose2d(spikeX1, spikeY1, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1);
        TrajectoryActionBuilder toBucketSpike1 = toSpike1.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(bucketX,bucketY, Math.toRadians(bucketHeading)), Math.toRadians(100))
                .waitSeconds(1);
        TrajectoryActionBuilder toSpike2 = toBucketSpike1.endTrajectory().fresh()
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(spikeX2, spikeY2, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1);
        TrajectoryActionBuilder toBucketSpike2 = toSpike2.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(bucketX,bucketY, Math.toRadians(bucketHeading)), Math.toRadians(100))
                .waitSeconds(1);
        TrajectoryActionBuilder toSpike3 = toBucketSpike2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(spikeX3, spikeY3, Math.toRadians(0)), Math.toRadians(-90))
                .waitSeconds(1);
        TrajectoryActionBuilder toBucketSpike3 = toSpike3.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(bucketX,bucketY, Math.toRadians(bucketHeading)), Math.toRadians(100))
                .waitSeconds(1);
        TrajectoryActionBuilder park = toBucketSpike3.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(parkX,parkY, Math.toRadians(0)), Math.toRadians(180))
                .waitSeconds(1);

        waitForStart();
        if (isStopRequested()) return;
//Cycle 1
        Actions.runBlocking(
                new ParallelAction(
                        claw.closeClaw(),
                        arm.upSpecimenArm(),
                        toBar.build(),
                        intake.intakeTiltUp()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        slides.liftUp()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        toSpike1.build(),
                        intake.intakeTiltDown(),
                        intake.intakeIn(),
                        slides.liftDownFull(),
                        arm.downSpecimenArm()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        toBucketSpike1.build(),
                        toSpike2.build(),
                        toBucketSpike2.build(),
                        toSpike3.build(),
                        toBucketSpike3.build(),
                        park.build()
                )
        );

    }
}