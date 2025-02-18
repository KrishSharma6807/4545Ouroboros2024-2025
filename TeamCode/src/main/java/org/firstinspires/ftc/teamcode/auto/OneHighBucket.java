package org.firstinspires.ftc.teamcode.auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
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
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.IntakeAuto;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.OuttakeSlides;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.Arrays;

@Config
@Autonomous(name = "OneHighBucket", group = "Autonomous")
public class OneHighBucket extends LinearOpMode {

    public static double baseAccelMin = -10;
    public static double baseAccelMax = 25;
    public static double baseTransVel = 25;
    public static double baseAngularVel = Math.PI;
    public static double bucketX = -56;
    public static double bucketY = -56;
    public static double bucketHeading = 45;

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(-25, -63, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        OuttakeSlides slides = new OuttakeSlides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        IntakeAuto intake = new IntakeAuto(hardwareMap);
        HorizontalSlides hSlides = new HorizontalSlides(hardwareMap);
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(baseTransVel),
                new AngularVelConstraint(baseAngularVel)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(baseAccelMin, baseAccelMax);

        TrajectoryActionBuilder toBucket = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(bucketX, bucketY, Math.toRadians(bucketHeading)), Math.toRadians(-135));
        TrajectoryActionBuilder park = toBucket.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-25, -15, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(.1)
                .lineToX(-20);
        TrajectoryActionBuilder waitShort = park.endTrajectory().fresh()
                .waitSeconds(.25);
        TrajectoryActionBuilder wait = park.endTrajectory().fresh()
                .waitSeconds(.75);
        TrajectoryActionBuilder waitLong = park.endTrajectory().fresh()
                .waitSeconds(1.5);

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw(),
                        toBucket.build(),
                        slides.liftUpPID(),
                        arm.upSampleArm(),
                        waitLong.build(),
                        claw.openClaw(),
                        wait.build(),
                        arm.downSampleArm(),
                        slides.liftDownFull(),
                        hSlides.returnIntakeSlides(),
                        slides.liftUpPartial(),
                        park.build()
                )
        );
    }
}
