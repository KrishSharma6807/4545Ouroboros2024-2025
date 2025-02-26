package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.Intake;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.OuttakeSlides;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.Arrays;

@Config
@Autonomous(name = "FourHighBucket", group = "Autonomous")
public class FourHighBucket extends LinearOpMode {

    public static double baseAccelMin = -10;
    public static double baseAccelMax = 20;
    public static double baseTransVel = 45;
    public static double baseAngularVel = Math.PI;
    public static double bucketX = -56;
    public static double bucketY = -56;
    public static double bucketHeading = 45;

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(25, 63, Math.toRadians(-90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        OuttakeSlides slides = new OuttakeSlides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        HorizontalSlides hSlides = new HorizontalSlides(hardwareMap);
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(baseTransVel),
                new AngularVelConstraint(baseAngularVel)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(baseAccelMin, baseAccelMax);

        TrajectoryActionBuilder toBucket = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(56,56, Math.toRadians(-135)), Math.toRadians(0));
        TrajectoryActionBuilder firstSample = toBucket.endTrajectory().fresh()
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(47, 38, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(47, 20, Math.toRadians(-90)), Math.toRadians(-90));
        TrajectoryActionBuilder toBucket2 = firstSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(56,56, Math.toRadians(-135)), Math.toRadians(100));
        TrajectoryActionBuilder secondSample = toBucket2.endTrajectory().fresh()
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(56, 38, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(56, 20, Math.toRadians(-90)), Math.toRadians(-90));
        TrajectoryActionBuilder toBucket3 = secondSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(56,56, Math.toRadians(-135)), Math.toRadians(100));
        TrajectoryActionBuilder thirdSample = toBucket3.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(57, 25, Math.toRadians(0)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(64, 25, Math.toRadians(0)), Math.toRadians(-90));
        TrajectoryActionBuilder toBucket4 = thirdSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(56,56, Math.toRadians(-135)), Math.toRadians(100));
        TrajectoryActionBuilder park = toBucket4.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(22,0, Math.toRadians(0)), Math.toRadians(180));
        TrajectoryActionBuilder waitShort = park.endTrajectory().fresh()
                .waitSeconds(5);
        TrajectoryActionBuilder wait = park.endTrajectory().fresh()
                .waitSeconds(.75);
        TrajectoryActionBuilder wait2 = park.endTrajectory().fresh()
                .waitSeconds(.75);
        TrajectoryActionBuilder wait3 = park.endTrajectory().fresh()
                .waitSeconds(.75);
        TrajectoryActionBuilder wait4 = park.endTrajectory().fresh()
                .waitSeconds(.75);
        TrajectoryActionBuilder waitClaw1 = park.endTrajectory().fresh()
                .waitSeconds(.75);
        TrajectoryActionBuilder waitClaw2 = park.endTrajectory().fresh()
                .waitSeconds(.75);
        TrajectoryActionBuilder waitClaw3 = park.endTrajectory().fresh()
                .waitSeconds(.75);
        TrajectoryActionBuilder waitClaw4 = park.endTrajectory().fresh()
                .waitSeconds(.75);
        TrajectoryActionBuilder waitLong = park.endTrajectory().fresh()
                .waitSeconds(.35);
        TrajectoryActionBuilder waitLong2 = park.endTrajectory().fresh()
                .waitSeconds(.35);
        TrajectoryActionBuilder waitLong3 = park.endTrajectory().fresh()
                .waitSeconds(.35);
        
        waitForStart();
        if (isStopRequested()) return;
//cycle 1
        Actions.runBlocking(
                new ParallelAction(
                        claw.closeClaw(),
                        toBucket.build(),
                        slides.liftUpPID(),
                        arm.upSampleArm(),
                        intake.intakeTiltDown()
                        )
        );

        Actions.runBlocking(
                new ParallelAction(
                        slides.liftUpPID(),
                        arm.upSampleArm()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        claw.openClawSample(),
                        wait.build(),
                        arm.downSampleArm(),
                        arm.downSampleArm(),
                        waitLong.build()
                )
        );
        //cycle 2

        Actions.runBlocking(
                new ParallelAction(
                        firstSample.build(),  // Road Runner trajectory
                        intake.intakeIn(),
                        hSlides.returnIntakeSlides(),
                        slides.liftDownFull(),
                        intake.intakeTiltDown()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        intake.intakeTiltUp(),
                        hSlides.returnIntakeSlides(),
                        claw.closeClaw(),
                        waitClaw1.build()
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        toBucket2.build(),
                        slides.liftUpPID(),
                        arm.upSampleArm(),
                        hSlides.returnIntakeSlides()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        claw.openClawSample(),
                        wait2.build()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        arm.downSampleArm(),
                        arm.downSampleArm(),
                        waitLong2.build()
                )
        );
        //Cycle 3
        Actions.runBlocking(
                new ParallelAction(
                        secondSample.build(),  // Road Runner trajectory
                        intake.intakeIn(),
                        hSlides.returnIntakeSlides(),
                        slides.liftDownFull(),
                        intake.intakeTiltDown()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        intake.intakeTiltUp(),
                        hSlides.returnIntakeSlides(),
                        claw.closeClaw(),
                        waitClaw2.build()
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        toBucket3.build(),
                        slides.liftUpPID(),
                        arm.upSampleArm(),
                        hSlides.returnIntakeSlides()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        claw.openClawSample(),
                        wait3.build()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        arm.downSampleArm(),
                        arm.downSampleArm(),
                        waitLong3.build()
                )
        );
        //Cycle 4
        Actions.runBlocking(
                new ParallelAction(
                        thirdSample.build(),  // Road Runner trajectory
                        intake.intakeIn(),
                        hSlides.returnIntakeSlides(),
                        slides.liftDownFull(),
                        intake.intakeTiltDown()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        intake.intakeTiltUp(),
                        hSlides.returnIntakeSlides(),
                        claw.closeClaw(),
                        waitClaw3.build()
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        toBucket4.build(),
                        slides.liftUpPID(),
                        arm.upSampleArm(),
                        hSlides.returnIntakeSlides()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        claw.openClawSample(),
                        wait4.build()
                )
        );
        //Park
        Actions.runBlocking(
                new ParallelAction(
                        park.build(),
                        slides.liftDownFull()
                )
        );
    }
}