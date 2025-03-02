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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.Arm;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.Claw;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.IntakeAuto;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.OuttakeSlides;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.Arrays;


@Config
@Autonomous(name = "ThreeSpecimenOneHumanPlayer", group = "Autonomous")
public class ThreeSpecimenOneHumanPlayer extends LinearOpMode {
    public static double xPosBar = 5;
    public static double yPosBar = -35;
    public static double SplineHeading = 90;
    public static double baseAccelMin = -10;
    public static double baseAccelMax = 25;
    public static double baseTransVel = 40;
    public static double baseAngularVel = Math.PI;
    public static double xPosObserve = 38;
    public static double yPosOverserve = -60;
    public static double rightHeading = 90;
    public static double xSpike1 = 47;
    public static double xSpike2 = 62;

    public static double xSpike1First = 25;
    public static double yFinal = -30;

    public static double count = -5;



    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(10, -63, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        OuttakeSlides slides = new OuttakeSlides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        IntakeAuto intake = new IntakeAuto(hardwareMap);
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(baseTransVel),
                new AngularVelConstraint(baseAngularVel)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(baseAccelMin, baseAccelMax);

        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder toBar = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(xPosBar,yPosBar, Math.toRadians(90)), Math.toRadians(SplineHeading))
                .lineToY(yFinal, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint);
        TrajectoryActionBuilder toSpike1 = toBar.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(32, -37, Math.toRadians(rightHeading)), Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(40, -14, Math.toRadians(rightHeading)), Math.toRadians(0))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(xSpike1 + 6, -14, Math.toRadians(rightHeading)), Math.toRadians(-90))
                .waitSeconds(.1);
        TrajectoryActionBuilder toObserve1 = toSpike1.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .lineToY(yPosOverserve + 5);
        TrajectoryActionBuilder toSpike2 = toSpike1.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(xSpike1 + 6, -7, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(xSpike2, -7, Math.toRadians(90)), Math.toRadians(-90))
                .waitSeconds(.1);
        TrajectoryActionBuilder toObserve2 = toSpike2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .lineToY(yPosOverserve + 5)
                .setTangent(Math.toRadians(-90))
                .lineToY(yPosOverserve-4, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
                .waitSeconds(.1);
        TrajectoryActionBuilder toBar2 = toObserve2.endTrajectory().fresh()
                .waitSeconds(.2)
                .splineToLinearHeading(new Pose2d(xSpike2, yPosOverserve, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(xPosBar + 3,yPosBar, Math.toRadians(90)), Math.toRadians(SplineHeading))
                .lineToY(yFinal, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint);
        TrajectoryActionBuilder toObserve3 = toBar2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xPosObserve,yPosOverserve + 5, Math.toRadians(rightHeading)),Math.toRadians(-90))
                .lineToY(yPosOverserve)
                .setTangent(Math.toRadians(-90))
                .lineToY(yPosOverserve-2.2, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint);
        TrajectoryActionBuilder toBar3 = toObserve3.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(xPosBar - 7 ,yPosBar, Math.toRadians(90)), Math.toRadians(SplineHeading))
                .lineToY(yFinal, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint);
        TrajectoryActionBuilder park = toBar3.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xPosObserve,yPosOverserve + 5, Math.toRadians(rightHeading)),Math.toRadians(-90))
                .lineToY(yPosOverserve-6);

        TrajectoryActionBuilder wait = toBar3.endTrajectory().fresh()
                .waitSeconds(.6);

        waitForStart();
        if (isStopRequested()) return;
//Cycle 1
        Actions.runBlocking(
                new ParallelAction(
                        intake.intakeTiltUp(),
                        claw.closeClaw(),
                        arm.upSpecimenArm(),
                        toBar.build()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        slides.liftUp(),
                        claw.openClaw(),
                        claw.closeClaw()

                )
        );
        //Cycle 2

        Actions.runBlocking(
                new ParallelAction(
                        toSpike1.build(),
                        claw.closeClaw(),
                        arm.downSpecimenArm(),
                        slides.liftDown()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        claw.openClaw(),
                        toObserve1.build(),
                        toSpike2.build()
                        //forwardObserve.build()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        toObserve2.build(),
                        claw.openClaw()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw(),
                        wait.build(),
                        slides.liftUpPartial(),
                        arm.upSpecimenArm(),
                        slides.liftDownFull(),
                        toBar2.build(),
                        slides.liftUp(),
                        claw.openClaw(),
                        claw.closeClaw(),
                        arm.downSpecimenArm(),
                        slides.liftDown(),
                        claw.closeClaw()
                        //Cycle 3
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        //Cycle 3
                        toObserve3.build(),
                        claw.openClaw()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw(),
                        wait.build(),
                        slides.liftUpPartial()
                        //forwardObserve2.build()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        claw.closeClaw(),
                        arm.upSpecimenArm(),
                        toBar3.build()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        slides.liftDownFull(),
                        slides.liftUp(),
                        claw.openClaw(),
                        claw.closeClaw(),
                        slides.liftDownFull(),
                        //Park
                        park.build(),
                        arm.downSpecimenArm()
                )
        );
    }
}