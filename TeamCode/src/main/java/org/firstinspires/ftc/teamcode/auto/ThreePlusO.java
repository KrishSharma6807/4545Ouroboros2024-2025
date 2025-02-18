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
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.IntakeAuto;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.OuttakeSlides;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.R;

import java.util.Arrays;

import page.j5155.expressway.core.actions.RaceParallelAction;


@Config
@Autonomous(name = "ThreePlusO", group = "Autonomous")
public class

ThreePlusO extends LinearOpMode {
    public static double xPosBar = 5;
    public static double yPosBar = -35;
    public static double SplineHeading = 90;
    public static double baseAccelMin = -10;
    public static double baseAccelMax = 25;
    public static double baseTransVel = 40;
    public static double baseAngularVel = Math.PI;
    public static double xPosObserve = 38;
    public static double yPosOverserve = -30;
    public static double rightHeading = 90;
    public static double ySpike = -14;
    public static double xSpike1 = 47;
    public static double xSpike2 = 57;
    public static double xSpike3 = 65;

    public static double xSpike1First = 25;
    public static double yFinal = -30;

    public static double count = -5;



    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(10, -63, Math.toRadians(-90));
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

        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder toBar1 = drive.actionBuilder(initialPose)
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(xPosBar,yPosBar, Math.toRadians(-90)), Math.toRadians(SplineHeading));

        TrajectoryActionBuilder toSpike1 = toBar1.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(40, -30, Math.toRadians(-rightHeading)), Math.toRadians(55))
                .splineToLinearHeading(new Pose2d(40, -14, Math.toRadians(-rightHeading)), Math.toRadians(0))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(xSpike1, ySpike, Math.toRadians(-rightHeading)), Math.toRadians(0));

        TrajectoryActionBuilder pushSpike1 = toSpike1.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .lineToY(yPosOverserve);

        TrajectoryActionBuilder toSpike2 = pushSpike1.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(xSpike1 + 3, ySpike + 2, Math.toRadians(-90)), Math.toRadians(50));

        TrajectoryActionBuilder pushSpike2 = toSpike2.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(xSpike2, yPosOverserve, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder toSpike3 = pushSpike2.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(xSpike3 - 3, ySpike + 2, Math.toRadians(-90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xSpike3, ySpike + 2, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder pushSpike3 =  toSpike3.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xSpike3, yPosOverserve-34 , Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder toBar2 = pushSpike3.endTrajectory().fresh()
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(xPosBar - 2,yPosBar+10, Math.toRadians(-90)), Math.toRadians(SplineHeading));

        TrajectoryActionBuilder toObserve2 = toBar2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xPosObserve, yPosOverserve-34 , Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder toBar3 = toObserve2.endTrajectory().fresh()
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(xPosBar - 4,yPosBar+10, Math.toRadians(-90)), Math.toRadians(SplineHeading));

        TrajectoryActionBuilder toObserve3 =  toBar3.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xPosObserve, yPosOverserve-34 , Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder toBar4 = toObserve3.endTrajectory().fresh()
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(xPosBar - 6,yPosBar+10, Math.toRadians(-90)), Math.toRadians(SplineHeading));

        TrajectoryActionBuilder toObserve4 = toBar4.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xPosObserve, yPosOverserve-34 , Math.toRadians(-90)), Math.toRadians(-90));

        //waits
        TrajectoryActionBuilder waitShort = toBar3.endTrajectory().fresh()
                .waitSeconds(.25);
        TrajectoryActionBuilder wait = toBar3.endTrajectory().fresh()
                .waitSeconds(.75);
        TrajectoryActionBuilder waitLong = toBar3.endTrajectory().fresh()
                .waitSeconds(1.5);

        waitForStart();
        if (isStopRequested()) return;

        // Cycle 1
        Actions.runBlocking(
                new ParallelAction(
                        slides.PIDGrab(),
                        claw.closeClaw(),
                        arm.upSpecimenArm(),
                        toBar1.build(),
                        hSlides.returnIntakeSlides()
                )
        );


        // Gathering samples
        Actions.runBlocking(
                new ParallelAction(
                        claw.openClaw(),
                        arm.downSpecimenArm(),
                        slides.liftDownFullWait(),
                        toSpike1.build()
                )
        );
        Actions.runBlocking(
                new RaceParallelAction(
                        pushSpike1.build(),
                        hSlides.extendIntakeSlidesInfinite()
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        toSpike2.build(),
                        hSlides.returnIntakeSlides()
                )
        );
        Actions.runBlocking(
                new RaceParallelAction(
                        pushSpike2.build(),
                        hSlides.extendIntakeSlidesInfinite()
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        toSpike3.build(),
                        hSlides.returnIntakeSlides()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        pushSpike3.build(),
                        claw.closeClaw()
                )
        );

        // Cycle 2
        Actions.runBlocking(
                new ParallelAction(
                        arm.upSpecimenArm(),
                        slides.PIDGrab(),
                        toBar2.build()
                )
        );

        // Cycle 3
        Actions.runBlocking(
                new ParallelAction(
                        claw.openClaw(),
                        slides.liftDownFullWait(),
                        arm.downSpecimenArm(),
                        toObserve2.build()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        claw.closeClaw(),
                        arm.upSpecimenArm(),
                        slides.PIDGrab(),
                        toBar3.build()
                )
        );
        // Cycle 4
        Actions.runBlocking(
                new ParallelAction(
                        claw.openClaw(),
                        arm.downSpecimenArm(),
                        slides.liftDownFullWait(),
                        toObserve3.build()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        claw.closeClaw(),
                        arm.upSpecimenArm(),
                        slides.PIDGrab(),
                        toBar4.build()
                )
        );

        // Cycle 5 and park
        Actions.runBlocking(
                new ParallelAction(
                        claw.openClaw(),
                        arm.downSpecimenArm(),
                        slides.liftDownFull(),
                        toObserve4.build()
                )
        );
    }
}