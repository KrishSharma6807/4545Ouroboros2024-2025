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
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.Intake;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.OuttakeSlides;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.Arrays;


@Config
@Autonomous(name = "FivePlusO", group = "Autonomous")
public class FivePlusO extends LinearOpMode {
    public static double xPosBar = 17;
    public static double yPosBar = -33;
    public static double SplineHeading = 90;
    public static double baseAccelMin = -20;
    public static double baseAccelMax = 50;
    public static double baseTransVel = 40;
    public static double baseAngularVel = Math.PI;
    public static double xPosObserve = 40;
    public static double yPosOverserve = -64;
    public static double rightHeading = 90;
    public static double ySpike = -20;
    public static double xSpike1 = 52;
    public static double xSpike2 = 53;
    public static double xSpike3 = 70;

    public static double xSpike1First = 25;
    public static double yFinal = -30;

    public static double count = -5;



    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(33, -63, Math.toRadians(-90));
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

        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder toBar1 = drive.actionBuilder(initialPose)
                .waitSeconds(0.1)
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(xPosBar,yPosBar+1, Math.toRadians(-90)), Math.toRadians(SplineHeading));

        TrajectoryActionBuilder pushSpikes = toBar1.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(40, -30, Math.toRadians(-rightHeading)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(40, -14, Math.toRadians(-rightHeading)), Math.toRadians(0))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(xSpike1, ySpike, Math.toRadians(-90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xSpike1, yPosOverserve+14, Math.toRadians(-90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(xSpike2, ySpike, Math.toRadians(-90)), Math.toRadians(50))
                .splineToLinearHeading(new Pose2d(xSpike2, yPosOverserve+14, Math.toRadians(-90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(xSpike3 - 3, ySpike, Math.toRadians(-90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(xSpike3, ySpike, Math.toRadians(-90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xSpike3, yPosOverserve, Math.toRadians(-90)), Math.toRadians(-90) , new TranslationalVelConstraint(baseTransVel), baseAccelConstraint);
                //.lineToY(yPosOverserve, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint);

//        TrajectoryActionBuilder pushSpike1 = toSpike1.endTrajectory().fresh()
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(xSpike1, yPosOverserve+15, Math.toRadians(-90)), Math.toRadians(-90));

//        TrajectoryActionBuilder toSpike2 = pushSpike1.endTrajectory().fresh()
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(xSpike1 + 3, ySpike + 2, Math.toRadians(-90)), Math.toRadians(50));

//        TrajectoryActionBuilder pushSpike2 = toSpike2.endTrajectory().fresh()
//                .splineToLinearHeading(new Pose2d(xSpike2, yPosOverserve+15, Math.toRadians(-90)), Math.toRadians(-90));

//        TrajectoryActionBuilder toSpike3 = pushSpike2.endTrajectory().fresh()
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(xSpike3 - 3, ySpike + 2, Math.toRadians(-90)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(xSpike3, ySpike + 2, Math.toRadians(-90)), Math.toRadians(-90));

//        TrajectoryActionBuilder pushSpike3 =  toSpike3.endTrajectory().fresh()
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(xSpike3, yPosOverserve-2, Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder toBar2 = pushSpikes.endTrajectory().fresh()
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(xPosBar-1,yPosBar, Math.toRadians(-90)), Math.toRadians(SplineHeading));

        TrajectoryActionBuilder backBar2 = toBar2.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(xPosBar, yPosBar-2 , Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder toObserve2 = toBar2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xPosObserve, yPosOverserve+2, Math.toRadians(-90)), Math.toRadians(-70));

        TrajectoryActionBuilder toBar3 = toObserve2.endTrajectory().fresh()
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(xPosBar-2,yPosBar+.5, Math.toRadians(-90)), Math.toRadians(SplineHeading));

        TrajectoryActionBuilder backBar3 = toBar3.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(xPosBar, yPosBar-2 , Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder toObserve3 =  toBar3.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xPosObserve, yPosOverserve , Math.toRadians(-90)), Math.toRadians(-70));

        TrajectoryActionBuilder toBar4 = toObserve3.endTrajectory().fresh()
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(xPosBar-3,yPosBar, Math.toRadians(-90)), Math.toRadians(SplineHeading));

        TrajectoryActionBuilder backBar4 = toBar4.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(xPosBar, yPosBar-2 , Math.toRadians(-90)), Math.toRadians(-90));

        TrajectoryActionBuilder toObserve4 = toBar4.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xPosObserve, yPosOverserve , Math.toRadians(-90)), Math.toRadians(-70));

        TrajectoryActionBuilder toBar5 = toObserve4.endTrajectory().fresh()
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(xPosBar-4,yPosBar, Math.toRadians(-90)), Math.toRadians(SplineHeading));

        TrajectoryActionBuilder backBar5 = toBar5.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(xPosBar, yPosBar-1 , Math.toRadians(-90)), Math.toRadians(-90));

        //waits
        TrajectoryActionBuilder waitShort = toBar3.endTrajectory().fresh()
                .waitSeconds(.1);
        TrajectoryActionBuilder wait = toBar3.endTrajectory().fresh()
                .waitSeconds(.75);
        TrajectoryActionBuilder waitLong = toBar3.endTrajectory().fresh()
                .waitSeconds(1.5);

        waitForStart();
        if (isStopRequested()) return;

        // Cycle 1
        Actions.runBlocking(
                new ParallelAction(
                        claw.closeClaw(),
                        arm.upSpecimenArm(),
                        slides.liftDownFull()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        hSlides.returnIntakeSlides(),
                        slides.PIDGrab(),
                        toBar1.build()
                )
        );


        // Gathering samples
        Actions.runBlocking(
                new ParallelAction(
                        claw.openClaw(),
                        arm.downSpecimenArm(),
                        slides.liftDownFull(),
                        pushSpikes.build()
                )
        );

        // Cycle 2
        Actions.runBlocking(
                new ParallelAction(
                        claw.closeClaw(),
                        arm.upSpecimenArm(),
                        slides.PIDGrab(),
                        toBar2.build()
                )
        );

        // Cycle 3
        Actions.runBlocking(
                new ParallelAction(
                        claw.openClaw(),
                        arm.downSpecimenArm(),
                        toObserve2.build(),
                        slides.liftDownFull()
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
                        toObserve3.build(),
                        slides.liftDownFull()
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
        Actions.runBlocking(
                new ParallelAction(
                        claw.closeClaw(),
                        arm.upSpecimenArm(),
                        slides.PIDGrab(),
                        toBar5.build()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        claw.openClaw(),
                        backBar5.build()
                )
        );
    }
}