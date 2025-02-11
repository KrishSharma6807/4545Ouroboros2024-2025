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

import java.util.Arrays;


@Config
@Autonomous(name = "ThreeSpecimenTwoHumanPlayer", group = "Autonomous")
public class

ThreeSpecimenTwoHumanPlayer extends LinearOpMode {
    public static double xPosBar = 5;
    public static double yPosBar = -40;
    public static double SplineHeading = 90;
    public static double baseAccelMin = -10;
    public static double baseAccelMax = 25;
    public static double baseTransVel = 25;
    public static double baseAngularVel = Math.PI;
    public static double xPosObserve = 42;
    public static double yPosOverserve = -50;
    public static double rightHeading = 90;
    public static double xSpike1 = 50;

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
        HorizontalSlides hSlides = new HorizontalSlides(hardwareMap);
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(baseTransVel),
                new AngularVelConstraint(baseAngularVel)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(baseAccelMin, baseAccelMax);

        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder toBar = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(xPosBar,yPosBar, Math.toRadians(90)), Math.toRadians(SplineHeading))
                .lineToY(yFinal, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint);
        TrajectoryActionBuilder backSpike1 = toBar.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .lineToY(yFinal-8);
        TrajectoryActionBuilder toSpike1 = backSpike1.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(38, -37, Math.toRadians(rightHeading)), Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(40, -14, Math.toRadians(rightHeading)), Math.toRadians(0))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(xSpike1 +3, -14, Math.toRadians(rightHeading)), Math.toRadians(0))
                .waitSeconds(.1);

        TrajectoryActionBuilder toObserve1 = toSpike1.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .lineToY(yPosOverserve+7.5);

        TrajectoryActionBuilder backObserve = toObserve1.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .lineToY(yPosOverserve-12, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
                .waitSeconds(.1);

        TrajectoryActionBuilder toBar2 = backObserve.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(xSpike1, yPosOverserve-7, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(xPosBar + 3,yPosBar, Math.toRadians(90)), Math.toRadians(SplineHeading))
                .lineToY(yFinal, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint);

        TrajectoryActionBuilder toObserve2 = toBar2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xPosObserve,yPosOverserve + 6, Math.toRadians(rightHeading)),Math.toRadians(-90))
                .lineToY(yPosOverserve);

        TrajectoryActionBuilder backObserve2 = toObserve2.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .lineToY(yPosOverserve-13, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
                .waitSeconds(.1);

        //        TrajectoryActionBuilder forwardObserve2 = backObserve2.endTrajectory().fresh()
//                .setTangent(Math.toRadians(-90))
//                .lineToY(yPosOverserve);

        TrajectoryActionBuilder toBar3 = backObserve2.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(xPosObserve, yPosOverserve - 10, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(xPosBar - 5 ,yPosBar, Math.toRadians(90)), Math.toRadians(SplineHeading))
                .lineToY(yFinal, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint);

        TrajectoryActionBuilder toObserve3 = toBar3.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(xPosObserve,yPosOverserve + 5, Math.toRadians(rightHeading)),Math.toRadians(-90))
                .lineToY(yPosOverserve-10);

        TrajectoryActionBuilder wait = toBar3.endTrajectory().fresh()
                        .waitSeconds(.85);

        waitForStart();
        if (isStopRequested()) return;
//Cycle 1
        Actions.runBlocking(
                new ParallelAction(
                        intake.intakeTiltUp(),
                        claw.closeClaw(),
                        arm.upSpecimenArm(),
                        toBar.build(),
                        hSlides.returnIntakeSlides()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        arm.upSpecimenArmFull(),
                        wait.build(),
                        backSpike1.build(),
                        claw.openClawWaitShort(),
                        wait.build()
                )
        );


//        Cycle 2

        Actions.runBlocking(
                new ParallelAction(
                        hSlides.returnIntakeSlides(),
                        wait.build()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        toSpike1.build(),
                        claw.closeClaw(),
                        arm.downSpecimenArm(),
                        slides.PIDGrab(),
                        wait.build(),
                        wait.build(),
                        wait.build(),
                        claw.openClaw(),
                        backObserve.build(),
                        hSlides.returnIntakeSlides(),
                        arm.downSpecimenArm(),
                        wait.build(),
                        claw.closeClaw(),
                        wait.build(),
                        wait.build(),
                        slides.liftUpPartialPID(),
                        wait.build(),
                        arm.upSpecimenArm(),
                        slides.PIDDownFull()
                        //forwardObserve.build()

                )
        );//
        Actions.runBlocking(
                new ParallelAction(
                        hSlides.returnIntakeSlides(),
                        slides.PIDDownFull(),
                        toBar2.build()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        arm.upSpecimenArmFull(),
                        hSlides.returnIntakeSlides(),
                        claw.closeClaw(),
                        arm.downSpecimenArm(),
                        slides.PIDGrab(),
                        claw.closeClaw()

                        //Cycle 3
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        //Cycle 3
                        claw.openClawWait(),
                        toObserve2.build(),
                        hSlides.returnIntakeSlides(),
                        claw.openClaw(),
                       slides.PIDGrab()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        hSlides.returnIntakeSlides(),
                        backObserve2.build(),
                        claw.closeClaw(),
                        wait.build(),
                        slides.liftUpPartialPID()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        hSlides.returnIntakeSlides(),
                        claw.closeClaw(),
                        arm.upSpecimenArm(),
                        slides.PIDDownFull(),
                        toBar3.build()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        slides.liftUp(),
//                        claw.openClaw(),
//                        claw.closeClaw(),
                        slides.PIDDownFull(),
                        //Park
                        hSlides.returnIntakeSlides(),
                        toObserve3.build(),
                        arm.downSpecimenArm()
                )
        );
    }
}