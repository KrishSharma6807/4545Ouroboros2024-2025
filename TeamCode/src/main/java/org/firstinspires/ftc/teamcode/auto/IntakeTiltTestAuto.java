package org.firstinspires.ftc.teamcode.auto;
// RR-specific imports
import com.acmerobotics.dashboard.config.Config;

// Non-RR imports
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.Arm;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.Claw;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.Intake;
import org.firstinspires.ftc.teamcode.Autonomous.HardwareClass.OuttakeSlides;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.autoExp.PIDToPoint;
import org.firstinspires.ftc.teamcode.autoExp.PackagedP2P;


@Config
@Autonomous(name = "IntakeTiltTestAuto", group = "Autonomous")
//@Disabled
public class IntakeTiltTestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(10, -63, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        OuttakeSlides slides = new OuttakeSlides(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        PackagedP2P p2p = new PackagedP2P(hardwareMap);
        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
                .waitSeconds(10);
        TrajectoryActionBuilder wait2 = drive.actionBuilder(initialPose)
                .waitSeconds(10);
        TrajectoryActionBuilder wait3 = drive.actionBuilder(initialPose)
                .waitSeconds(10);
        waitForStart();
        if (isStopRequested()) return;
//Cycle 1
//        Actions.runBlocking(
//                new SequentialAction(
//                        arm.downSampleArm(),
//                        wait.build(),
//                        intake.intakeTiltUp(),
//                        wait2.build(),
//                        claw.closeClawWait(),
//                        wait3.build()
//                )
//        );
        p2p.setTarget(10,10,0);
        p2p.runToPosition(this);
    }
}