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
        IntakeAuto intake = new IntakeAuto(hardwareMap);
        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
                .waitSeconds(10);
        waitForStart();
        if (isStopRequested()) return;
//Cycle 1
        Actions.runBlocking(
                new SequentialAction(
                        slides.PIDGrab(),
                        wait.build(),
                        intake.intakeTiltUp()
                )
        );
    }
}