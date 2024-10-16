package com.example.meepmeep;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                //Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(210), 15)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(30, -60, Math.toRadians(90)))
               // PATH ONE
               // Starts at X:-25
//                .splineToConstantHeading(new Vector2d(-52, -50), Math.toRadians(0))
//                .turn(Math.toRadians(-45))
//                .splineToLinearHeading(new Pose2d(-30, 0, Math.toRadians(0)), Math.toRadians(-180))
//                .splineToConstantHeading(new Vector2d(-50, -50),Math.toRadians(0))
//                .turn(Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(-30, 0, Math.toRadians(0)), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(-52, -50), Math.toRadians(0))
//                .turn(Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(-30, 0, Math.toRadians(0)), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(-50, -50),Math.toRadians(0))
//                .turn(Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(-30, 0, Math.toRadians(0)), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(-50, -51),Math.toRadians(-90))
//                .turn(Math.toRadians(180))
//                .lineToX(50)
                // PATH TWO
                // Starts at X:30+
                .setTangent((Math.PI))
                .splineToLinearHeading(new Pose2d(-55,-58, Math.toRadians(45)), Math.toRadians(-160))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-48, -38, Math.toRadians(90)), Math.toRadians(45))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-55, -58, Math.toRadians(45)), Math.toRadians(-60))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-25, 0, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-25, 0, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-25, 0, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(-50, -50, Math.toRadians(45)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-25, 0, Math.toRadians(0)), Math.toRadians(0))
                        .build());
//j



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}