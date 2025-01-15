package com.example.meepmeep;

import com.acmerobotics.roadrunner.Math;
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
                .setConstraints(50, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -60, Math.toRadians(90)))
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

//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(54,51, Math.toRadians(-135)), Math.toRadians(0))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(43, 38, Math.toRadians(-90)), Math.toRadians(190))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(54,51, Math.toRadians(-135)), Math.toRadians(100))
//                .setTangent(Math.toRadians(290))
//                .splineToLinearHeading(new Pose2d(50, 38, Math.toRadians(-90)), Math.toRadians(-90))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(54,51, Math.toRadians(-135)), Math.toRadians(100))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(48, 25, Math.toRadians(0)), Math.toRadians(-90))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(54,51, Math.toRadians(-135)), Math.toRadians(100))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(22,0, Math.toRadians(0)), Math.toRadians(180))
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(2,34, Math.toRadians(270)), Math.toRadians(270))

                //PATH THREE
                .splineToLinearHeading(new Pose2d(0, -35, Math.toRadians(0)), Math.toRadians(180))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(20, -50, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -20, Math.toRadians(0)), Math.toRadians(0))
                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
//f