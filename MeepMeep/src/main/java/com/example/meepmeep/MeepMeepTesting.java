package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, 0, 0))
                .lineToX(-50)
                .turn(Math.toRadians(90))
                .lineToY(-50)
                .splineToLinearHeading(new Pose2d(0, -65, Math.toRadians(180)),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50, -50), Math.toRadians(0))
               // .lineToYConstantHeading(new Vector2d(50, 50), Math.toRadians(0)) #unkown error
                .splineToConstantHeading(new Vector2d(0, 50), Math.toRadians(0))
                //.splineToSplineHeading(new Pose2d(-50,50, Math.toRadians(0)))   #unkown error
                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}