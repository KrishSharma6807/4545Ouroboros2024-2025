package com.example.meepmeep;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    //public static double xPosBar = 5;
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
//    public static double xPosBar = 2;
//    public static double yPosBar = 35;
    public static double yFinal = -30;

    public static double bucketX = -56;
    public static double bucketY = -56;
    public static double bucketHeading = 45;

    public static double spikeX1 = 47;
    public static double spikeY1 = 38;

    public static double spikeX2 = 57;
    public static double spikeY2 = 38;

    public static double spikeX3 = 56;
    public static double spikeY3 = 25;

    public static double parkX = 22;
    public static double parkY = 10;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(baseAccelMin, baseAccelMax);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                //Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180) * 3, Math.toRadians(180) * 3, 12)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(33, 63, Math.toRadians(-90)))
                // 4 SPECI NEW STYLE (5 WORKING (NEED +1))
//                .splineToLinearHeading(new Pose2d(xPosBar,yPosBar, Math.toRadians(-90)), Math.toRadians(SplineHeading))
//                //.lineToY(yFinal, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
//
//
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(40, -30, Math.toRadians(-rightHeading)), Math.toRadians(55))
//                .splineToLinearHeading(new Pose2d(40, -14, Math.toRadians(-rightHeading)), Math.toRadians(0))
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(xSpike1, ySpike, Math.toRadians(-rightHeading)), Math.toRadians(0))
//
//
//                .setTangent(Math.toRadians(-90))
//                .lineToY(yPosOverserve)
//
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(xSpike1 + 3, ySpike + 2, Math.toRadians(-90)), Math.toRadians(50))
//                .splineToLinearHeading(new Pose2d(xSpike2, yPosOverserve, Math.toRadians(-90)), Math.toRadians(-90))
//
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(xSpike3 - 3, ySpike + 2, Math.toRadians(-90)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(xSpike3, ySpike + 2, Math.toRadians(-90)), Math.toRadians(-90))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(xSpike3, yPosOverserve-34 , Math.toRadians(-90)), Math.toRadians(-90))
//
//                .setTangent(Math.toRadians(135))
//                .splineToLinearHeading(new Pose2d(xPosBar - 2,yPosBar+10, Math.toRadians(-90)), Math.toRadians(SplineHeading))
//
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(xPosObserve, yPosOverserve-34 , Math.toRadians(-90)), Math.toRadians(-90))
//
//                .setTangent(Math.toRadians(135))
//                .splineToLinearHeading(new Pose2d(xPosBar - 4,yPosBar+10, Math.toRadians(-90)), Math.toRadians(SplineHeading))
//
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(xPosObserve, yPosOverserve-34 , Math.toRadians(-90)), Math.toRadians(-90))
//
//                .setTangent(Math.toRadians(135))
//                .splineToLinearHeading(new Pose2d(xPosBar - 6,yPosBar+10, Math.toRadians(-90)), Math.toRadians(SplineHeading))
//
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(xPosObserve, yPosOverserve-34 , Math.toRadians(-90)), Math.toRadians(-90))
                //4BUCKET W/ DT
                //Bucket1
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(54,51, Math.toRadians(-135)), Math.toRadians(0))
                //Spike1
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(47, 38, Math.toRadians(-90)), Math.toRadians(-90))
                //Bucket2
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(54,51, Math.toRadians(-135)), Math.toRadians(100))
                //Spike2
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(56, 38, Math.toRadians(-90)), Math.toRadians(-90))
                //Bucket3
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(54,51, Math.toRadians(-135)), Math.toRadians(100))
                //Spike3
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(57, 25, Math.toRadians(0)), Math.toRadians(-90))
                //Bucket4
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(54,51, Math.toRadians(-135)), Math.toRadians(100))
                //Park
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(22,0, Math.toRadians(0)), Math.toRadians(180))
                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}