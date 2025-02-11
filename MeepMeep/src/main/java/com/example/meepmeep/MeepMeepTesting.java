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
    public static double xSpike2 = 57;

    public static double xSpike1First = 25;
//    public static double xPosBar = 2;
//    public static double yPosBar = 35;
    public static double yFinal = -30;

    public static double bucketX = 56;
    public static double bucketY = 56;
    public static double bucketHeading = -135;

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
                .setConstraints(60, 60, Math.toRadians(180) * 3, Math.toRadians(180) * 3, 11.3)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -63, Math.toRadians(90)))
                //THREE SPECI 2 HUMAN PLAYER
//                splineToLinearHeading(new Pose2d(xPosBar,yPosBar, Math.toRadians(90)), Math.toRadians(SplineHeading))
//                        .lineToY(yFinal, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
//
//                        .setTangent(Math.toRadians(-90))
//                        .splineToLinearHeading(new Pose2d(32, -37, Math.toRadians(rightHeading)), Math.toRadians(45))
//                        .splineToLinearHeading(new Pose2d(40, -14, Math.toRadians(rightHeading)), Math.toRadians(0))
//                        .setTangent(0)
//                        .splineToLinearHeading(new Pose2d(xSpike1, -14, Math.toRadians(rightHeading)), Math.toRadians(-90))
//                        .waitSeconds(.1)
//                        .setTangent(Math.toRadians(-90))
//                        .lineToY(yPosOverserve + 5)
//                        .setTangent(Math.toRadians(-90))
//                        .lineToY(yPosOverserve-15, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
//
//                        .waitSeconds(.2)
//                        .splineToLinearHeading(new Pose2d(xSpike1, yPosOverserve, Math.toRadians(90)), Math.toRadians(90))
//                        .splineToLinearHeading(new Pose2d(xPosBar + 3,yPosBar, Math.toRadians(90)), Math.toRadians(SplineHeading))
//                        .lineToY(yFinal, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
//
//                        .setTangent(Math.toRadians(-90))
//                        .splineToLinearHeading(new Pose2d(xPosObserve,yPosOverserve + 5, Math.toRadians(rightHeading)),Math.toRadians(-90))
//                        .lineToY(yPosOverserve)
//                        .setTangent(Math.toRadians(-90))
//                        .lineToY(yPosOverserve-12, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
//                        .waitSeconds(.1)
//                        .splineToLinearHeading(new Pose2d(xPosObserve, yPosOverserve, Math.toRadians(90)), Math.toRadians(90))
//                        .setTangent(Math.toRadians(90))
//                        .splineToLinearHeading(new Pose2d(xPosBar - 10 ,yPosBar, Math.toRadians(90)), Math.toRadians(SplineHeading))
//                        .lineToY(yFinal, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
//                        .setTangent(Math.toRadians(-90))
//                        .splineToLinearHeading(new Pose2d(xPosObserve,yPosOverserve + 5, Math.toRadians(rightHeading)),Math.toRadians(-90))
//                        .lineToY(yPosOverserve).

                //3 SPECI 1 HUMAN PLAYER
//                .splineToLinearHeading(new Pose2d(xPosBar,yPosBar, Math.toRadians(90)), Math.toRadians(SplineHeading))
//                .lineToY(yFinal, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
//
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(32, -37, Math.toRadians(rightHeading)), Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(40, -14, Math.toRadians(rightHeading)), Math.toRadians(0))
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(xSpike1, -14, Math.toRadians(rightHeading)), Math.toRadians(-90))
//                .waitSeconds(.1)
//                .setTangent(Math.toRadians(-90))
//                .lineToY(yPosOverserve + 5)
//
//                .splineToLinearHeading(new Pose2d(xSpike1, -14, Math.toRadians(90)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(xSpike2, -14, Math.toRadians(90)), Math.toRadians(-90))
//                .setTangent(Math.toRadians(-90))
//                .lineToY(yPosOverserve + 5)
//                .setTangent(Math.toRadians(-90))
//                .lineToY(yPosOverserve-5, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
//
//                .waitSeconds(.2)
//                .splineToLinearHeading(new Pose2d(xSpike2, yPosOverserve, Math.toRadians(90)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(xPosBar + 3,yPosBar, Math.toRadians(90)), Math.toRadians(SplineHeading))
//                .lineToY(yFinal, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
//
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(xPosObserve,yPosOverserve + 5, Math.toRadians(rightHeading)),Math.toRadians(-90))
//                .lineToY(yPosOverserve)
//                .setTangent(Math.toRadians(-90))
//                .lineToY(yPosOverserve-5, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
//
//                .splineToLinearHeading(new Pose2d(xPosObserve, yPosOverserve, Math.toRadians(90)), Math.toRadians(90))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(xPosBar - 10 ,yPosBar, Math.toRadians(90)), Math.toRadians(SplineHeading))
//                .lineToY(yFinal, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(xPosObserve,yPosOverserve + 5, Math.toRadians(rightHeading)),Math.toRadians(-90))
//                .lineToY(yPosOverserve)
//                .lineToY(yPosOverserve-5, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
//
//                .splineToLinearHeading(new Pose2d(xPosObserve, yPosOverserve, Math.toRadians(90)), Math.toRadians(90))
//                .setTangent(Math.toRadians(90))
////                Final Cycle
//                .splineToLinearHeading(new Pose2d(xPosBar - 7 ,yPosBar, Math.toRadians(90)), Math.toRadians(SplineHeading))
//                .lineToY(yFinal, new TranslationalVelConstraint(baseTransVel), baseAccelConstraint)
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(xPosObserve,yPosOverserve + 5, Math.toRadians(rightHeading)),Math.toRadians(-90))
//                .lineToY(yPosOverserve)

//                .splineToLinearHeading(new Pose2d(xPosBar,yPosBar, Math.toRadians(-90)), Math.toRadians(-90))
//                .lineToY(yFinal)
//                .lineToY(yFinal+1)
//                .splineToLinearHeading(new Pose2d(spikeX1, spikeY1, Math.toRadians(-90)), Math.toRadians(-90))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(bucketX,bucketY, Math.toRadians(bucketHeading)), Math.toRadians(100))
//                .setTangent(Math.toRadians(290))
//                .splineToLinearHeading(new Pose2d(spikeX2, spikeY2, Math.toRadians(-90)), Math.toRadians(-90))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(bucketX,bucketY, Math.toRadians(bucketHeading)), Math.toRadians(100))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(spikeX3, spikeY3, Math.toRadians(0)), Math.toRadians(-90))
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(bucketX,bucketY, Math.toRadians(bucketHeading)), Math.toRadians(100))
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(parkX,parkY, Math.toRadians(0)), Math.toRadians(180))



                .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}