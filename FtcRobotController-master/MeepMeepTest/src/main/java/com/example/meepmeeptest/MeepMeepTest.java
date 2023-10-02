package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60,Math.toRadians(277.573824), Math.toRadians(277.573824), 13.6)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62, Math.toRadians(270)))

                                .lineTo(new Vector2d(-35,-35))
                                .turn(Math.toRadians(-45))
                                .waitSeconds(.5)
                                .lineToLinearHeading(new Pose2d(-35,-25, Math.toRadians(180)))
 //                               .lineTo(new Vector2d(-60,-12))
//
//                                .waitSeconds(.5)
//                                .splineToConstantHeading(new Vector2d(-60,-10), Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(-55,-12), Math.toRadians(180))

//                                 .splineToConstantHeading(new Vector2d(-60,-12), Math.toRadians(100)) //530
                                .splineTo(new Vector2d(-35,-22),Math.toRadians(180))
                                //.splineToSplineHeading(new Pose2d(-35,-22, Math.toRadians(140)), Math.toRadians(280))

//                                .lineToLinearHeading(new Pose2d(-35,-12, Math.toRadians(180)))
//                                .turn(Math.toRadians(-45))
//                                .waitSeconds(.5)
//                                .lineToLinearHeading(new Pose2d(-60,-12, Math.toRadians(180)))
                          //      .lineTo(new Vector2d(-60,-12))

                                .waitSeconds(.5)

                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}