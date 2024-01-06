package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity redAlliance = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48180821614297, 52.48180821614297, 3.5906233757563073, 3.5906233757563073, 13.19)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33, -61.5, Math.toRadians(90)))
                                .forward(29)
                                .turn(Math.toRadians(90))
                                .waitSeconds(1.5)
                                .strafeRight(20)
                                .back(25)
                                .splineTo(new Vector2d(48,-34), 0)
                                .waitSeconds(1.5)
                                .strafeLeft(25)
                                .build()
                );

        RoadRunnerBotEntity redAudience = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -70, Math.toRadians(90)))
                                .waitSeconds(7)
                                .forward(35)
                                .waitSeconds(8)
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(redAudience)
                .addEntity(redAlliance)
                .start();
    }
}