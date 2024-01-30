package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.PositionConstants.BACKDROP_CENTER_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.BACKDROP_RIGHT_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.NEAR_START_POSE;
import static com.example.meepmeeptesting.PositionConstants.PIXEL_STACK_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.PURPLE_CENTER_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.PURPLE_RIGHT_HEADING;
import static com.example.meepmeeptesting.PositionConstants.PURPLE_RIGHT_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.RIGGING_DOWN_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.RIGGING_UP_VECTOR;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(35, 60, 5.375, 5.375, 4.95)
                .setDimensions(16.7, 15.6)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(NEAR_START_POSE)
                                .lineTo(PURPLE_CENTER_VECTOR)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.open_lower();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //scorer.close_lower();
                                    //scorer.open_upper();
                                    //scorer.deploy();
                                })
                                .waitSeconds(0.1)
                                .lineToSplineHeading(new Pose2d(BACKDROP_CENTER_VECTOR, Math.toRadians(0)))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.open_lower();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                                    //scorer.take();
                                })
                                .waitSeconds(0.1)
                                .UNSTABLE_addTemporalMarkerOffset(2.5, () -> {
                                    // intake
                                })
                                .lineToConstantHeading(PIXEL_STACK_VECTOR)
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    // intake stop
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2.1, () -> {
                                    // lift up
                                    // rotate scorer
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2.4, () -> {
                                    // lift down
                                })
                                .lineToConstantHeading(BACKDROP_CENTER_VECTOR)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    // release
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                                    // take
                                })
                                .waitSeconds(1)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}