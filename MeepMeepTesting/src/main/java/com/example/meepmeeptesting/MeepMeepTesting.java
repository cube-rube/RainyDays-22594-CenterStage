package com.example.meepmeeptesting;

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
                                .lineToSplineHeading(new Pose2d(PURPLE_RIGHT_VECTOR, PURPLE_RIGHT_HEADING))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.open_lower();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //scorer.close_lower();
                                    //scorer.open_upper();
                                    //scorer.deploy();
                                })
                                .waitSeconds(0.1)
                                .lineToSplineHeading(new Pose2d(BACKDROP_RIGHT_VECTOR.plus(new Vector2d(0, -1)), Math.toRadians(0))) // move to backdrop
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.open_lower();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                                    //scorer.take();
                                })
                                .waitSeconds(0.1)
                                .setReversed(true)
                                .splineToConstantHeading(RIGGING_UP_VECTOR, Math.toRadians(180))
                                .splineToConstantHeading(RIGGING_DOWN_VECTOR, Math.toRadians(180))
                                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                                    // intake
                                })
                                .splineToConstantHeading(PIXEL_STACK_VECTOR, Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(1)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    // intake stop
                                })
                                .splineToConstantHeading(RIGGING_DOWN_VECTOR, Math.toRadians(0))
                                .splineToConstantHeading(RIGGING_UP_VECTOR, Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    // lift.setReference(570);
                                    // scorer.deploy();
                                })

                                .splineToConstantHeading(BACKDROP_RIGHT_VECTOR.plus(new Vector2d(0, -1)), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    // scorer.open_lower();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                                    // scorer.open_upper();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                                    // scorer.take();
                                    // lift.setReference(0);
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