package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.PositionConstants.BACKDROP_CENTER_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.BACKDROP_LEFT_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.BACKDROP_RIGHT_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.DIFF_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.NEAR_START_POSE;
import static com.example.meepmeeptesting.PositionConstants.PIXEL_STACK_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.PURPLE_CENTER_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.PURPLE_LEFT_HEADING;
import static com.example.meepmeeptesting.PositionConstants.PURPLE_LEFT_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.PURPLE_RIGHT_HEADING;
import static com.example.meepmeeptesting.PositionConstants.PURPLE_RIGHT_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.RIGGING_DOWN_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.RIGGING_UP_VECTOR;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
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
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.closeLower();
                                    //scorer.closeUpper();
                                    //scorer.deployAuto();
                                    //lift.resetEncoders();
                                })
                                .waitSeconds(0.8)
                                .lineTo(PURPLE_CENTER_VECTOR)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.openLower();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                                    //scorer.deploy();
                                })
                                .waitSeconds(0.1)
                                .lineToSplineHeading(new Pose2d(BACKDROP_CENTER_VECTOR.plus(DIFF_VECTOR), Math.toRadians(0)))
                                .waitSeconds(0.05)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.openUpper();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                                    //scorer.take();
                                })
                                .waitSeconds(0.4)
                                .setReversed(true)
                                .splineToConstantHeading(RIGGING_UP_VECTOR, Math.toRadians(180))
                                .splineToConstantHeading(RIGGING_DOWN_VECTOR, Math.toRadians(180))
                                .splineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(0, -6)), Math.toRadians(180))

                                .setReversed(false)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //intake.take()
                                })
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, 5.375, 4.95))
                                .lineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(0, 5)))
                                .resetVelConstraint()

                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.closeLower();
                                    //scorer.closeUpper();
                                    //intake.eject();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                                    //intake.stop();
                                })
                                .lineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(5,5)))
                                .splineToConstantHeading(RIGGING_DOWN_VECTOR.plus(new Vector2d(0, 0.5)), Math.toRadians(0))
                                .splineToConstantHeading(RIGGING_UP_VECTOR.plus(new Vector2d(0, 0.5)), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.deploy();
                                    //lift.setReference(420);
                                })
                                .splineToConstantHeading(BACKDROP_RIGHT_VECTOR.plus(DIFF_VECTOR), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //scorer.openLower();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                    //scorer.openUpper();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                                    //scorer.take();
                                    //lift.setReference(0);
                                })
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(47, -60, Math.toRadians(90)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}