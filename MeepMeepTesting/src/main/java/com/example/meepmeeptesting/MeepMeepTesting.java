package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.PositionConstants.BACKDROP_CENTER_COORDS;
import static com.example.meepmeeptesting.PositionConstants.BACKDROP_CENTER_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.BACKDROP_RIGHT_COORDS;
import static com.example.meepmeeptesting.PositionConstants.BACKDROP_RIGHT_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.DOOR_DOWN_COORDS;
import static com.example.meepmeeptesting.PositionConstants.DOOR_UP_COORDS;
import static com.example.meepmeeptesting.PositionConstants.FAR_START_COORDS;
import static com.example.meepmeeptesting.PositionConstants.FIRST_PIXEL_STACK_COORDS;
import static com.example.meepmeeptesting.PositionConstants.PURPLE_CENTER_FAR;
import static com.example.meepmeeptesting.PositionConstants.SECOND_PIXEL_STACK_COORDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 75, 3.5, 50, 6.5)
                .setDimensions(16.83, 15.7677)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(FAR_START_COORDS[0], FAR_START_COORDS[1], Math.toRadians(90)))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.closeLower();
                                    //scorer.closeUpper();
                                    //scorer.deployAuto();
                                    //lift.resetEncoders();
                                })
                                .waitSeconds(0.8)
                                .lineTo(new Vector2d(PURPLE_CENTER_FAR[0], PURPLE_CENTER_FAR[1]))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.openLower();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                                    //scorer.deployAutoPush();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //scorer.deploy();
                                })
                                .waitSeconds(0.2)

                                // going to stack
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //intake.take();
                                    //intake.openRightFlap();
                                })
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(SECOND_PIXEL_STACK_COORDS[0],
                                        SECOND_PIXEL_STACK_COORDS[1],
                                        Math.toRadians(0)), Math.toRadians(90))
                                .waitSeconds(0.8)
                                .setReversed(false)

                                // going to backdrop
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.closeLower();
                                    //scorer.closeUpper();
                                    //intake.eject();
                                    //intake.closeRightFlap();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                                    //intake.stop();
                                })
                                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1]), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.deploy();
                                    //lift.setReference(570);
                                })
                                .resetVelConstraint()
                                .splineToConstantHeading(new Vector2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1]), Math.toRadians(0))

                                // deploying white pixel
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.openLower();
                                })
                                .waitSeconds(0.2)
                                // deploying yellow pixel
                                .lineTo(new Vector2d(BACKDROP_CENTER_COORDS[0], BACKDROP_CENTER_COORDS[1]))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.openUpper();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //scorer.take();
                                    //lift.setReference(0);
                                })
                                .waitSeconds(0.4)

                                // going to stack
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1]), Math.toRadians(180))
                                .resetVelConstraint()
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //intake.take();
                                })
                                .splineToConstantHeading(new Vector2d(SECOND_PIXEL_STACK_COORDS[0], SECOND_PIXEL_STACK_COORDS[1]), Math.toRadians(250))
                                .waitSeconds(0.8)

                                // going to backdrop
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                                .setReversed(false)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.closeLower();
                                    //scorer.closeUpper();
                                    //intake.eject();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                                    //intake.stop();
                                })
                                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1]), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.deploy();
                                    //lift.setReference(570);
                                })
                                .resetVelConstraint()
                                .splineToConstantHeading(new Vector2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1]), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.openLower();
                                    //scorer.openUpper();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //scorer.take();
                                    //lift.setReference(0);
                                })
                                .waitSeconds(0.3)

                                // going to stack
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1]), Math.toRadians(180))
                                .resetVelConstraint()
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //intake.take();
                                })
                                .splineToConstantHeading(new Vector2d(SECOND_PIXEL_STACK_COORDS[0], SECOND_PIXEL_STACK_COORDS[1]), Math.toRadians(250))
                                .setReversed(false)
                                .waitSeconds(0.8)

                                // going to backdrop
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                                .setReversed(false)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.closeLower();
                                    //scorer.closeUpper();
                                    //intake.eject();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                                    //intake.stop();
                                })
                                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1]), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.deploy();
                                    //lift.setReference(570);
                                })
                                .resetVelConstraint()
                                .splineToConstantHeading(new Vector2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1]), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.openLower();
                                    //scorer.openUpper();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    //scorer.take();
                                    //lift.setReference(0);
                                })
                                .waitSeconds(0.3)

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