package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.PositionConstants.BACKDROP_CENTER_COORDS;
import static com.example.meepmeeptesting.PositionConstants.BACKDROP_CENTER_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.BACKDROP_LEFT_COORDS;
import static com.example.meepmeeptesting.PositionConstants.BACKDROP_RIGHT_COORDS;
import static com.example.meepmeeptesting.PositionConstants.BACKDROP_RIGHT_VECTOR;
import static com.example.meepmeeptesting.PositionConstants.DOOR_DOWN_COORDS;
import static com.example.meepmeeptesting.PositionConstants.DOOR_UP_COORDS;
import static com.example.meepmeeptesting.PositionConstants.END_NEAR;
import static com.example.meepmeeptesting.PositionConstants.FAR_START_COORDS;
import static com.example.meepmeeptesting.PositionConstants.FIRST_PIXEL_STACK_COORDS;
import static com.example.meepmeeptesting.PositionConstants.NEAR_START_COORDS;
import static com.example.meepmeeptesting.PositionConstants.PURPLE_CENTER_FAR;
import static com.example.meepmeeptesting.PositionConstants.PURPLE_CENTER_NEAR;
import static com.example.meepmeeptesting.PositionConstants.PURPLE_LEFT_FAR;
import static com.example.meepmeeptesting.PositionConstants.PURPLE_LEFT_NEAR;
import static com.example.meepmeeptesting.PositionConstants.PURPLE_RIGHT_NEAR;
import static com.example.meepmeeptesting.PositionConstants.RIGGING_DOWN_COORDS;
import static com.example.meepmeeptesting.PositionConstants.RIGGING_UP_COORDS;
import static com.example.meepmeeptesting.PositionConstants.SECOND_PIXEL_STACK_COORDS;
import static com.example.meepmeeptesting.PositionConstants.START_HEADING;
import static com.example.meepmeeptesting.PositionConstants.THIRD_PIXEL_STACK_COORDS;

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
                .setConstraints(25, 75, 3.5, 50, 6.5)
                .setDimensions(16.83, 15.7677)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(NEAR_START_COORDS[0], NEAR_START_COORDS[1], START_HEADING))
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(35, 3.5, 6.5),
                                        SampleMecanumDrive.getAccelerationConstraint(50))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.closeLower();
                                    //scorer.closeUpper();
                                    //scorer.deployAuto();
                                    //lift.resetEncoders();
                                })
                                .waitSeconds(0.8)
                                .lineTo(new Vector2d(PURPLE_CENTER_NEAR[0], PURPLE_CENTER_NEAR[1]))
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

                                // GOING TO BACKDROP
                                .lineToSplineHeading(new Pose2d(BACKDROP_CENTER_COORDS[0], BACKDROP_CENTER_COORDS[1], Math.toRadians(0)))
                                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                                    //scorer.openUpper();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                                    //scorer.take();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                                    //intake.eject();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
                                    //intake.stop();
                                })
                                .waitSeconds(0.4)

                                // GOING TO STACK
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(RIGGING_UP_COORDS[0], RIGGING_UP_COORDS[1]), Math.toRadians(180))
                                .setConstraints(SampleMecanumDrive.getVelocityConstraint(25, 3.5, 6.5),
                                        SampleMecanumDrive.getAccelerationConstraint(50))
                                .splineToConstantHeading(new Vector2d(RIGGING_DOWN_COORDS[0], RIGGING_DOWN_COORDS[1]), Math.toRadians(180))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.openLower();
                                    //scorer.openUpper();
                                    //intake.take();
                                    //intake.openLeftFlap();
                                })
                                .splineToConstantHeading(new Vector2d(FIRST_PIXEL_STACK_COORDS[0] + 0.5, FIRST_PIXEL_STACK_COORDS[1] + 8.5), Math.toRadians(270))
                                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                                    //intake.closeLeftFlap();
                                })
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, 3.5, 6.5))
                                .splineToConstantHeading(new Vector2d(FIRST_PIXEL_STACK_COORDS[0] - 1.2, FIRST_PIXEL_STACK_COORDS[1] - 12), Math.toRadians(270))
                                .lineToConstantHeading(new Vector2d(FIRST_PIXEL_STACK_COORDS[0] - 1.2, FIRST_PIXEL_STACK_COORDS[1] + 8.5))
                                .setReversed(false)
                                .waitSeconds(0.8)
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, 3.5, 6.5))

                                // GOING TO BACKDROP
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.closeLower();
                                    //scorer.closeUpper();
                                    //intake.eject();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                                    //intake.stop();
                                })
                                .splineToConstantHeading(new Vector2d(RIGGING_DOWN_COORDS[0], RIGGING_DOWN_COORDS[1]), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(RIGGING_UP_COORDS[0], RIGGING_UP_COORDS[1]), Math.toRadians(0))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(35, 3.5, 6.5))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.deploy();
                                    //lift.setReference(570);
                                })
                                .splineToConstantHeading(new Vector2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1]), Math.toRadians(0))
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    //scorer.openLower();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                                    //scorer.openUpper();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                                    //scorer.take();
                                    //lift.setReference(0);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                                    //intake.eject();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
                                    //intake.stop();
                                })
                                .resetVelConstraint()
                                .waitSeconds(0.6)

                                // PARKING
                                .lineToSplineHeading(new Pose2d(END_NEAR[0], END_NEAR[1], Math.toRadians(END_NEAR[2])))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}