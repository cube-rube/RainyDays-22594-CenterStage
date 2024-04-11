package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.BACKDROP_CENTER_COORDS;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.BACKDROP_LEFT_COORDS;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.BACKDROP_RIGHT_COORDS;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.DOOR_DOWN_COORDS;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.DOOR_UP_COORDS;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.END_NEAR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.NEAR_START_COORDS;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.PURPLE_CENTER_NEAR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.PURPLE_LEFT_NEAR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.PURPLE_RIGHT_NEAR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.START_HEADING;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.THIRD_PIXEL_STACK_COORDS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.misc.GameConstants;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.Scorer;
import org.firstinspires.ftc.teamcode.modules.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.modules.vision.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "BlueNear2+4")
public class BlueNear2_4 extends LinearOpMode {
    private FtcDashboard dashboard;
    private SampleMecanumDrive drive;
    private Intake intake;
    private Lift lift;
    private Scorer scorer;
    private OpenCvWebcam webcam;
    private PropDetectionPipeline pipeline;
    private PropDetectionPipeline.PropPosition position;
    private TrajectorySequence traj1;

    @Override
    public void runOpMode() throws InterruptedException {
        init_robot();
        final boolean[] camera_init = {true};
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                while (camera_init[0]) {
                    telemetry.addLine("camera turned on");
                    telemetry.addData("prop_pos", pipeline.getPropPosition());
                    telemetry.update();
                }
            }
            @Override
            public void onError(int errorCode) {
            }
        });


        sleep(3000);

        position = pipeline.getPropPosition();
        telemetry.addData("prop_pos", position);
        telemetry.update();

        switch (position) {
            case RIGHT:
                move_right();
                break;
            case CENTER:
                move_center();
                break;
            case LEFT:
                move_left();
                break;
        }
        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {

            }
        });

        camera_init[0] = false;
        while (opModeInInit()) {
            telemetry.addLine("finished building trajectory");
            telemetry.addData("prop_pos", position);
            telemetry.update();
        }
        waitForStart();
        resetRuntime();
        start_traj();

        ElapsedTime intakeTimer = new ElapsedTime();
        double timeLeft = 0;
        while (opModeIsActive()) {
            PoseCache.pose = drive.getPoseEstimate();
            lift.PIDControl();
            intake.autoControl();
            drive.update();

            if (intake.intakeState == Intake.IntakeState.INTAKE) {
                if (scorer.getLowerPixel() && scorer.getUpperPixel()) {
                    if (intakeTimer.seconds() >= 0.15) {
                        intake.stop();
                        scorer.closeUpper();
                        scorer.closeLower();
                    }
                } else {
                    intakeTimer.reset();
                }
            } else {
                intakeTimer.reset();
            }
            if (!drive.isBusy()) {
                if (timeLeft == 0) {
                    timeLeft = 30 - getRuntime();
                }
                telemetry.addData("TIME LEFT:", timeLeft);
            }

            telemetry.addData("Time", getRuntime());
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }

    private void start_traj() {
        drive.followTrajectorySequenceAsync(traj1);
    }

    private void move_left() {
        traj1 = drive.trajectorySequenceBuilder(new Pose2d(NEAR_START_COORDS[0], NEAR_START_COORDS[1], START_HEADING))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    scorer.deployAuto();
                    lift.resetEncoders();
                })
                .waitSeconds(0.8)
                .lineTo(new Vector2d(PURPLE_LEFT_NEAR[0] + 1.5, PURPLE_LEFT_NEAR[1]))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    scorer.deployAutoPush();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorer.deployAutoUp();
                })
                .waitSeconds(0.2)

                // GOING TO BACKDROP
                .lineToSplineHeading(new Pose2d(BACKDROP_LEFT_COORDS[0], BACKDROP_LEFT_COORDS[1] + 1.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                    scorer.take();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
                    intake.stop();
                })
                .waitSeconds(0.4)

                // GOING TO STACK
                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0] + 15, DOOR_UP_COORDS[1] - 3), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1] - 3), Math.toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                    scorer.openUpper();
                    intake.take();
                    intake.openRightFlap();
                })
                .splineToConstantHeading(new Vector2d(THIRD_PIXEL_STACK_COORDS[0] - 2, THIRD_PIXEL_STACK_COORDS[1] - 7), Math.toRadians(90))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    intake.closeRightFlap();
                })
                .splineToConstantHeading(new Vector2d(THIRD_PIXEL_STACK_COORDS[0] - 2.7, THIRD_PIXEL_STACK_COORDS[1] + 9), Math.toRadians(90))
                .setReversed(false)
                .resetVelConstraint()
                .waitSeconds(0.4)

                // GOING TO BACKDROP
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intake.stop();
                })
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1] + 3), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0] + 15, DOOR_UP_COORDS[1] + 3), Math.toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                    lift.setReference(570);
                })
                .splineToConstantHeading(new Vector2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1]), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    scorer.take();
                    lift.setReference(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
                    intake.stop();
                })
                .resetVelConstraint()
                .waitSeconds(0.6)

                // GOING TO STACK
                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0] + 15, DOOR_UP_COORDS[1] - 3), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1] - 3), Math.toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    scorer.openLower();
                    scorer.openUpper();
                    intake.take();
                })
                .splineToConstantHeading(new Vector2d(THIRD_PIXEL_STACK_COORDS[0] - 2, THIRD_PIXEL_STACK_COORDS[1] - 7), Math.toRadians(90))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, 3.5, 6.5))
                .splineToConstantHeading(new Vector2d(THIRD_PIXEL_STACK_COORDS[0] - 2.7, THIRD_PIXEL_STACK_COORDS[1] + 12), Math.toRadians(90))
                .setReversed(false)
                .resetVelConstraint()
                .waitSeconds(0.4)

                // GOING TO BACKDROP
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intake.stop();
                })
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1] + 3), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0] + 15, DOOR_UP_COORDS[1] + 3), Math.toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                    lift.setReference(570);
                })
                .splineToConstantHeading(new Vector2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1]), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    scorer.take();
                    lift.setReference(0);
                })
                .resetVelConstraint()
                .waitSeconds(0.6)

                // PARKING
                .lineToSplineHeading(new Pose2d(END_NEAR[0], END_NEAR[1], Math.toRadians(END_NEAR[2])))
                .build();
    }

    private void move_center() {
        traj1 = drive.trajectorySequenceBuilder(new Pose2d(NEAR_START_COORDS[0], NEAR_START_COORDS[1], START_HEADING))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    scorer.deployAuto();
                    lift.resetEncoders();
                })
                .waitSeconds(0.8)
                .lineTo(new Vector2d(PURPLE_CENTER_NEAR[0], PURPLE_CENTER_NEAR[1]))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    scorer.deployAutoPush();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    scorer.deploy();
                })
                .waitSeconds(0.2)

                // GOING TO BACKDROP
                .lineToSplineHeading(new Pose2d(BACKDROP_CENTER_COORDS[0], BACKDROP_CENTER_COORDS[1], Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                    scorer.take();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
                    intake.stop();
                })
                .waitSeconds(0.4)

                // GOING TO STACK
                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1] - 2.5), Math.toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                    scorer.openUpper();
                    intake.take();
                    intake.openRightFlap();
                })
                .splineToConstantHeading(new Vector2d(THIRD_PIXEL_STACK_COORDS[0] - 2, THIRD_PIXEL_STACK_COORDS[1] - 7), Math.toRadians(90))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    intake.closeRightFlap();
                })
                .splineToConstantHeading(new Vector2d(THIRD_PIXEL_STACK_COORDS[0] - 2.7, THIRD_PIXEL_STACK_COORDS[1] + 9), Math.toRadians(90))
                .setReversed(false)
                .resetVelConstraint()
                .waitSeconds(0.4)

                // GOING TO BACKDROP
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intake.stop();
                })
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1] + 2), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1] + 2), Math.toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                    lift.setReference(570);
                })
                .splineToConstantHeading(new Vector2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1]), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    scorer.take();
                    lift.setReference(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
                    intake.stop();
                })
                .resetVelConstraint()
                .waitSeconds(0.6)

                // GOING TO STACK
                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1] - 2.5), Math.toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    scorer.openLower();
                    scorer.openUpper();
                    intake.take();
                })
                .splineToConstantHeading(new Vector2d(THIRD_PIXEL_STACK_COORDS[0] - 2.7, THIRD_PIXEL_STACK_COORDS[1] - 7), Math.toRadians(90))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, 3.5, 6.5))
                .splineToConstantHeading(new Vector2d(THIRD_PIXEL_STACK_COORDS[0] - 2.7, THIRD_PIXEL_STACK_COORDS[1] + 12), Math.toRadians(90))
                .setReversed(false)
                .resetVelConstraint()
                .waitSeconds(0.4)

                // GOING TO BACKDROP
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intake.stop();
                })
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1] + 2), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1] + 2), Math.toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                    lift.setReference(700);
                })
                .splineToConstantHeading(new Vector2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1]), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    scorer.take();
                    lift.setReference(0);
                })
                .resetVelConstraint()
                .waitSeconds(0.7)

                // PARKING
                .lineToSplineHeading(new Pose2d(END_NEAR[0], END_NEAR[1], Math.toRadians(END_NEAR[2])))
                .build();
    }

    private void move_right() {
        traj1 = drive.trajectorySequenceBuilder(new Pose2d(NEAR_START_COORDS[0], NEAR_START_COORDS[1], START_HEADING))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    scorer.deployAuto();
                    lift.resetEncoders();
                })
                .waitSeconds(0.8)
                .lineToLinearHeading(new Pose2d(PURPLE_RIGHT_NEAR[0], PURPLE_RIGHT_NEAR[1], Math.toRadians(PURPLE_RIGHT_NEAR[2])))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    scorer.deployAutoPush();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorer.deploy();
                })
                .waitSeconds(0.2)

                // GOING TO BACKDROP
                .lineToSplineHeading(new Pose2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1], Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                    scorer.take();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
                    intake.stop();
                })
                .waitSeconds(0.4)

                // GOING TO STACK
                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1] - 1.5), Math.toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(25, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                    scorer.openUpper();
                    intake.take();
                    intake.openRightFlap();
                })
                .splineToConstantHeading(new Vector2d(THIRD_PIXEL_STACK_COORDS[0] - 2, THIRD_PIXEL_STACK_COORDS[1] - 7), Math.toRadians(90))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.closeRightFlap();
                })
                .splineToConstantHeading(new Vector2d(THIRD_PIXEL_STACK_COORDS[0] - 2, THIRD_PIXEL_STACK_COORDS[1] + 9), Math.toRadians(90))
                .setReversed(false)
                .resetVelConstraint()
                .waitSeconds(0.4)

                // GOING TO BACKDROP
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intake.stop();
                })
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1]), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1] - 1.5), Math.toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                    lift.setReference(570);
                })
                .splineToConstantHeading(new Vector2d(BACKDROP_CENTER_COORDS[0], BACKDROP_CENTER_COORDS[1]), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    scorer.take();
                    lift.setReference(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(2.2, () -> {
                    intake.stop();
                })
                .resetVelConstraint()
                .waitSeconds(0.6)

                // GOING TO STACK
                .setReversed(true)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1] - 3), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1] - 3), Math.toRadians(180))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    scorer.openLower();
                    scorer.openUpper();
                    intake.take();
                })
                .splineToConstantHeading(new Vector2d(THIRD_PIXEL_STACK_COORDS[0] - 2, THIRD_PIXEL_STACK_COORDS[1] - 7), Math.toRadians(90))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, 3.5, 6.5))
                .splineToConstantHeading(new Vector2d(THIRD_PIXEL_STACK_COORDS[0] - 2, THIRD_PIXEL_STACK_COORDS[1] + 12), Math.toRadians(90))
                .setReversed(false)
                .resetVelConstraint()
                .waitSeconds(0.4)

                // GOING TO BACKDROP
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intake.stop();
                })
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1]), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(0))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                    lift.setReference(570);
                })
                .splineToConstantHeading(new Vector2d(BACKDROP_CENTER_COORDS[0] + 0.5, BACKDROP_CENTER_COORDS[1]), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    scorer.take();
                    lift.setReference(0);
                })
                .resetVelConstraint()
                .waitSeconds(0.6)

                // PARKING
                .lineToSplineHeading(new Pose2d(END_NEAR[0], END_NEAR[1], Math.toRadians(END_NEAR[2])))
                .build();
    }

    public void init_robot() {
        dashboard = FtcDashboard.getInstance();

        drive = new SampleMecanumDrive(this.hardwareMap);
        Pose2d startPose = new Pose2d(NEAR_START_COORDS[0], NEAR_START_COORDS[1], START_HEADING);
        drive.setPoseEstimate(startPose);
        PoseCache.pose = startPose;
        intake = new Intake(this);
        lift = new Lift(this, dashboard);
        scorer = new Scorer(this, lift);

        GameConstants.ALLIANCE_COLOR = AllianceColor.BLUE;
        GameConstants.STARTPOS = GameConstants.StartPos.NEAR;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new PropDetectionPipeline();
        webcam.setPipeline(pipeline);
        telemetry.update();
    }
}
