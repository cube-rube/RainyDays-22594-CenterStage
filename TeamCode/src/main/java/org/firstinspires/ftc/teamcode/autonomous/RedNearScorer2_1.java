package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.BACKDROP_CENTER_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.BACKDROP_LEFT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.BACKDROP_RIGHT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.DIFF_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.NEAR_START_POSE;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.PIXEL_STACK_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.PURPLE_CENTER_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.PURPLE_LEFT_HEADING;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.PURPLE_LEFT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.PURPLE_RIGHT_HEADING;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.PURPLE_RIGHT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.RIGGING_DOWN_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.RIGGING_UP_VECTOR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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

@Autonomous(name = "RedNearScorer2+2")
public class RedNearScorer2_1 extends LinearOpMode {
    private FtcDashboard dashboard;
    private SampleMecanumDrive drive;
    private Intake intake;
    private Lift lift;
    private Scorer scorer;
    private OpenCvWebcam webcam;
    private PropDetectionPipeline pipeline;
    private PropDetectionPipeline.PropPosition position;
    private enum TrajectoryState {
        TRAJECTORY_1,
        INTAKE_1,
        INTAKE_2,
        TRAJECTORY_2,
        IDLE
    }
    private TrajectoryState currentTrajectory = TrajectoryState.TRAJECTORY_1;
    private TrajectorySequence traj1, intake_traj1, intake_traj2, traj2;
    private double temp = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        init_robot();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                while (opModeInInit()) {
                    telemetry.addLine("camera turned on");
                    telemetry.addData("prop_pos", pipeline.getPropPosition());
                    telemetry.update();
                }
            }
            @Override
            public void onError(int errorCode) {
            }
        });

        waitForStart();
        resetRuntime();
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

        double delta = 0, deltaIntake = 0;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            switch (currentTrajectory) {
                case TRAJECTORY_1:
                    if (!drive.isBusy()) {
                        currentTrajectory = TrajectoryState.INTAKE_1;
                        drive.followTrajectorySequenceAsync(intake_traj1);
                    }
                    break;
                case INTAKE_1:
                    if (!drive.isBusy()) {
                        if (scorer.getLowerPixel() && scorer.getUpperPixel() || getRuntime() >= 19.5) {
                            delta += timer.seconds();
                            if (delta >= 0.5) {
                                currentTrajectory = TrajectoryState.TRAJECTORY_2;
                                drive.followTrajectorySequenceAsync(traj2);
                            }
                        } else {
                            delta = 0;
                            deltaIntake += timer.seconds();
                            if (deltaIntake >= 0.2) {
                                if (intake.isMoving()) {
                                    intake.stop();
                                } else {
                                    intake.take();
                                }
                                deltaIntake = 0;
                            }
                        }
                        timer.reset();
                    } else {
                        timer.reset();
                    }
                    break;

            }
            PoseCache.pose = drive.getPoseEstimate();
            lift.PIDControl();
            intake.autoControl();
            drive.update();
            telemetry.addData("Time", getRuntime());
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
            telemetry.addData("temp_X", temp);
            telemetry.addData("deltaIntake", deltaIntake);
            telemetry.update();
        }
    }

    private void move_left() {
        traj1 = drive.trajectorySequenceBuilder(NEAR_START_POSE)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    scorer.deployAuto();
                    lift.resetEncoders();
                })
                .waitSeconds(0.8)
                .lineToSplineHeading(new Pose2d(PURPLE_LEFT_VECTOR, PURPLE_LEFT_HEADING))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deployAutoPush();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorer.deploy();
                    scorer.closeLower();
                    scorer.openUpper();
                    intake.stop();
                })
                .waitSeconds(0.2)
                .lineToSplineHeading(new Pose2d(BACKDROP_LEFT_VECTOR.minus(DIFF_VECTOR), Math.toRadians(0)))
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                    scorer.take();
                })
                .waitSeconds(0.4)
                .setReversed(true)
                .splineToConstantHeading(RIGGING_UP_VECTOR, Math.toRadians(180))
                .splineToConstantHeading(RIGGING_DOWN_VECTOR, Math.toRadians(180))
                .splineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(-3.2, -12)), Math.toRadians(180))
                .waitSeconds(0.2)
                .build();

        intake_traj1 = drive.trajectorySequenceBuilder(traj1.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15,
                        DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(-3.2, 1)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.take())
                .build();

        traj2 = drive.trajectorySequenceBuilder(intake_traj1.end())
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intake.stop();
                })
                .lineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(5,0)))
                .splineToConstantHeading(RIGGING_DOWN_VECTOR.plus(new Vector2d(0, -0.2)), Math.toRadians(0))
                .splineToConstantHeading(RIGGING_UP_VECTOR.plus(new Vector2d(0, -0.2)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                    lift.setReference(570);
                })
                .splineToConstantHeading(BACKDROP_RIGHT_VECTOR.plus(new Vector2d(2.5,0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    scorer.take();

                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.setReference(0);
                })
                .waitSeconds(1.5)
                .lineToSplineHeading(new Pose2d(47, -60, Math.toRadians(90)))
                .build();

        drive.followTrajectorySequenceAsync(traj1);
    }

    private void move_center() {
        traj1 = drive.trajectorySequenceBuilder(NEAR_START_POSE)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    scorer.deployAuto();
                    lift.resetEncoders();
                })
                .waitSeconds(0.8)
                .lineTo(PURPLE_CENTER_VECTOR)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorer.deploy();
                })
                .waitSeconds(0.2)
                .lineToSplineHeading(new Pose2d(BACKDROP_CENTER_VECTOR.plus(DIFF_VECTOR)/*.plus(new Vector2d(0, 2.7))*/, Math.toRadians(0)))
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                    scorer.take();
                })
                .waitSeconds(0.4)
                .setReversed(true)
                .splineToConstantHeading(RIGGING_UP_VECTOR, Math.toRadians(180))
                .splineToConstantHeading(RIGGING_DOWN_VECTOR, Math.toRadians(180))
                .splineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(-3.2, -12)), Math.toRadians(180))
                .waitSeconds(0.2)
                .build();

        intake_traj1 = drive.trajectorySequenceBuilder(traj1.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15,
                        DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(-3.2, 1)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.take())
                .build();

        traj2 = drive.trajectorySequenceBuilder(intake_traj1.end())
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intake.stop();
                })
                .lineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(5,0)))
                .splineToConstantHeading(RIGGING_DOWN_VECTOR.plus(new Vector2d(0, -0.2)), Math.toRadians(0))
                .splineToConstantHeading(RIGGING_UP_VECTOR.plus(new Vector2d(0, -0.2)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                    lift.setReference(570);
                })
                .splineToConstantHeading(BACKDROP_RIGHT_VECTOR.plus(new Vector2d(2.5,0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    scorer.take();

                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.setReference(0);
                })
                .waitSeconds(1.5)
                .lineToSplineHeading(new Pose2d(47, -60, Math.toRadians(90)))
                .build();

        drive.followTrajectorySequenceAsync(traj1);
    }

    private void move_right() {
        traj1 = drive.trajectorySequenceBuilder(NEAR_START_POSE)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    scorer.deployAuto();
                    lift.resetEncoders();
                })
                .waitSeconds(0.8)
                .lineToSplineHeading(new Pose2d(PURPLE_RIGHT_VECTOR, PURPLE_RIGHT_HEADING))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorer.closeLower();
                    scorer.openUpper();
                    scorer.deploy();
                })
                .waitSeconds(0.2)
                .lineToSplineHeading(new Pose2d(BACKDROP_RIGHT_VECTOR.plus(DIFF_VECTOR), Math.toRadians(0)))
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                    scorer.take();
                })
                .waitSeconds(0.4)
                .setReversed(true)
                .splineToConstantHeading(RIGGING_UP_VECTOR, Math.toRadians(180))
                .splineToConstantHeading(RIGGING_DOWN_VECTOR, Math.toRadians(180))
                .splineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(-3.2, -12)), Math.toRadians(180))
                .waitSeconds(0.2)
                .build();

        intake_traj1 = drive.trajectorySequenceBuilder(traj1.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15,
                        DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(-3.2, 1)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.take())
                .build();

        traj2 = drive.trajectorySequenceBuilder(intake_traj1.end())
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    intake.eject();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intake.stop();
                })
                .lineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(5,0)))
                .splineToConstantHeading(RIGGING_DOWN_VECTOR.plus(new Vector2d(0, -0.2)), Math.toRadians(0))
                .splineToConstantHeading(RIGGING_UP_VECTOR.plus(new Vector2d(0, -0.2)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                    lift.setReference(570);
                })
                .splineToConstantHeading(BACKDROP_LEFT_VECTOR.plus(new Vector2d(2.5,0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    scorer.take();

                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    lift.setReference(0);
                })
                .waitSeconds(1.5)
                .lineToSplineHeading(new Pose2d(47, -60, Math.toRadians(90)))
                .build();

        drive.followTrajectorySequenceAsync(traj1);
    }

    public void init_robot() {
        dashboard = FtcDashboard.getInstance();

        drive = new SampleMecanumDrive(this.hardwareMap);
        drive.setPoseEstimate(NEAR_START_POSE);
        PoseCache.pose = NEAR_START_POSE;
        intake = new Intake(this);
        lift = new Lift(this, dashboard);
        scorer = new Scorer(this, lift);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new PropDetectionPipeline(AllianceColor.RED);
        webcam.setPipeline(pipeline);
        telemetry.update();
    }
}
