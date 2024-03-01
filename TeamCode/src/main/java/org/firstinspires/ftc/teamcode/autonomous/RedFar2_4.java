package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.BACKDROP_CENTER_COORDS;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.BACKDROP_LEFT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.BACKDROP_RIGHT_COORDS;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.BACKDROP_RIGHT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.DIFF_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.DOOR_DOWN_COORDS;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.DOOR_UP_COORDS;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.END;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.FAR_START_COORDS;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.NEAR_START_POSE;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.FIRST_PIXEL_STACK_COORDS;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.PIXEL_STACK_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.PURPLE_CENTER_FAR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.PURPLE_LEFT_FAR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.PURPLE_LEFT_NEAR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.PURPLE_RIGHT_HEADING;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.PURPLE_RIGHT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.RIGGING_DOWN_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.RIGGING_UP_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.SECOND_PIXEL_STACK_BROKE;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.SECOND_PIXEL_STACK_COORDS;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.START_HEADING;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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

@Autonomous(name = "RedFar2+4")
public class RedFar2_4 extends LinearOpMode {
    private FtcDashboard dashboard;
    private SampleMecanumDrive drive;
    private Intake intake;
    private Lift lift;
    private Scorer scorer;
    private OpenCvWebcam webcam;
    private PropDetectionPipeline pipeline;
    private PropDetectionPipeline.PropPosition position;
    private TrajectorySequence traj1, intake_traj1, traj2;

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

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            PoseCache.pose = drive.getPoseEstimate();
            lift.PIDControl();
            intake.autoControl();
            drive.update();

            telemetry.addData("Time", getRuntime());
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }

    private void move_left() {
        traj1 = drive.trajectorySequenceBuilder(new Pose2d(FAR_START_COORDS[0], FAR_START_COORDS[1], START_HEADING))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    scorer.deployAuto();
                    lift.resetEncoders();
                })
                .waitSeconds(0.8)
                .lineTo(new Vector2d(PURPLE_LEFT_FAR[0], PURPLE_LEFT_FAR[1]))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    scorer.deployAutoPush();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    scorer.take();
                })
                .waitSeconds(0.2)

                // going to stack
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //intake.openRightFlap();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    intake.take();
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
                    scorer.closeLower();
                    scorer.closeUpper();
                    intake.eject();
                    intake.closeRightFlap();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intake.stop();
                })
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1]), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                    lift.setReference(570);
                })
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1]), Math.toRadians(0))

                // deploying white pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .waitSeconds(0.2)
                // deploying yellow pixel
                .lineTo(new Vector2d(BACKDROP_CENTER_COORDS[0], BACKDROP_CENTER_COORDS[1]))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorer.take();
                    lift.setReference(0);
                })
                .waitSeconds(0.4)

                // going to stack
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1]), Math.toRadians(180))
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.take();
                })
                .splineToConstantHeading(new Vector2d(SECOND_PIXEL_STACK_COORDS[0], SECOND_PIXEL_STACK_COORDS[1]), Math.toRadians(250))
                .waitSeconds(0.8)

                // going to backdrop
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .setReversed(false)
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
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                    lift.setReference(570);
                })
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1]), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorer.take();
                    lift.setReference(0);
                })
                .waitSeconds(0.3)

                // going to stack
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1]), Math.toRadians(180))
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.take();
                })
                .splineToConstantHeading(new Vector2d(SECOND_PIXEL_STACK_COORDS[0], SECOND_PIXEL_STACK_COORDS[1]), Math.toRadians(250))
                .setReversed(false)
                .waitSeconds(0.8)

                // going to backdrop
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .setReversed(false)
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
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                    lift.setReference(570);
                })
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1]), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorer.take();
                    lift.setReference(0);
                })
                .waitSeconds(0.3)

                .lineToSplineHeading(new Pose2d(END[0], END[1], Math.toRadians(END[2])))

                .build();

        drive.followTrajectorySequenceAsync(traj1);
    }

    private void move_center() {
        traj1 = drive.trajectorySequenceBuilder(new Pose2d(FAR_START_COORDS[0], FAR_START_COORDS[1], START_HEADING))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    scorer.deployAuto();
                    lift.resetEncoders();
                })
                .waitSeconds(0.8)
                .lineTo(new Vector2d(PURPLE_CENTER_FAR[0], PURPLE_CENTER_FAR[1]))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    scorer.deployAutoPush();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    scorer.take();
                })
                .waitSeconds(0.8)

                // going to stack
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //intake.openRightFlap();
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    intake.take();
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(SECOND_PIXEL_STACK_COORDS[0],
                        SECOND_PIXEL_STACK_COORDS[1],
                        Math.toRadians(0)), Math.toRadians(100))
                .splineToConstantHeading(new Vector2d(SECOND_PIXEL_STACK_COORDS[0] - 1.5,
                        SECOND_PIXEL_STACK_COORDS[1]), Math.toRadians(0))
                .waitSeconds(0.8)
                .setReversed(false)

                // going to backdrop
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    intake.eject();
                    intake.closeRightFlap();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    intake.stop();
                })
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1]), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                })
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1]), Math.toRadians(0))

                // deploying white pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .waitSeconds(0.2)
                // deploying yellow pixel
                .lineTo(new Vector2d(BACKDROP_CENTER_COORDS[0], BACKDROP_CENTER_COORDS[1]))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorer.take();
                    lift.setReference(0);
                })
                .waitSeconds(0.4)

                // going to stack
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1]), Math.toRadians(180))
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.take();
                })
                .splineToConstantHeading(new Vector2d(SECOND_PIXEL_STACK_COORDS[0] - 1.5, SECOND_PIXEL_STACK_COORDS[1] + SECOND_PIXEL_STACK_BROKE), Math.toRadians(250))
                .waitSeconds(0.8)

                // going to backdrop
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .setReversed(false)
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
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                    lift.setReference(570);
                })
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1]), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorer.take();
                    lift.setReference(0);
                })
                .waitSeconds(0.3)

                // going to stack
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(DOOR_UP_COORDS[0], DOOR_UP_COORDS[1]), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(DOOR_DOWN_COORDS[0], DOOR_DOWN_COORDS[1]), Math.toRadians(180))
                .resetVelConstraint()
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    intake.take();
                })
                .splineToConstantHeading(new Vector2d(SECOND_PIXEL_STACK_COORDS[0] - 1.5, SECOND_PIXEL_STACK_COORDS[1]), Math.toRadians(250))
                .setReversed(false)
                .waitSeconds(0.8)

                // going to backdrop
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, 3.5, 6.5))
                .setReversed(false)
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
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                    lift.setReference(570);
                })
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(BACKDROP_RIGHT_COORDS[0], BACKDROP_RIGHT_COORDS[1]), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorer.take();
                    lift.setReference(0);
                })
                .waitSeconds(0.3)

                .lineToSplineHeading(new Pose2d(END[0], END[1], Math.toRadians(END[2])))

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
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    scorer.deployAutoPush();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorer.deploy();
                })
                .waitSeconds(0.2)
                .lineToSplineHeading(new Pose2d(BACKDROP_RIGHT_VECTOR.plus(DIFF_VECTOR), Math.toRadians(0)))
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
                .splineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(-2.5, -0.5)), Math.toRadians(180))
                .waitSeconds(0.2)
                .build();

        intake_traj1 = drive.trajectorySequenceBuilder(traj1.end())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15,
                        DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> intake.take())
                .lineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(8, -0.5)))
                .lineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(-2.5, -0.5)))
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
                .lineToConstantHeading(PIXEL_STACK_VECTOR.plus(new Vector2d(5,-0.5)))
                .splineToConstantHeading(RIGGING_DOWN_VECTOR.plus(new Vector2d(0, -0.6)), Math.toRadians(0))
                .splineToConstantHeading(RIGGING_UP_VECTOR.plus(new Vector2d(0, -0.5)), Math.toRadians(0))
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
        Pose2d startPose = new Pose2d(FAR_START_COORDS[0], FAR_START_COORDS[1], START_HEADING);
        drive.setPoseEstimate(startPose);
        PoseCache.pose = startPose;
        intake = new Intake(this);
        lift = new Lift(this, dashboard);
        scorer = new Scorer(this, lift);

        GameConstants.ALLIANCE_COLOR = AllianceColor.RED;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new PropDetectionPipeline(AllianceColor.RED);
        webcam.setPipeline(pipeline);
        telemetry.update();
    }
}
