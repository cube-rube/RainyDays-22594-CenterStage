package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.BACKDROP_CENTER_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.BACKDROP_LEFT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.BACKDROP_RIGHT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.DIFF_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.NEAR_START_POSE;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.PURPLE_CENTER_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.PURPLE_LEFT_HEADING;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.PURPLE_LEFT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.PURPLE_RIGHT_HEADING;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.PURPLE_RIGHT_VECTOR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Autonomous(name = "BlueNear2+0")
public class BlueNearScorer2_0 extends LinearOpMode {
    private FtcDashboard dashboard;
    private SampleMecanumDrive drive;
    private Intake intake;
    private Lift lift;
    private Scorer scorer;
    private OpenCvWebcam webcam;
    private PropDetectionPipeline pipeline;
    private PropDetectionPipeline.PropPosition position;
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

        intake.stop();
        while (opModeIsActive()) {
            lift.PIDControl();
            drive.update();
            intake.autoControl();

            PoseCache.pose = drive.getPoseEstimate();
            telemetry.addData("Time", getRuntime());
            telemetry.addData("heading", drive.getExternalHeading());
            telemetry.addData("raw_heading", drive.getRawExternalHeading());
            telemetry.addData("cache_heading", PoseCache.pose.getHeading());
            telemetry.update();
        }
    }

    private void move_left() {
        TrajectorySequence traj = drive.trajectorySequenceBuilder(NEAR_START_POSE)
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
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    scorer.deployAutoPush();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorer.deployAutoUp();
                    scorer.closeLower();
                    scorer.openUpper();
                })
                .waitSeconds(0.2)
                .lineToSplineHeading(new Pose2d(BACKDROP_LEFT_VECTOR.plus(new Vector2d(1, 2)), Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.deploy();
                })
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openLower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                    scorer.take();
                })
                .waitSeconds(0.4)
                .lineToSplineHeading(new Pose2d(47, 60, Math.toRadians(270)))
                .build();
        drive.followTrajectorySequenceAsync(traj);
    }

    private void move_center() {
        TrajectorySequence traj = drive.trajectorySequenceBuilder(NEAR_START_POSE)
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
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    scorer.deployAutoPush();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    scorer.deploy();
                })
                .waitSeconds(0.2)
                .lineToSplineHeading(new Pose2d(BACKDROP_CENTER_VECTOR.plus(new Vector2d(1, 1))/*.plus(new Vector2d(0, 2.7))*/, Math.toRadians(0)))
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                    scorer.take();
                })
                .waitSeconds(0.4)
                .lineToSplineHeading(new Pose2d(47, 60, Math.toRadians(270)))
                .build();
        drive.followTrajectorySequenceAsync(traj);
    }

    private void move_right() {
        TrajectorySequence traj = drive.trajectorySequenceBuilder(NEAR_START_POSE)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.closeLower();
                    scorer.closeUpper();
                    scorer.deployAuto();
                    lift.resetEncoders();
                })
                .waitSeconds(1.2)
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
                .lineToSplineHeading(new Pose2d(BACKDROP_RIGHT_VECTOR.minus(new Vector2d(-1.5, 3)), Math.toRadians(0)))
                .waitSeconds(0.05)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.openUpper();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> {
                    scorer.take();
                })
                .waitSeconds(0.4)
                .lineToSplineHeading(new Pose2d(47, 60, Math.toRadians(270)))
                .build();
        drive.followTrajectorySequenceAsync(traj);
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
        pipeline = new PropDetectionPipeline(AllianceColor.BLUE);
        webcam.setPipeline(pipeline);
        telemetry.update();
    }
}
