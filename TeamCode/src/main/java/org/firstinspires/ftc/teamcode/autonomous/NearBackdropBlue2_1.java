package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.BluePositionConstants.NEAR_START_POSE;
import static org.firstinspires.ftc.teamcode.autonomous.BluePositionConstants.PIXEL_STACK_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.BluePositionConstants.PURPLE_RIGHT_HEADING;
import static org.firstinspires.ftc.teamcode.autonomous.BluePositionConstants.RIGGING_DOWN_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.BluePositionConstants.RIGGING_UP_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.BluePositionConstants.BACKDROP_RIGHT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.BluePositionConstants.PURPLE_RIGHT_VECTOR;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

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

@Autonomous
public class NearBackdropBlue2_1 extends LinearOpMode {
    private FtcDashboard dashboard;
    private SampleMecanumDrive drive;
    private Intake intake;
    private Lift lift;
    private Scorer scorer;
    private Servo finger;
    private OpenCvWebcam webcam;
    private PropDetectionPipeline pipeline;
    private PropDetectionPipeline.PropPosition position;
    private final Pose2d startPose = new Pose2d(15, 63, Math.toRadians(270));
    private boolean sensor = false;
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

        position = pipeline.getPropPosition();
        telemetry.addData("prop_pos", position);
        telemetry.update();
        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
            }
        });

        scorer.close_lower();
        scorer.take();
        lift.resetEncoders();
        finger.setPosition(0.58);

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
    }

    private void move_left() {
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(20, 35, Math.toRadians(290)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // release pixel
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(20, 42))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    // lift up
                    // rotate scorer
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    // lift down
                })
                .lineToSplineHeading(new Pose2d(44, 42, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // release
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    // take
                })
                .waitSeconds(1)
                .build();
        drive.followTrajectorySequence(traj);
    }

    private void move_center() {
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(11.7, 31))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    finger.setPosition(0.1);
                })
                .waitSeconds(1)
                .lineTo(new Vector2d(11.7, 35))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    lift.moveToPos2(300);
                    scorer.deploy();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    lift.moveToPos2(0);
                })
                .lineToSplineHeading(new Pose2d(44, 35, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.open_lower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    scorer.take();
                })
                .waitSeconds(1)
                .build();
        drive.followTrajectorySequence(traj);
    }

    private void move_right() {
        TrajectorySequence traj = drive.trajectorySequenceBuilder(NEAR_START_POSE)
                .lineToSplineHeading(new Pose2d(PURPLE_RIGHT_VECTOR, PURPLE_RIGHT_HEADING)) // move to pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // release pixel
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    // lift up
                    // rotate scorer
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    // lift down
                })
                .lineToSplineHeading(new Pose2d(BACKDROP_RIGHT_VECTOR, Math.toRadians(0))) // move to backdrop
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // release
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    // take
                })
                .waitSeconds(1)
                .lineToConstantHeading(RIGGING_UP_VECTOR)
                .lineToConstantHeading(RIGGING_DOWN_VECTOR)
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    // intake
                })
                .lineToConstantHeading(PIXEL_STACK_VECTOR)
                .waitSeconds(1)
                .lineToConstantHeading(RIGGING_DOWN_VECTOR) // moving back to backdrop
                .lineToConstantHeading(RIGGING_UP_VECTOR)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    // lift up
                    // rotate scorer
                })
                .UNSTABLE_addTemporalMarkerOffset(1.1, () -> {
                    // lift down
                })
                .lineToConstantHeading(BACKDROP_RIGHT_VECTOR)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // release
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    // take
                })
                .waitSeconds(1)
                .build();
        drive.followTrajectorySequence(traj);
    }

    public void init_robot() {
        dashboard = FtcDashboard.getInstance();

        drive = new SampleMecanumDrive(this.hardwareMap);
        drive.setPoseEstimate(startPose);
        intake = new Intake(this);
        lift = new Lift(this, dashboard);
        scorer = new Scorer(this, lift);
        finger = hardwareMap.get(Servo.class, "servo_finger");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new PropDetectionPipeline(AllianceColor.BLUE);
        webcam.setPipeline(pipeline);
        telemetry.update();
    }
}
