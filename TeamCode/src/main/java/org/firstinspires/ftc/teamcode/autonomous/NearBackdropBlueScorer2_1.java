package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.BACKDROP_LEFT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.BACKDROP_RIGHT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.NEAR_START_POSE;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.PIXEL_STACK_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.PURPLE_CENTER_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.PURPLE_LEFT_HEADING;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.PURPLE_LEFT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.PURPLE_RIGHT_HEADING;
import static org.firstinspires.ftc.teamcode.autonomous.constants.BluePositionConstants.PURPLE_RIGHT_VECTOR;
import static org.firstinspires.ftc.teamcode.autonomous.constants.RedPositionConstants.BACKDROP_CENTER_VECTOR;

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
public class NearBackdropBlueScorer2_1 extends LinearOpMode {
    private FtcDashboard dashboard;
    private SampleMecanumDrive drive;
    private Intake intake;
    private Lift lift;
    private Scorer scorer;
    private Servo finger;
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

        scorer.close_lower();
        scorer.close_upper();
        scorer.deployAuto();
        lift.resetEncoders();

        while (opModeIsActive()) {
            lift.PIDControl();
            drive.update();
            telemetry.addData("Time", getRuntime());
            telemetry.update();
        }
    }

    private void move_left() {
        TrajectorySequence traj = drive.trajectorySequenceBuilder(NEAR_START_POSE)
                .lineToSplineHeading(new Pose2d(PURPLE_LEFT_VECTOR, PURPLE_LEFT_HEADING))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    finger.setPosition(0.1);
                })
                .lineToConstantHeading(PURPLE_LEFT_VECTOR.plus(new Vector2d(0, 6)))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    lift.setReference(300);
                    scorer.deploy();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    lift.setReference(0);
                })
                .lineToSplineHeading(new Pose2d(BACKDROP_LEFT_VECTOR, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.open_lower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    scorer.take();
                })
                .waitSeconds(1)
                .build();
        drive.followTrajectorySequenceAsync(traj);
    }

    private void move_center() {
        TrajectorySequence traj = drive.trajectorySequenceBuilder(NEAR_START_POSE)
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
                .build();
        drive.followTrajectorySequenceAsync(traj);
    }

    private void move_right() {
        TrajectorySequence traj = drive.trajectorySequenceBuilder(NEAR_START_POSE)
                .splineTo(PURPLE_RIGHT_VECTOR, PURPLE_RIGHT_HEADING)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    finger.setPosition(0.1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    lift.setReference(300);
                    scorer.deploy();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    lift.setReference(0);
                })
                .lineToSplineHeading(new Pose2d(BACKDROP_RIGHT_VECTOR, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    scorer.open_lower();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> {
                    scorer.take();
                })
                .waitSeconds(1)
                .build();
        drive.followTrajectorySequenceAsync(traj);
    }

    public void init_robot() {
        dashboard = FtcDashboard.getInstance();

        drive = new SampleMecanumDrive(this.hardwareMap);
        drive.setPoseEstimate(NEAR_START_POSE);
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
