package org.firstinspires.ftc.teamcode.autonomous.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.Scorer;
import org.firstinspires.ftc.teamcode.modules.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.modules.vision.PropDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Disabled
public class NearBackdropBlueAutoScorerOld extends LinearOpMode {
    private FtcDashboard dashboard;
    private SampleMecanumDrive drive;
    private Intake intake;
    private Lift lift;
    private Scorer scorer;
    private OpenCvWebcam webcam;
    private PropDetectionPipeline pipeline;
    private PropDetectionPipeline.PropPosition position;
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
        
        Trajectory traj = null;

        scorer.closeLower();
        scorer.take();
        lift.resetEncoders();

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
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(24, 8), Math.toRadians(25))
                .build();
        drive.followTrajectory(traj);
        intake.setPower(0.25);
        sleep(700);
        intake.setPower(0);

        Trajectory traj1 = drive.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(26, 36), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj1);
        lift.moveToPos(400);
        scorer.deploy();
        sleep(600);
        lift.moveToPos(0);

        Trajectory trajStrafe = drive.trajectoryBuilder(traj1.end(), false)
                .lineTo(new Vector2d(20, 38))
                .build();
        drive.followTrajectory(trajStrafe);
        sleep(500);
        scorer.openLower();
        sleep(500);
        lift.moveToPos(300);
        scorer.take();
        sleep(1500);
        lift.moveToPos(0);
        sleep(400);

        /*
        Trajectory tra = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(2, 36), Math.toRadians(180))
                .build();
        drive.followTrajectory(tra);

         */
    }

    private void move_center() {
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(26, 1.5))
                .build();
        drive.followTrajectory(traj);
        intake.setPower(0.25);
        sleep(700);
        intake.setPower(0);

        Trajectory traj1 = drive.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(26.6, 33.5), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj1);
        lift.moveToPos(400);
        scorer.deploy();
        sleep(600);
        lift.moveToPos(0);

        sleep(500);
        scorer.openLower();
        sleep(500);
        lift.moveToPos(300);
        scorer.take();
        sleep(1500);
        lift.moveToPos(0);
        sleep(400);
        /*
        Trajectory tra = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(2, 33.5), Math.toRadians(180))
                .build();
        drive.followTrajectory(tra);

         */
    }

    private void move_right() {
        Trajectory traj = drive.trajectoryBuilder(new Pose2d(), false)
                .forward(15)
                .splineTo(new Vector2d(25, -5.2), 5.58)
                .build();
        drive.followTrajectory(traj);
        intake.setPower(0.25);
        sleep(700);
        intake.setPower(0);

        Trajectory traj1 = drive.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(28, 33), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj1);
        lift.moveToPos(400);
        scorer.deploy();
        sleep(600);
        lift.moveToPos(0);

        Trajectory trajStrafe = drive.trajectoryBuilder(traj1.end(), false)
                .lineTo(new Vector2d(34.4, 36))
                .build();
        drive.followTrajectory(trajStrafe);
        sleep(500);
        scorer.openLower();
        sleep(500);
        lift.moveToPos(300);
        scorer.take();
        sleep(1500);
        lift.moveToPos(0);
        sleep(400);

        /*
        Trajectory tra = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(5, 33), Math.toRadians(180))
                .build();
        drive.followTrajectory(tra);

         */
    }

    public void init_robot() {
        dashboard = FtcDashboard.getInstance();
        drive = new SampleMecanumDrive(this.hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
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
