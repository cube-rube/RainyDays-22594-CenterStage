package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
public class AutoMode extends LinearOpMode {
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



        waitForStart();

        position = PropDetectionPipeline.PropPosition.CENTER;
        telemetry.addData("prop_pos", position);
        telemetry.update();

        Trajectory traj = null;

        scorer.close_lower();
        scorer.take();

        switch (position) {
            case RIGHT:
                traj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(24, -4.5), 5.58)
                        .build();
                drive.followTrajectory(traj);
                break;
            case CENTER:
                traj = drive.trajectoryBuilder(new Pose2d())
                        .forward(27.5)
                        .build();
                drive.followTrajectory(traj);
                break;
            case LEFT:
                traj = drive.trajectoryBuilder(new Pose2d(), false)
                        .splineTo(new Vector2d(24, 5.5), -5.58)
                        .build();
                drive.followTrajectory(traj);
                break;
        }
        intake.setPower(0.25);
        sleep(700);
        sensor = true;
        intake.setPower(0);
        Trajectory traj1 = drive.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(26, -36.8), Math.toRadians(-90))
                .build();
        drive.followTrajectory(traj1);
        lift.moveToPos(400);
        scorer.deploy();
        sleep(600);
        lift.moveToPos(0);
        Trajectory trajStrafe = null;
        switch (position) {
            case LEFT:
                trajStrafe = drive.trajectoryBuilder(traj1.end(), false)
                        .lineTo(new Vector2d(21.4, -36.8))
                        .build();
                drive.followTrajectory(trajStrafe);
                break;
            case CENTER:
                break;
            case RIGHT:
                trajStrafe = drive.trajectoryBuilder(traj1.end(), false)
                        .strafeRight(5.6)
                        .build();
                drive.followTrajectory(trajStrafe);
                sleep(200);
                break;
        }
        sleep(500);
        scorer.open_lower();
        sleep(500);
        lift.moveToPos(300);
        scorer.take();
        sleep(2000);
        lift.moveToPos(0);
        sleep(1000);
    }

    public void init_robot() {
        dashboard = FtcDashboard.getInstance();
        drive = new SampleMecanumDrive(this.hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        intake = new Intake(this);
        lift = new Lift(this, dashboard);
        scorer = new Scorer(this, lift);

        telemetry.update();
    }
}
