package org.firstinspires.ftc.teamcode.autonomous;

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
public class NearBackdropRedAuto extends LinearOpMode {
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
                telemetry.addLine("camera doesn't turn on");
            }
        });

        waitForStart();

        position = pipeline.getPropPosition();
        telemetry.addData("prop_pos", position);
        telemetry.update();

        Trajectory traj = null;

        switch (position) {
            case RIGHT:
                traj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(24, -3.5), 5.58)
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
                traj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(24, 4.5), -5.58)
                        .build();
                drive.followTrajectory(traj);
                break;
        }
        intake.setPower(0.22);
        sleep(700);
        intake.setPower(0);
        Trajectory traj1 = drive.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(25.5, -35), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj1);
    }

    public void init_robot() {
        dashboard = FtcDashboard.getInstance();
        drive = new SampleMecanumDrive(this.hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        intake = new Intake(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new PropDetectionPipeline(AllianceColor.RED);
        webcam.setPipeline(pipeline);
        telemetry.update();
    }
}
