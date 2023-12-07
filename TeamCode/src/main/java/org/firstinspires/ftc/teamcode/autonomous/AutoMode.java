package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.BasicDrive;
import org.firstinspires.ftc.teamcode.modules.Deploy;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "AutoMode")
public class AutoMode extends LinearOpMode {
    OpenCvWebcam camProp;
    PropDetectionPipeline propPipeline = new PropDetectionPipeline();
    PropDetectionPipeline.PropPosition propPosition = PropDetectionPipeline.PropPosition.LEFT;
    private BasicDrive basicDrive;
    private Lift lift;
    private Intake intake;
    private Deploy deploy;
    private ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        initCamera();

        camProp.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                camProp.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

        waitForStart();
        propPosition = propPipeline.getPropPosition();

        telemetry.addData("Snapshot post-START analysis", propPipeline);
        telemetry.update();

        switch (propPosition) {
            case LEFT:
                leftPixelPush();
            case RIGHT:
                rightPixelPush();
            case CENTER:
                centerPixelPush();
        }
    }

    private void leftPixelPush() {

    }

    private void centerPixelPush() {

    }

    private void rightPixelPush() {

    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camProp = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Camera1"), cameraMonitorViewId);

        camProp.setPipeline(propPipeline);
        camProp.setMillisecondsPermissionTimeout(5000);

        telemetry.addData("Camera: ", "Initialized");
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        basicDrive = new BasicDrive(this, dashboard);
        lift = new Lift(this, dashboard);
        intake = new Intake(this);
        deploy = new Deploy(this);

        telemetry.update();
    }
}
