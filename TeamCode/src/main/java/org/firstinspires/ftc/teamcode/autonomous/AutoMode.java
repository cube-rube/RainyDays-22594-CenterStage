package org.firstinspires.ftc.teamcode.autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.BasicDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.Scorer;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.modules.vision.PropDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "AutoMode")
public class AutoMode extends LinearOpMode {
    OpenCvWebcam camProp;
    PropDetectionPipeline propPipeline = new PropDetectionPipeline(AllianceColor.BLUE);
    PropDetectionPipeline.PropPosition propPosition = PropDetectionPipeline.PropPosition.LEFT;
    private BasicDrive basicDrive;
    private SampleMecanumDrive mecanumDrive;
    private Lift lift;
    private Intake intake;
    private Scorer scorer;
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

        telemetry.addData("Snapshot post-START analysis", propPosition);
        telemetry.update();

        lift.runtimeReset();
        runtime.reset();
        while (runtime.seconds() <= 2) {
            telemetry.addLine("Lift");
            telemetry.update();
            lift.moveToPos(Lift.MAX_POS);
        }
        telemetry.addLine("Parking");
        telemetry.update();
        parking();

        /*
        switch (propPosition) {
            case LEFT:
                leftPixelPush();
            case RIGHT:
                rightPixelPush();
            case CENTER:
                centerPixelPush();
        }
         */
        /*
        switch (STARTPOS) {
            case NOT_NEAR_BACKDROP:
                switch (ALLIANCECOLOR) {
                    case BLUE:
                    case RED:
                }
            case NEAR_BACKDROP:
                switch (ALLIANCECOLOR) {
                    case BLUE:
                    case RED:
                }
        }
        */
    }

    private void leftPixelPush() {
        basicDrive.encoderDriveY(BasicDrive.DRIVE_SPEED, 28, 28, 2); // ВПЕРЕД
        // СТРЕЙФ НАЛЕВО ДО ЦЕНТРА Spike Mark
        // ВПЕРЕД
        // ВЫПЛЕВЫВАЕМ ПИКСЕЛЬ
    }

    private void centerPixelPush() {
        // ВПЕРЕД
        // ВЫПЛЕВЫВАЕМ ПИКСЕЛЬ
    }

    private void rightPixelPush() {
        basicDrive.encoderDriveY(BasicDrive.DRIVE_SPEED, 28, 28, 3);
        // СТРЕЙФ НАПРАВО ДО ЦЕНТРА Spike Mark
        // ВПЕРЕД
        // ВЫПЛЕВЫВАЕМ ПИКСЕЛЬ
    }

    private void parking() {
        basicDrive.encoderDriveY(0, 0, 0, 1);
        telemetry.addLine("Still");
        telemetry.update();
        basicDrive.encoderDriveY(BasicDrive.DRIVE_SPEED, 35, 35, 3);
        basicDrive.encoderDriveY(BasicDrive.DRIVE_SPEED, 35, 35, 3);
        telemetry.addLine("Forward");
        telemetry.update();
        basicDrive.encoderDriveY(BasicDrive.DRIVE_SPEED, -2.05, -2.05,2);
        telemetry.addLine("Back");
        telemetry.update();
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
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(this, dashboard);
        intake = new Intake(this);
        scorer = new Scorer(this);

        telemetry.update();
    }
}
