package org.firstinspires.ftc.teamcode.autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.BasicDrive;
import org.firstinspires.ftc.teamcode.drive.BasicDriveNoEncoders;
import org.firstinspires.ftc.teamcode.modules.Deploy;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.vision.PropDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "AutoMode")
public class AutoMode extends LinearOpMode {
    OpenCvWebcam camProp;
    PropDetectionPipeline propPipeline = new PropDetectionPipeline();
    int propPos;
    private BasicDriveNoEncoders basicDrive;
    private Lift lift;
    private Intake intake;
    private Deploy deploy;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initCamera();
        initRobot();

        waitForStart();

        camProp.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                propPos = propPipeline.getPropPosition();
            }
            @Override
            public void onError(int errorCode) {
            }
        });
        if (propPos == 0) {

        }
        else if (propPos == 1) {

        }
        else if (propPos == 2) {

        }
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camProp = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Camera1"), cameraMonitorViewId);

        camProp.setPipeline(propPipeline);

        telemetry.addData("Camera: ", "Initialized");
    }

    private void initRobot() {
        basicDrive = new BasicDriveNoEncoders(this, runtime);
        lift = new Lift(this);
        intake = new Intake(this);
        deploy = new Deploy(this);

        telemetry.update();
    }
}
