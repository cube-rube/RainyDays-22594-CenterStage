package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Autonomous extends LinearOpMode {
    OpenCvWebcam camProp;
    PropDetectionPipeline propPipeline = new PropDetectionPipeline();
    int propPos;

    @Override
    public void runOpMode() throws InterruptedException {
        initCamera();

        waitForStart();

        camProp.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                propPos = propPipeline.getPropPosition();
            }
            @Override
            public void onError(int errorCode)
            {
            }
        });
        


    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camProp = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Camera1"), cameraMonitorViewId);

        camProp.setPipeline(propPipeline);
    }
}
