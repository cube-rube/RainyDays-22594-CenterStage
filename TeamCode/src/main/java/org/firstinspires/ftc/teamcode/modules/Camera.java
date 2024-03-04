package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.vision.AllianceColor;
import org.firstinspires.ftc.teamcode.modules.vision.PropDetectionPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
    private final Telemetry telemetry;
    public final OpenCvWebcam webcam;
    public final PropDetectionPipeline pipeline;

    public Camera(LinearOpMode linearOpMode) {
        telemetry = linearOpMode.telemetry;
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new PropDetectionPipeline();
        webcam.setPipeline(pipeline);
        telemetry.addLine("Camera initialized");
    }

    public PropDetectionPipeline.PropPosition getPropPosition() {
        return pipeline.getPropPosition();
    }
}
