package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="BLUEPROPDETECTION")
public class CameraBluePropDetection extends OpMode {
    OpenCvWebcam camera = null;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Camera1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.setPipeline(new PropDetectionPipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void loop() {

    }

    class PropDetectionPipeline extends OpenCvPipeline {
        Mat ybcrcb = new Mat();
        Mat leftCrop, centerCrop, rightCrop;
        double avgLeftFin, avgCenterFin, avgRightFin;
        Mat output = new Mat();
        Scalar rectColorLeft = new Scalar(255.0, 0.0, 0.0);
        Scalar rectColorCenter = new Scalar(0.0, 255.0, 0.0);
        Scalar rectColorRight = new Scalar(0.0, 0.0, 255.0);

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, ybcrcb, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect leftRect = new Rect(1, 1, 212, 359);
            Rect centerRect = new Rect(213, 1, 212, 359);
            Rect rightRect = new Rect(426, 1, 213, 359);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColorLeft, 1);
            Imgproc.rectangle(output, centerRect, rectColorCenter, 1);
            Imgproc.rectangle(output, rightRect, rectColorRight, 1);

            leftCrop = ybcrcb.submat(leftRect);
            centerCrop = ybcrcb.submat(centerRect);
            rightCrop = ybcrcb.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(centerCrop, centerCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar avgLeft = Core.mean(leftCrop);
            Scalar avgCenter = Core.mean(centerCrop);
            Scalar avgRight = Core.mean(rightCrop);

            avgLeftFin = avgLeft.val[0];
            avgCenterFin = avgCenter.val[0];
            avgRightFin = avgRight.val[0];

            double mx = Math.max(avgLeftFin, Math.max(avgCenterFin, avgRightFin));

            if (mx == avgLeftFin) {
                telemetry.addLine("LEFT");
            }
            else if (mx == avgCenterFin) {
                telemetry.addLine("CENTER");
            }
            else if (mx == avgRightFin) {
                telemetry.addLine("RIGHT");
            }

            return output;
        }
    }
}
