package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropDetectionPipeline extends OpenCvPipeline {
	public int TEAM = 1;
	public int PROP_HEIGHT = 0;
	
    Mat ybcrcb = new Mat();
    Mat leftCrop, centerCrop, rightCrop;
    double avgLeft, avgCenter, avgRight;
    Mat output = new Mat();
	
    Scalar rectColorLeft = new Scalar(255.0, 0.0, 0.0);
    Scalar rectColorCenter = new Scalar(255.0, 0.0, 0.0);
    Scalar rectColorRight = new Scalar(255.0, 0.0, 0.0);
    int propPosition;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, ybcrcb, Imgproc.COLOR_RGB2YCrCb);

        Rect leftRect = new Rect(0, 120 + PROP_HEIGHT, 80, 100);
        Rect centerRect = new Rect(280, 115 + PROP_HEIGHT, 80, 80);
        Rect rightRect = new Rect(560, 120 + PROP_HEIGHT, 80, 100);

        input.copyTo(output);

        leftCrop = ybcrcb.submat(leftRect);
        centerCrop = ybcrcb.submat(centerRect);
        rightCrop = ybcrcb.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop, TEAM);
        Core.extractChannel(centerCrop, centerCrop, TEAM);
        Core.extractChannel(rightCrop, rightCrop, TEAM);

        avgLeft = Core.mean(leftCrop).val[0];
        avgCenter = Core.mean(centerCrop).val[0];
        avgRight = Core.mean(rightCrop).val[0];
		
        double mx = Math.max(avgLeft, Math.max(avgCenter, avgRight));

        if (mx == avgLeft) {
            rectColorLeft = new Scalar(0.0, 255.0, 0.0);
            rectColorCenter = new Scalar(255.0, 0.0, 0.0);
            rectColorRight = new Scalar(255.0, 0.0, 0.0);
            propPosition = 0;
        }
        else if (mx == avgCenter) {
            rectColorLeft = new Scalar(255.0, 0.0, 0.0);
            rectColorCenter = new Scalar(0.0, 255.0, 0.0);
            rectColorRight = new Scalar(255.0, 0.0, 0.0);
            propPosition = 1;
        }
        else if (mx == avgRight) {
            rectColorLeft = new Scalar(255.0, 0.0, 0.0);
            rectColorCenter = new Scalar(255.0, 0.0, 0.0);
            rectColorRight = new Scalar(0.0, 255.0, 0.0);
            propPosition = 2;
        }
		
       
        Imgproc.rectangle(output, leftRect, rectColorLeft, 2);
        Imgproc.rectangle(output, centerRect, rectColorCenter, 2);
        Imgproc.rectangle(output, rightRect, rectColorRight, 2);

        return output;
    }

    public int getPropPosition() {
        return propPosition;
    }
}
