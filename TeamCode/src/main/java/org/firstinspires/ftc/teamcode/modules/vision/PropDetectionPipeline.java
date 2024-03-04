package org.firstinspires.ftc.teamcode.modules.vision;

import org.firstinspires.ftc.teamcode.misc.GameConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropDetectionPipeline extends OpenCvPipeline {
    Mat ybcrcb = new Mat();
    Mat leftCrop, centerCrop, rightCrop;
    double avgLeft, avgCenter, avgRight;
    Mat output = new Mat();
	
    Scalar rectColorLeft = new Scalar(255.0, 0.0, 0.0);
    Scalar rectColorCenter = new Scalar(255.0, 0.0, 0.0);
    Scalar rectColorRight = new Scalar(255.0, 0.0, 0.0);
    public enum PropPosition {
        LEFT,
        CENTER,
        RIGHT
    }
    public volatile PropPosition position;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, ybcrcb, Imgproc.COLOR_RGB2YCrCb);

        Rect leftRect = null;
        Rect centerRect = null;
        Rect rightRect = null;

        switch (GameConstants.ALLIANCE_COLOR) {
            case RED:
                leftRect = new Rect(50, 140, 120, 120);
                centerRect = new Rect(260, 120, 100, 100);
                rightRect = new Rect(480, 140, 120, 120);
                break;
            case BLUE:
                switch (GameConstants.STARTPOS) {
                    case NEAR:
                        leftRect = new Rect(90, 140, 120, 80);
                        centerRect = new Rect(280, 140, 100, 80);
                        rightRect = new Rect(480, 140, 120, 80);
                        break;
                    case FAR:
                        leftRect = new Rect(0, 140, 100, 70);
                        centerRect = new Rect(220, 140, 60, 60);
                        rightRect = new Rect(400, 140, 90, 70);
                        break;
                }
                break;
        }

        input.copyTo(output);

        leftCrop = ybcrcb.submat(leftRect);
        centerCrop = ybcrcb.submat(centerRect);
        rightCrop = ybcrcb.submat(rightRect);

        switch (GameConstants.ALLIANCE_COLOR) {
            case RED:
                Core.extractChannel(leftCrop, leftCrop, 1);
                Core.extractChannel(centerCrop, centerCrop, 1);
                Core.extractChannel(rightCrop, rightCrop, 1);
                break;
            case BLUE:
                Core.extractChannel(leftCrop, leftCrop, 2);
                Core.extractChannel(centerCrop, centerCrop, 2);
                Core.extractChannel(rightCrop, rightCrop, 2);
                break;
        }

        avgLeft = Core.mean(leftCrop).val[0];
        avgCenter = Core.mean(centerCrop).val[0];
        avgRight = Core.mean(rightCrop).val[0];
		
        double mx = Math.max(avgLeft, Math.max(avgCenter, avgRight));

        if (mx == avgLeft) {
            rectColorLeft = new Scalar(0.0, 255.0, 0.0);
            rectColorCenter = new Scalar(255.0, 0.0, 0.0);
            rectColorRight = new Scalar(255.0, 0.0, 0.0);
            position = PropPosition.LEFT;
        }
        else if (mx == avgCenter) {
            rectColorLeft = new Scalar(255.0, 0.0, 0.0);
            rectColorCenter = new Scalar(0.0, 255.0, 0.0);
            rectColorRight = new Scalar(255.0, 0.0, 0.0);
            position = PropPosition.CENTER;
        }
        else if (mx == avgRight) {
            rectColorLeft = new Scalar(255.0, 0.0, 0.0);
            rectColorCenter = new Scalar(255.0, 0.0, 0.0);
            rectColorRight = new Scalar(0.0, 255.0, 0.0);
            position = PropPosition.RIGHT;
        }
		
       
        Imgproc.rectangle(output, leftRect, rectColorLeft, 2);
        Imgproc.rectangle(output, centerRect, rectColorCenter, 2);
        Imgproc.rectangle(output, rightRect, rectColorRight, 2);

        return output;
    }

    public PropPosition getPropPosition() {
        return position;
    }
}
