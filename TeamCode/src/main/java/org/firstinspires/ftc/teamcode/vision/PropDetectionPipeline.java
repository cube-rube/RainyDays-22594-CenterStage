package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.misc.GameConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PropDetectionPipeline extends OpenCvPipeline {
	public int PROP_HEIGHT = 0;
	private AllianceColor ALLIANCECOLOR;
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

    public PropDetectionPipeline(AllianceColor color) {
        this.ALLIANCECOLOR = color;
    }

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

        switch (ALLIANCECOLOR) {
            case RED:
                Core.extractChannel(leftCrop, leftCrop, 1);
                Core.extractChannel(centerCrop, centerCrop, 1);
                Core.extractChannel(rightCrop, rightCrop, 1);
            case BLUE:
                Core.extractChannel(leftCrop, leftCrop, 2);
                Core.extractChannel(centerCrop, centerCrop, 2);
                Core.extractChannel(rightCrop, rightCrop, 2);
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
