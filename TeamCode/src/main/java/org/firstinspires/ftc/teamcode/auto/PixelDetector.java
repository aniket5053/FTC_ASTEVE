package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


@Autonomous(name="Test", group= "Auto")
public class PixelDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        CENTER,
        RIGHT,
        NOT_FOUND
    }
    private Location location;
    static final Rect Left_ROI = new Rect(
            new Point(20, 0),
            new Point(100, 200));
    static final Rect Center_ROI = new Rect(
            new Point(120, 0),
            new Point(200, 200));
    static final Rect Right_ROI = new Rect(
            new Point(220, 0),
            new Point(300, 200));
    static double PERCENT_COLOR_THRESHOLD = 0.4;
    public PixelDetector(Telemetry t) {telemetry = t;}

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Color
        Scalar lowHSV = new Scalar(0,0,168);
        Scalar highHSV = new Scalar(172,111,255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(Left_ROI);
        Mat center = mat.submat(Center_ROI);
        Mat right = mat.submat(Right_ROI);

        double leftValue = Core.sumElems(left).val[0]/Left_ROI.area()/255;
        double centerValue = Core.sumElems(center).val[0]/Center_ROI.area()/255;
        double rightValue = Core.sumElems(right).val[0]/Right_ROI.area()/255;

        left.release();
        center.release();
        right.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Center raw value", (int) Core.sumElems(center).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean pixelLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean pixelCenter = centerValue > PERCENT_COLOR_THRESHOLD;
        boolean pixelRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if ((pixelLeft && pixelCenter) || (pixelLeft && pixelRight) || (pixelRight && pixelCenter))
        {
            //not found
            location = Location.NOT_FOUND;
            telemetry.addData("Pixel Location", "not found");
        }
        else if (pixelLeft){
            //left
            location = Location.LEFT;
            telemetry.addData("Pixel Location", "left");

        }
        else if(pixelCenter){
            //center
            location = Location.CENTER;
            telemetry.addData("Pixel Location", "center");

        }
        else if(pixelRight){
            //right
            location = Location.RIGHT;
            telemetry.addData("Pixel Location", "right");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);

        Scalar leftZone = new Scalar(255,0,0);
        Scalar centerZone = new Scalar(0,255,0);
        Scalar rightZone = new Scalar(0,0,255);

        Imgproc.rectangle(mat, Left_ROI, location == Location.LEFT? leftZone:centerZone);
        Imgproc.rectangle(mat, Left_ROI, location == Location.LEFT? leftZone:rightZone);

        Imgproc.rectangle(mat, Center_ROI, location == Location.CENTER? leftZone:centerZone);
        Imgproc.rectangle(mat, Center_ROI, location == Location.CENTER? rightZone:centerZone);

        Imgproc.rectangle(mat, Right_ROI, location == Location.RIGHT? leftZone:rightZone);
        Imgproc.rectangle(mat, Right_ROI, location == Location.RIGHT? rightZone:centerZone);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
