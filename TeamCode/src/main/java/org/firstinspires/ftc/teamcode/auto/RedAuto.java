package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Red Auto", group= "Auto")
public class RedAuto extends LinearOpMode{

    static final double FEET_PER_METER = 3.28084;

    static final long MS_PER_INCH = 18;



    OpenCvCamera pixle_cam;
    OpenCvCamera april_cam;
    private DcMotor LEFTDRIVE;
    private DcMotor RIGHTDRIVE;
    private DcMotor LEFTAXLE;
    private DcMotor RIGHTAXLE;
    private Servo ELBOW1;
    private Servo ELBOW2;
    private Servo WRIST1;
    private Servo CLAWLEFT;
    private Servo CLAWRIGHT;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int BLUE_LEFT = 1;
    int BLUE_MIDDLE = 2;
    int BLUE_RIGHT = 3;
    int RED_LEFT = 4;
    int RED_MIDDLE = 5;
    int RED_RIGHT = 6;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException{
        LEFTDRIVE = hardwareMap.get(DcMotor.class, "LEFT DRIVE");
        RIGHTDRIVE = hardwareMap.get(DcMotor.class, "RIGHT DRIVE");
        LEFTAXLE = hardwareMap.get(DcMotor.class, "LEFT AXLE");
        RIGHTAXLE = hardwareMap.get(DcMotor.class, "RIGHT AXLE");
        ELBOW1 = hardwareMap.get(Servo.class, "ELBOW1");
        ELBOW2 = hardwareMap.get(Servo.class, "ELBOW2");
        WRIST1 = hardwareMap.get(Servo.class, "WRIST");
        CLAWLEFT = hardwareMap.get(Servo.class, "CLAW LEFT");
        CLAWRIGHT = hardwareMap.get(Servo.class, "CLAW RIGHT");

        LEFTDRIVE.setDirection(DcMotor.Direction.REVERSE);
        LEFTAXLE.setDirection(DcMotor.Direction.REVERSE);
        ELBOW2.setDirection(Servo.Direction.REVERSE);
        CLAWLEFT.setDirection(Servo.Direction.REVERSE);






        //set's up pixle detection
        int pixelId = hardwareMap.appContext.
                getResources().getIdentifier("pixelId",
                        "id", hardwareMap.appContext.getPackageName());
        pixle_cam = OpenCvCameraFactory.getInstance().
                createWebcam(hardwareMap.get(WebcamName.class, "pixle_cam"), pixelId);
        PixelDetector detector = new PixelDetector(telemetry);
        pixle_cam.setPipeline(detector);
        pixle_cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                pixle_cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        //sets up april tag detection
        int aprilId = hardwareMap.appContext.
                getResources().getIdentifier("aprilId",
                        "id", hardwareMap.appContext.getPackageName());
        april_cam = OpenCvCameraFactory.getInstance().
                createWebcam(hardwareMap.get(WebcamName.class, "april_cam"), aprilId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        april_cam.setPipeline(aprilTagDetectionPipeline);
        april_cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                april_cam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == BLUE_LEFT || tag.id == BLUE_MIDDLE || tag.id == BLUE_RIGHT ||
                            tag.id == RED_LEFT || tag.id == RED_MIDDLE || tag.id == RED_RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

       switch(detector.getLocation()){
           case LEFT:
               //move forward 3.5 feet
               forward(42);
               turnLeft(90);
               //move elbow 1 down
               elbowDegrees(0);
               //open left claw
               CLAWLEFT.setPosition(0);
               //move elbow up
               turnRight(90);
               forward(30);
               turnRight(90);
               forward(72);
               turnRight(90);
               forward(15);
               turnLeft(90);








           case CENTER:



           case RIGHT:




           case NOT_FOUND:
       }


    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        //telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        //telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void turnLeft(double angle){

        // Calculate the number of movements needed to reach the target angle
        double movementsNeeded = Math.abs(angle / 90);

        // Calculate the adjusted sleep time based on the target angle
        double adjustedSleepTime = 650 * movementsNeeded;
        LEFTDRIVE.setPower(-0.5);
        RIGHTAXLE.setPower(0.5);
        sleep((long) adjustedSleepTime);
        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
        sleep(500);
    }
    void turnRight(double angle){

        // Calculate the number of movements needed to reach the target angle
        double movementsNeeded = Math.abs(angle / 90);

        // Calculate the adjusted sleep time based on the target angle
        double adjustedSleepTime = 650 * movementsNeeded;
        LEFTDRIVE.setPower(0.5);
        RIGHTAXLE.setPower(-0.5);
        sleep((long) adjustedSleepTime);
        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
        sleep(500);
    }
    void forward(double inches){
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 55);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000 * movementsNeeded;

        // Set power to the motors for moving forward
        LEFTDRIVE.setPower(0.5);
        RIGHTDRIVE.setPower(0.5);

        // Sleep for the adjusted time
        sleep((long) adjustedSleepTime);

        // Stop the motors after the sleep
        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
        sleep(500);
    }

    void backward(double inches){
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 55);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000 * movementsNeeded;

        // Set power to the motors for moving backward
        LEFTDRIVE.setPower(-0.5);
        RIGHTDRIVE.setPower(-0.5);

        // Sleep for the adjusted time
        sleep((long) adjustedSleepTime);

        // Stop the motors after the sleep
        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
        sleep(500);
    }
    private double degreesToPosition(double degrees) {
        // Assuming your servo has a range of motion of 180 degrees
        double minDegrees = 0.0;
        double maxDegrees = 180.0;

        // Convert degrees to the servo position scale (0 to 1)
        return (degrees - minDegrees) / (maxDegrees - minDegrees);
    }
    private void elbowDegrees(double degrees) {
        double position = degreesToPosition(degrees);

        ELBOW1.setPosition(position);
        ELBOW2.setPosition(position);

        sleep(1000);
    }

    public void clawsDegrees(double degrees){
        double position = degreesToPosition(degrees);

        CLAWLEFT.setPosition(position);
        CLAWRIGHT.setPosition(position);

        sleep(1000);

    }


}
