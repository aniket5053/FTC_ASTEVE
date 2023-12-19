package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="APRIL AUTO", group= "Auto")
public class TESTFORAPRILAUTO extends LinearOpMode{

    static final double FEET_PER_METER = 3.28084;

    static final long MS_PER_INCH = 18;



    OpenCvCamera pixle_cam;
    OpenCvCamera april_cam;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor leftElevator;
    private DcMotor rightElevator;
    private Servo leftElbow;
    private Servo rightElbow;
    private Servo leftWrist;
    private Servo leftClaw;
    private Servo rightClaw;
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
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FrtLtMtr");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BckLtMtr");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FrtRtMtr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BckRtMtr");
        // TA TODO: Configure HW so that names match
        DcMotor leftElevator   = hardwareMap.get(DcMotor.class, "LtElevator");
        DcMotor rightElevator  = hardwareMap.get(DcMotor.class, "RtElevator");
        Servo   leftElbow  = hardwareMap.get(Servo.class, "LtElbow");
        Servo   rightElbow = hardwareMap.get(Servo.class, "RtElbow");
        Servo   leftWrist  = hardwareMap.get(Servo.class, "LtWrist");
//        Servo   rightWrist = hardwareMap.get(Servo.class, "RtWrist");
        Servo   leftClaw   = hardwareMap.get(Servo.class, "LtClaw");
        Servo   rightClaw  = hardwareMap.get(Servo.class, "RtClaw");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        // TA TODO: test out directions - esp Elevator - it was different in teleop and Auto
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftElevator.setDirection(DcMotor.Direction.REVERSE);
        rightElbow.setDirection(Servo.Direction.REVERSE);
        leftClaw.setDirection(Servo.Direction.REVERSE);





        //set's up pixle detection
        int pixelId = hardwareMap.appContext.
                getResources().getIdentifier("pixelId",
                        "id", hardwareMap.appContext.getPackageName());
        pixle_cam = OpenCvCameraFactory.getInstance().
                createWebcam(hardwareMap.get(WebcamName.class, "pixle_cam"), pixelId);
        PixelDetector detector = new PixelDetector(telemetry, "red");
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

        waitForStart(); {
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
               leftClaw.setPosition(0);
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
        frontLeftMotor.setPower(-0.5);
        frontRightMotor.setPower(0.5);
        sleep((long) adjustedSleepTime);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        sleep(500);
    }
    void turnRight(double angle){

        // Calculate the number of movements needed to reach the target angle
        double movementsNeeded = Math.abs(angle / 90);

        // Calculate the adjusted sleep time based on the target angle
        double adjustedSleepTime = 650 * movementsNeeded;
        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(-0.5);
        sleep((long) adjustedSleepTime);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        sleep(500);
    }
    void forward(double inches){
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 55);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000 * movementsNeeded;

        // Set power to the motors for moving forward
        frontLeftMotor.setPower(1);
        frontRightMotor.setPower(1);

        // Sleep for the adjusted time
        sleep((long) adjustedSleepTime);

        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        sleep(500);
    }

    void backward(double inches){
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 55);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000 * movementsNeeded;

        // Set power to the motors for moving backward
        frontLeftMotor.setPower(-1);
        frontRightMotor.setPower(-1);

        // Sleep for the adjusted time
        sleep((long) adjustedSleepTime);

        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
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

        leftElbow.setPosition(position);
        rightElbow.setPosition(position);

        sleep(1000);
    }

    public void clawsDegrees(double degrees){
        double position = degreesToPosition(degrees);

        leftClaw.setPosition(position);
        rightClaw.setPosition(position);

        sleep(1000);

    }


}
