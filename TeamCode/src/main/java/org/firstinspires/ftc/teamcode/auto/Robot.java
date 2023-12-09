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

@Autonomous(name="Robot", group= "Auto")
public class Robot extends LinearOpMode {

    private static final double FEET_PER_METER = 3.28084;
    private static final long MS_PER_INCH = 18;

    DcMotor LEFTDRIVE;
    DcMotor RIGHTDRIVE;
    DcMotor LEFTAXLE;
    DcMotor RIGHTAXLE;
    Servo ELBOW1;
    Servo ELBOW2;
    Servo WRIST1;
    Servo CLAWLEFT;

    Servo CLAWRIGHT;

    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        //
    }

    void initializeHardware() {
        LEFTDRIVE = hardwareMap.get(DcMotor.class, "LEFT DRIVE");
        RIGHTDRIVE = hardwareMap.get(DcMotor.class, "RIGHT DRIVE");
        LEFTAXLE = hardwareMap.get(DcMotor.class, "LEFT AXLE");
        RIGHTAXLE = hardwareMap.get(DcMotor.class, "RIGHT AXLE");
        ELBOW1 = hardwareMap.get(Servo.class, "ELBOW1");
        ELBOW2 = hardwareMap.get(Servo.class, "ELBOW2");
        WRIST1 = hardwareMap.get(Servo.class, "WRIST");
        CLAWLEFT = hardwareMap.get(Servo.class, "CLAWLEFT");
        CLAWRIGHT = hardwareMap.get(Servo.class, "CLAWRIGHT");

        LEFTDRIVE.setDirection(DcMotor.Direction.REVERSE);
        LEFTAXLE.setDirection(DcMotor.Direction.REVERSE);
        ELBOW2.setDirection(Servo.Direction.REVERSE);
        CLAWLEFT.setDirection(Servo.Direction.REVERSE);

        LEFTAXLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTAXLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void setupCamera(PixelDetector detector) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        detector = new PixelDetector(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle error
            }
        });
    }


    void moveForward(double inches) {
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 55);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000 * movementsNeeded;

        // Set power to the motors for moving forward
        LEFTDRIVE.setPower(1);
        RIGHTDRIVE.setPower(1);

        // Sleep for the adjusted time
        sleep((long) adjustedSleepTime);

        // Stop the motors after the sleep
        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
        sleep(500);
    }

    void moveBackward(double inches) {
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 55);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000 * movementsNeeded;

        // Set power to the motors for moving backward
        LEFTDRIVE.setPower(-1);
        RIGHTDRIVE.setPower(-1);

        // Sleep for the adjusted time
        sleep((long) adjustedSleepTime);

        // Stop the motors after the sleep
        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
        sleep(500);
    }

    void turnLeft(double angle) {
        // Calculate the number of movements needed to reach the target angle
        double movementsNeeded = Math.abs(angle / 90);

        // Calculate the adjusted sleep time based on the target angle
        double adjustedSleepTime = 650 * movementsNeeded;
        LEFTDRIVE.setPower(-0.5);
        RIGHTDRIVE.setPower(0.5);
        sleep((long) adjustedSleepTime);
        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
        sleep(500);
    }

    void turnRight(double angle) {
        // Calculate the number of movements needed to reach the target angle
        double movementsNeeded = Math.abs(angle / 90);

        // Calculate the adjusted sleep time based on the target angle
        double adjustedSleepTime = 650 * movementsNeeded;
        LEFTDRIVE.setPower(0.5);
        RIGHTDRIVE.setPower(-0.5);
        sleep((long) adjustedSleepTime);
        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
        sleep(500);
    }


    private double degreesToPosition(double degrees) {
        double minDegrees = 0.0;
        double maxDegrees = 180.0;

        // Convert degrees to the servo position scale (0 to 1)
        return (degrees - minDegrees) / (maxDegrees - minDegrees);
    }

    void elbowDegrees(double degrees) {
        double position = degreesToPosition(degrees);

        ELBOW1.setPosition(position);
        ELBOW2.setPosition(position);

        sleep(1000);
    }

    void clawsDegrees(double degrees) {
        double position = degreesToPosition(degrees);

        CLAWLEFT.setPosition(position);
        CLAWRIGHT.setPosition(position);

        sleep(1000);
    }

    void home() {
        WRIST1.setPosition(1);
        ELBOW1.setPosition(1);
        ELBOW2.setPosition(1);
        //move elevator home here
        LEFTAXLE.setTargetPosition(0);
        RIGHTAXLE.setTargetPosition(0);
        LEFTAXLE.setPower(1);
        RIGHTAXLE.setPower(1);
        LEFTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RIGHTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LEFTAXLE.isBusy() && RIGHTAXLE.isBusy()) {
            LEFTDRIVE.setPower(-gamepad1.left_stick_y);
            RIGHTDRIVE.setPower(-gamepad1.right_stick_y);
        }
        LEFTAXLE.setPower(0);
        RIGHTAXLE.setPower(0);

    }

    void floor() {
        ELBOW1.setPosition(1);
        ELBOW2.setPosition(1);
        WRIST1.setPosition(0);
        //move elevator down
        LEFTAXLE.setTargetPosition(250);  //s/b -360
        RIGHTAXLE.setTargetPosition(250);
        LEFTAXLE.setPower(1);
        RIGHTAXLE.setPower(1);
        LEFTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RIGHTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LEFTAXLE.isBusy() && RIGHTAXLE.isBusy()) {
            //
        }
        LEFTAXLE.setPower(0);
        RIGHTAXLE.setPower(0);

    }


    void underbar_cross()
    {
        ELBOW1.setPosition(1);
        ELBOW2.setPosition(1);
        WRIST1.setPosition(0.5);
        //move elevator down
        LEFTAXLE.setTargetPosition(250);  //s/b -360
        RIGHTAXLE.setTargetPosition(250);
        LEFTAXLE.setPower(1);
        RIGHTAXLE.setPower(1);
        LEFTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RIGHTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LEFTAXLE.isBusy() && RIGHTAXLE.isBusy()) {
            //
        }
        LEFTAXLE.setPower(0);
        RIGHTAXLE.setPower(0);
    }

    void score_low()
    {
        ELBOW1.setPosition(0);
        ELBOW2.setPosition(0);
        WRIST1.setPosition(0.8);
        //move elevator up
        LEFTAXLE.setTargetPosition(-50);
        RIGHTAXLE.setTargetPosition(-50);
        LEFTAXLE.setPower(1);
        RIGHTAXLE.setPower(1);
        LEFTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RIGHTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LEFTAXLE.isBusy() && RIGHTAXLE.isBusy()) {
            //
        }
        LEFTAXLE.setPower(0);
        RIGHTAXLE.setPower(0);
    }

    void score_high()
    {
        ELBOW1.setPosition(0);
        ELBOW2.setPosition(0);
        WRIST1.setPosition(0.8);
        //move elevator up
        LEFTAXLE.setTargetPosition(-700);
        RIGHTAXLE.setTargetPosition(-700);
        LEFTAXLE.setPower(1);
        RIGHTAXLE.setPower(1);
        LEFTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RIGHTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LEFTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LEFTAXLE.isBusy() && RIGHTAXLE.isBusy()) {
            //
        }
        LEFTAXLE.setPower(0);
        RIGHTAXLE.setPower(0);
    }
}
