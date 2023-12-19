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

@Autonomous(name="Robot", group= "Auto")
public class Robot extends LinearOpMode {

    private static final double FEET_PER_METER = 3.28084;
    private static final long MS_PER_INCH = 18;

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor leftElevator;
    DcMotor rightElevator;
    Servo leftElbow;
    Servo rightElbow;
    Servo leftWrist;
    Servo leftClaw;

    Servo rightClaw;

    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        //
    }

    void initializeHardware() {
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

        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void setupCamera(PixelDetector detector, String color) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        detector = new PixelDetector(telemetry, color);
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
        frontLeftMotor.setPower(1);
        frontRightMotor.setPower(1);

        // Sleep for the adjusted time
        sleep((long) adjustedSleepTime);

        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        sleep(500);
    }

    void moveBackward(double inches) {
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

    void turnLeft(double angle) {
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

    void turnRight(double angle) {
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


    private double degreesToPosition(double degrees) {
        double minDegrees = 0.0;
        double maxDegrees = 180.0;

        // Convert degrees to the servo position scale (0 to 1)
        return (degrees - minDegrees) / (maxDegrees - minDegrees);
    }

    void elbowDegrees(double degrees) {
        double position = degreesToPosition(degrees);

        leftElbow.setPosition(position);
        rightElbow.setPosition(position);

        sleep(1000);
    }

    void clawsDegrees(double degrees) {
        double position = degreesToPosition(degrees);

        leftClaw.setPosition(position);
        rightClaw.setPosition(position);

        sleep(1000);
    }

    void home() {
        leftWrist.setPosition(0.9);
        leftElbow.setPosition(0.9);
        rightElbow.setPosition(0.9);
        //move elevator home here
        leftElevator.setTargetPosition(0);
        rightElevator.setTargetPosition(0);
        leftElevator.setPower(1);
        rightElevator.setPower(1);
        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftElevator.isBusy() && rightElevator.isBusy()) {
            frontLeftMotor.setPower(-gamepad1.left_stick_y);
            frontRightMotor.setPower(-gamepad1.right_stick_y);
        }
        leftElevator.setPower(0);
        rightElevator.setPower(0);

    }

    void floor() {
        leftElbow.setPosition(0.25);
        rightElbow.setPosition(0.9);
        leftWrist.setPosition(0.57);
        //move elevator down
        leftElevator.setTargetPosition(300);  //s/b -360
        rightElevator.setTargetPosition(300);
        leftElevator.setPower(1);
        rightElevator.setPower(1);
        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftElevator.isBusy() && rightElevator.isBusy()) {
            //
        }
        leftElevator.setPower(0);
        rightElevator.setPower(0);

    }


    void score_low()
    {
        leftElbow.setPosition(0.1);
        rightElbow.setPosition(0.1);
        leftWrist.setPosition(0.2);
        //move elevator up
        leftElevator.setTargetPosition(-50);
        rightElevator.setTargetPosition(-50);
        leftElevator.setPower(1);
        rightElevator.setPower(1);
        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftElevator.isBusy() && rightElevator.isBusy()) {
            //
        }
        leftElevator.setPower(0);
        rightElevator.setPower(0);
    }

    void score_high()
    {
        leftElbow.setPosition(0.1);
        rightElbow.setPosition(0.1);
        leftWrist.setPosition(0.2);
        //move elevator up
        leftElevator.setTargetPosition(-700);
        rightElevator.setTargetPosition(-700);
        leftElevator.setPower(1);
        rightElevator.setPower(1);
        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftElevator.isBusy() && rightElevator.isBusy()) {
            //
        }
        leftElevator.setPower(0);
        rightElevator.setPower(0);
    }

    void underbar_crossing(){
        leftElbow.setPosition(0.9);
        rightElbow.setPosition(0.9);
        leftWrist.setPosition(0.57);
        //move elevator down
        leftElevator.setTargetPosition(300);  //s/b -360
        rightElevator.setTargetPosition(300);
        leftElevator.setPower(1);
        rightElevator.setPower(1);
        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (leftElevator.isBusy() && rightElevator.isBusy()) {
            //
        }
        leftElevator.setPower(0);
        rightElevator.setPower(0);

    }

}
