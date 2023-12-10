package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="RIGHT BACK", group= "Auto")
public class RIGHT_BACK extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;
    DcMotor LEFTDRIVE;
    DcMotor RIGHTDRIVE;
    DcMotor LEFTAXLE;
    DcMotor RIGHTAXLE;
    Servo ELBOW1;
    Servo ELBOW2;
    Servo WRIST1;
    Servo CLAWLEFT;

    Servo CLAWRIGHT;

    Robot robot = new Robot();
    PixelDetector detector = new PixelDetector(telemetry, "red");
    @Override
    public void runOpMode() throws InterruptedException {
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
        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        detector = new PixelDetector(telemetry, "red");
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

        waitForStart();
        switch (detector.getLocation()){
            case LEFT:
                // Move 3.5 ft forward (3.5 ft = 42 inches)
                robot.moveForward(42);

                // Turn left 90 degrees
                robot.turnLeft(90);

                // move wrist down
                robot.WRIST1.setPosition(0.56);

                // open left claw
                robot.CLAWLEFT.setPosition(0.5);

                // move wrist up
                robot.WRIST1.setPosition(0.8);

                // Turn right 90 degrees
                robot.turnRight(90);

                // Move 2.5 ft forward (2.5 ft = 30 inches)
                robot.moveForward(30);

                // Turn right 90 degrees
                robot.turnRight(90);

                //setup robot to cross over
                robot.underbar_crossing();

                // Move forward 6 ft (6 ft = 72 inches)
                robot.moveForward(72);

                // Turn right 90 degrees
                robot.turnRight(90);

                // Move forward 1.25 ft (1.25 ft = 15 inches)
                robot.moveForward(15);

                //Turn left 90 degrees
                robot.turnLeft(90);

                // Move forward 1 ft (1 ft = 12 inches)
                robot.moveForward(12);

                // Call score_low method
                robot.score_low();

                // Set clawright position to 1
                robot.CLAWRIGHT.setPosition(1);

                break;
            case CENTER:
                // Move forward 3.75 inches
                robot.moveForward(3.75);

                // Put wrist down
                robot.WRIST1.setPosition(0.5);

                // Open left claw
                robot.CLAWLEFT.setPosition(1);

                // Move wrist up
                robot.WRIST1.setPosition(0.8);

                // Move forward 1.25 inches
                robot.moveForward(1.25);

                // Turn right 90 degrees
                robot.turnRight(90);

                //setup robot to cross over
                robot.underbar_crossing();

                // Move forward 6 ft (6 ft = 72 inches)
                robot.moveForward(72);

                // Turn right 90 degrees
                robot.turnRight(90);

                // Move forward 1.5 ft (1.5 ft = 18 inches)
                robot.moveForward(18);

                //Turn left 90 degrees
                robot.turnLeft(90);

                // Move forward 1 ft (1 ft = 12 inches)
                robot.moveForward(12);

                // Call score_low method
                robot.score_low();

                // Set clawright position to 1
                robot.CLAWRIGHT.setPosition(1);


                break;

            case RIGHT:
                // Move 3.5 ft forward
                robot.moveForward(42); // convert feet to inches

                // Turn right 90 degrees
                robot.turnRight(90);

                // move wrist down
                robot.WRIST1.setPosition(0.5);

                // open left claw
                robot.CLAWLEFT.setPosition(1);

                // Move back 2 ft
                robot.moveBackward(24);

                //setup robot to cross over
                robot.underbar_crossing();

                // Turn left 90 degrees
                robot.turnLeft(90);

                // Move forward 2.5 ft
                robot.moveForward(30);

                // Turn right 90 degrees
                robot.turnRight(90);

                // Move 6 ft forward
                robot.moveForward(72);

                // Turn right 90 degrees
                robot.turnRight(90);

                // Move forward 1.75 ft
                robot.moveForward(21);

                //Turn left 90 degrees
                robot.turnLeft(90);

                // Move forward 1 ft (1 ft = 12 inches)
                robot.moveForward(12);

                // Call score_low method
                robot.score_low();

                // Set clawright position to 1
                robot.CLAWRIGHT.setPosition(1);

                break;
            case NOT_FOUND:



        }
        robot.webcam.stopStreaming();



    }



}
