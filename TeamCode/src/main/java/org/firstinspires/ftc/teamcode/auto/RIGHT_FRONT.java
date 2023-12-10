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

@Autonomous(name="RIGHT FRONT", group= "Auto")
public class RIGHT_FRONT extends LinearOpMode {

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
                robot.WRIST1.setPosition(0.25);

                // open left claw
                robot.CLAWLEFT.setPosition(1);

                // move wrist up
                robot.WRIST1.setPosition(0.9);

              //move back 2.5 ft
                robot.moveBackward(30);

                //turn right 180
                robot.turnRight(180);

                //move forward 1 ft
                robot.moveForward(12);

                // Call score_low method
                robot.score_low();

                // Set clawright position to 1
                robot.CLAWRIGHT.setPosition(1);


                break;
            case CENTER:
                // Move forward 3.75 ft
                robot.moveForward(45);

                // Put wrist down
                robot.WRIST1.setPosition(0.25);

                // Open left claw
                robot.CLAWLEFT.setPosition(1);

                // Move wrist up
                robot.WRIST1.setPosition(0.9);

                //move back 1 ft
                robot.moveBackward(12);

                //turn right 90
                robot.turnRight(90);

                //move forward 2 ft
                robot.moveForward(24);

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
                robot.WRIST1.setPosition(0.25);

                // open left claw
                robot.CLAWLEFT.setPosition(1);

                // Move wrist up
                robot.WRIST1.setPosition(0.9);

                //move forward 1.5 ft
                robot.moveForward(18);

                //turn right 90
                robot.turnRight(90);

                //move forward 1 ft
                robot.moveForward(12);

                //turn left 90
                robot.turnLeft(90);

                //move forward 0.5 ft
                robot.moveForward(6);

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
