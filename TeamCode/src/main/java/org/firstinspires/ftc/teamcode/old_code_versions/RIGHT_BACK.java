package org.firstinspires.ftc.teamcode.old_code_versions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.PixelDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Autonomous(name="RIGHT BACK", group= "Auto")
public class RIGHT_BACK extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor leftElevator;
    DcMotor rightElevator;
    Servo leftElbow;
    Servo rightElbow;
    Servo leftWrist;
    Servo leftClaw;

    Servo rightClaw;

    Robot robot = new Robot();
    PixelDetector detector = new PixelDetector(telemetry, "red");
    @Override
    public void runOpMode() throws InterruptedException {
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
                robot.leftWrist.setPosition(0.25);

                // open left claw
                robot.leftClaw.setPosition(0.5);

                // move wrist up
                robot.leftWrist.setPosition(0.9);

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
                robot.rightClaw.setPosition(1);

                break;
            case CENTER:
                // Move forward 3.75 ft
                robot.moveForward(45);

                // Put wrist down
                robot.leftWrist.setPosition(0.25);

                // Open left claw
                robot.leftClaw.setPosition(1);

                // Move wrist up
                robot.leftWrist.setPosition(0.9);

                // Move forward 1.25 ft
                robot.moveForward(15);

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
                robot.rightClaw.setPosition(1);


                break;

            case RIGHT:
                // Move 3.5 ft forward
                robot.moveForward(42); // convert feet to inches

                // Turn right 90 degrees
                robot.turnRight(90);

                // move wrist down
                robot.leftWrist.setPosition(0.25);

                // open left claw
                robot.leftClaw.setPosition(1);

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
                robot.rightClaw.setPosition(1);

                break;
            case NOT_FOUND:



        }
        robot.webcam.stopStreaming();


        sleep(3000);

    }



}
