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

    Robot robot = new Robot();
    PixelDetector detector = new PixelDetector(telemetry);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeHardware();

        robot.setupCamera(detector);

        waitForStart();
        switch (detector.getLocation()){
            case LEFT:
                // Move 3.5 ft forward (3.5 ft = 42 inches)
                robot.moveForward(42);

                // Turn left 90 degrees
                robot.turnLeft(90);

                // move wrist down
                robot.WRIST1.setPosition(0);

                // open left claw
                robot.CLAWLEFT.setPosition(1);

                // move wrist up
                robot.WRIST1.setPosition(1);

                // Turn right 90 degrees
                robot.turnRight(90);

                // Move 2.5 ft forward (2.5 ft = 30 inches)
                robot.moveForward(30);

                // Turn right 90 degrees
                robot.turnRight(90);

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

                break;

            case RIGHT:

                break;
            case NOT_FOUND:



        }
        robot.webcam.stopStreaming();



    }



}
