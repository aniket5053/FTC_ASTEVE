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

@Autonomous(name="Encoder Test", group= "Auto")
public class EncoderTest extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;

    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initializeHardware();

        waitForStart();
        // Move 3.5 ft forward (3.5 ft = 42 inches)
        robot.moveForward(42);

        // Turn left 90 degrees
        robot.turnLeft(90);

        // move wrist down
        robot.WRIST1.setPosition(0.5);

        // open left claw
        robot.CLAWLEFT.setPosition(1);

        // move wrist up
        robot.WRIST1.setPosition(0.8);

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



        }


    }


