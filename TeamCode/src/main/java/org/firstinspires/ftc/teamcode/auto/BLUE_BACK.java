package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "BLUE BACK Team Prop", group = "Auto")
public class BLUE_BACK extends LinearOpMode {

    static final double CLAW_OPEN = 0.5;
    static final double CLAW_CLOSED = 1.0;
    static final double ELBOW_UP = 0.17;
    static final double ELBOW_DOWN = 0.79;
    static final double WRIST_HOME = 0.15;
    static final double WRIST_OUT = 0.86;
    static final double WRIST_IN = 0.05;
    static final double WRIST_SCORE = 0.44;
    double botHeading = 0.0;
    double deltaHeading = 0.0;
    boolean isRotating;
    boolean firstTime = true;
    double initialHeading = 0.0;

    // Vars for auto movement
    double curAngle;
    double deltaAngle, unsignedDelta;
    int curPos = 0;
    double countsPerRev = 560;
    double pi = 3.1415927;
    double wheelDiameter = 3;
    double countsPerInch = countsPerRev / (wheelDiameter * pi);
    int encoderCounts;
    int deltaPos;
    ElapsedTime myStopwatch = new ElapsedTime();
    // Declare our motors
    // Make sure your ID's match your configuration
    // TA DONE: Configure HW so that names match
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    // Retrieve the IMU from the hardware map
    // TA DONE: Configure HW so that names match
    IMU imu;

    // TA DONE: Configure HW so that names match
    DcMotor leftElevator;
    DcMotor rightElevator;
    Servo leftElbow;
    Servo rightElbow;
    Servo leftWrist;
    Servo rightWrist;
    Servo leftClaw;
    Servo rightClaw;

    PixelDetector detector = new PixelDetector(telemetry, "blue");

    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration
        // TA DONE: Configure HW so that names match
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrtLtMtr");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BckLtMtr");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrtRtMtr");
        backRightMotor = hardwareMap.get(DcMotor.class, "BckRtMtr");

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        // TA DONE: test out directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        // TA DONE: Configure HW so that names match
        imu = hardwareMap.get(IMU.class, "IMU");
        // Adjust the orientation parameters to match your robot
        // TA DONE: Verify IMU orientation matches code below
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        double botHeading = 0.0;
        double deltaHeading = 0.0;

        // TA DONE: Configure HW so that names match

        leftElevator = hardwareMap.get(DcMotor.class, "LtElevator");
        rightElevator = hardwareMap.get(DcMotor.class, "RtElevator");
        leftElbow = hardwareMap.get(Servo.class, "LtElbow");
        rightElbow = hardwareMap.get(Servo.class, "RtElbow");
        leftWrist = hardwareMap.get(Servo.class, "LtWrist");
        rightWrist = hardwareMap.get(Servo.class, "RtWrist");
        leftClaw = hardwareMap.get(Servo.class, "LtClaw");
        rightClaw = hardwareMap.get(Servo.class, "RtClaw");

        //TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "TouchSensor");

        leftElevator.setDirection(DcMotor.Direction.REVERSE);
        leftElbow.setDirection(Servo.Direction.REVERSE);
        leftWrist.setDirection(Servo.Direction.REVERSE);
        leftClaw.setDirection(Servo.Direction.REVERSE);


        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle error
            }
        });


        leftClaw.setPosition(CLAW_CLOSED);
        sleep(250);
        rightClaw.setPosition(CLAW_CLOSED);
        sleep(250);

        waitForStart();
        imu.resetYaw();
        switch (detector.getLocation()) {
            case LEFT:
                driveStraight(30);
                setWristOut();
                //turn left 90
                turnToAngle(90);
                //drops on right spike mark
                rightClaw.setPosition(CLAW_OPEN);
                setWristIn();

                //strafe right
                strafe(-30);

                //turn 180
                turnToAngle(180);

                //move backwards
                driveStraight(-84);

                //strafe right
                strafe(20);

                score();

                //drive back 4 inches
                driveStraight(-4);

                leftClaw.setPosition(CLAW_OPEN);
                sleep(750);
                //move forward 4 inches
                driveStraight(4);
                //strafe right 12 inches
                strafe(12);
                home();

                break;

            case CENTER:
                //move forward 12 inches
                driveStraight(12);
                //do a 180
                turnToAngle(180);
                setWristOut();

                //move back 20 inches
                driveStraight(-20);

                //drops on center spike mark
                rightClaw.setPosition(CLAW_OPEN);
                setWristIn();

                //turn left 90
                turnToAngle(90);
                //move backwards
                driveStraight(-84);

                //strafe right
                strafe(16);

                score();

                //drive back 4 inches
                driveStraight(-4);

                leftClaw.setPosition(CLAW_OPEN);
                sleep(750);
                //move forward 4 inches
                driveStraight(4);
                //strafe right 12 inches
                strafe(12);
                home();





                break;
            case RIGHT:

                //move forward 30 inches
                driveStraight(30);
                setWristOut();
                //turn right 90
                turnToAngle(-90);


                //drops on right spike mark
                rightClaw.setPosition(CLAW_OPEN);
                setWristIn();

               //strafe left
                strafe(-30);

                //move backwards
                driveStraight(-84);

                //strafe right
                strafe(12);

                score();

                //drive back 4 inches
                driveStraight(-4);

                leftClaw.setPosition(CLAW_OPEN);
                sleep(750);
                //move forward 4 inches
                driveStraight(4);
                //strafe right 12 inches
                strafe(-12);
                home();

                break;

            case NOT_FOUND:


        }
        webcam.stopStreaming();


    }

    void autoMec(double y, double x, double rx) {

        telemetry.update();


        y = .95 * y;
        x = .55 * x;
        rx = .75 * rx;

        //////////////////////////////////////////////////////////////////////////////////
        // the code section below is correcting for robot rotation when it shouldn't happen
        //////////////////////////////////////////////////////////////////////////////////
        if (rx == 0.0) isRotating = false;
        else {
            isRotating = true;
            firstTime = true;
        }

        if ((!isRotating) && (firstTime)) {
            initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            firstTime = false;
        }

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        deltaHeading = botHeading - initialHeading;
        if (deltaHeading > 340.0) deltaHeading = deltaHeading - 360;
        if (deltaHeading < -340.0) deltaHeading = (360 + deltaHeading);
        // TA TODO: test to optimize this empirical constant and polarity of deltaHeading

        //*************************************************************************************
        // TA TODO: UPDATE THIS SECTION WHEN imu FIX KNOWN !!!!
        // IMU is not working reliably per FTC chat site (probably ESD issue) eliminate for now
        //botHeading = 0.0;
        //deltaHeading = 0.0;
        //*************************************************************************************
        if ((!isRotating) && (Math.abs(deltaHeading) > 2.2))// DEGREES
        {
            rx = rx + deltaHeading * .02;
        }
        //////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////


        // Rotate the movement direction counter to the robot's rotation
//        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // autonomous is robot oriented not field oriented!!!!
        botHeading = 0.0;
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // TA TODO: test to optimize this empirical constant
        //rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

    }


    void turnToAngle(double finalAngle) {
        double curTime;
        double startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        myStopwatch.reset();

        do {
            curAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            deltaAngle = curAngle - finalAngle;
            unsignedDelta = Math.abs(deltaAngle);
            if (unsignedDelta > 30.0) autoMec(0.0, 0.0, deltaAngle / unsignedDelta);
            else autoMec(0.0, 0.0, deltaAngle / 30.0);
            curTime = myStopwatch.time(TimeUnit.MILLISECONDS);
            //    } while ( (unsignedDelta > 2.0) && (curTime < (finalAngle - startAngle)*200.0 ) );
        } while ((unsignedDelta > 2.0));

        //Check to see if its worth trying again!!
        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        deltaAngle = (startAngle - finalAngle);
        unsignedDelta = Math.abs(deltaAngle);
        if (unsignedDelta > 2.0) {
            autoMec(0.0, 0.0, deltaAngle / unsignedDelta);  // give a nudge to start
            myStopwatch.reset();

            do {
                curAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                deltaAngle = curAngle - finalAngle;
                unsignedDelta = Math.abs(deltaAngle);
                if (unsignedDelta > 10.0) autoMec(0.0, 0.0, deltaAngle / unsignedDelta);
                else autoMec(0.0, 0.0, deltaAngle / 10.0);
                curTime = myStopwatch.time(TimeUnit.MILLISECONDS);
                //        } while ((unsignedDelta > 2.0) && (curTime < (finalAngle - startAngle) * 200.0));
            } while ((unsignedDelta > 2.0));
        }


        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(100);
    }


    void driveStraight(double inches) {

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(50);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderCounts = (int) (countsPerInch * inches);

        do {
            curPos = frontLeftMotor.getCurrentPosition();
            ;
            deltaPos = encoderCounts - curPos;
            unsignedDelta = Math.abs(deltaPos);
            //full speed
            if (unsignedDelta > 10 * countsPerInch)
                autoMec((double) deltaPos / unsignedDelta, 0.0, 0.0);
                // slow down last 10 inches
            else if (unsignedDelta > 2 * countsPerInch)
                autoMec((double) deltaPos / (double) (10 * countsPerInch), 0.0, 0.0);
                // min speed of .2 when real close
            else autoMec(0.2 * (double) deltaPos / unsignedDelta, 0.0, 0.0);
        } while (unsignedDelta > (double) countsPerInch / 2);


        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(100);
    }


    void strafe(double inches) {

        encoderCounts = (int) (countsPerInch * inches);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(50);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        do {
            curPos = frontLeftMotor.getCurrentPosition();
            ;
            deltaPos = encoderCounts - curPos;
            unsignedDelta = Math.abs(deltaPos);
            if (unsignedDelta > 10 * countsPerInch)
                autoMec(0.0, (double) deltaPos / unsignedDelta, 0.0);
            else if (unsignedDelta > 2 * countsPerInch)
                autoMec(0.0, (double) deltaPos / 10 * (double) countsPerInch, 0.0);
            else autoMec(0.2 * (double) deltaPos / unsignedDelta, 0.0, 0.0);
        } while (unsignedDelta > (double) countsPerInch / 2);


        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(100);
    }

    void setWristOut() {
        leftWrist.setPosition(WRIST_OUT);
        rightWrist.setPosition(WRIST_OUT);
        sleep(1000);
    }

    void setWristIn() {
        leftWrist.setPosition(WRIST_IN);
        rightWrist.setPosition(WRIST_IN);
        sleep(1000);
    }


    void score() {
        if (leftElbow.getPosition() != ELBOW_UP) {
            leftElbow.setPosition(ELBOW_UP + 0.4);
            rightElbow.setPosition(ELBOW_UP + 0.4);
            sleep(250);
            leftElbow.setPosition(ELBOW_UP + 0.2);
            rightElbow.setPosition(ELBOW_UP + 0.2);
            sleep(250);
            leftElbow.setPosition(ELBOW_UP);
            rightElbow.setPosition(ELBOW_UP);
        }
        leftWrist.setPosition(WRIST_SCORE);
        rightWrist.setPosition(WRIST_SCORE);
        sleep(2000);
    }

    void home() {
        leftWrist.setPosition(WRIST_HOME);
        rightWrist.setPosition(WRIST_HOME);
        sleep(150);
        //elbow down - elbow moves too fast and smashes, try a two step
        if (leftElbow.getPosition() != ELBOW_DOWN) {
            leftElbow.setPosition(ELBOW_DOWN - .506);
            rightElbow.setPosition(ELBOW_DOWN - .506);
            sleep(250);
            leftElbow.setPosition(ELBOW_DOWN - 0.306);
            rightElbow.setPosition(ELBOW_DOWN - 0.306);
            sleep(250);
            leftElbow.setPosition(ELBOW_DOWN - 0.106);
            rightElbow.setPosition(ELBOW_DOWN - 0.106);
            sleep(300);
            leftElbow.setPosition(ELBOW_DOWN);
            rightElbow.setPosition(ELBOW_DOWN);
            sleep(250);
        }
        sleep(1500);
        /////////////////////////////////////////////////////////////////////////
        // Telemetry
        /////////////////////////////////////////////////////////////////////////
        telemetry.addLine(String.format("Lt/Rt Frt Encdrs: %d  /  %d ",
                frontLeftMotor.getCurrentPosition(), frontRightMotor.getCurrentPosition()));
        telemetry.addLine(String.format("Lt/Rt Bck Encdrs: %d  /  %d ",
                backLeftMotor.getCurrentPosition(), backRightMotor.getCurrentPosition()));
        telemetry.addLine();
        telemetry.addLine(String.format("Cur / Delta Enc: %d  /  %d ",
                curPos, deltaPos));
        telemetry.addLine();
        telemetry.addLine();
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addLine(String.format("Heading / Error: %5.1f / %5.1f ",
                botHeading, deltaHeading));
        telemetry.addLine();
        telemetry.addLine(String.format("Cur / Delta Ang: %5.1f  /  %5.1f ",
                curAngle, deltaAngle));

        telemetry.update();
    }


}
