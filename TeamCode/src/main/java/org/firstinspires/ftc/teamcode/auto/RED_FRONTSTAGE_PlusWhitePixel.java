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

@Autonomous(name = "RED  FRONTSTAGE", group = "Auto")
public class RED_FRONTSTAGE_PlusWhitePixel extends LinearOpMode {

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
    boolean startTurn = true;
    boolean firstTime = true;
    double initialHeading = 0.0;

    // Vars for auto movement
    double startAngle;
    double curAngle;
    double deltaAngle, unsignedDelta;
    int curPos = 0;
    int step = 0;
    boolean driveMoveCompleted = false;
    boolean elevatorMoveCompleted = false;
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

    PixelDetector detector = new PixelDetector(telemetry, "red");

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
                step = 0;
                propIsLeft();
                break;

            case CENTER:
            case NOT_FOUND:             // should never be this case, but have for completeness
                step = 0;
                propIsCenter();
                break;

            case RIGHT:
                step = 0;
                propIsRight();
                break;
        }

        webcam.stopStreaming();

    }


    void propIsLeft() {
        do {
            switch (step) {
                case 0:
                    resetDriveEncoders();
                    step = 10;
                    break;

                case 10:
                    setWristOut();
                    moveELevatorToTrgtPos(150);
                    driveMoveCompleted = forward(24);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        step = 20;
                    }
                    break;

                case 20:
                    moveELevatorToTrgtPos(150);
                    setWristOut();
                    driveMoveCompleted = turnLeft(startAngle, 90);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        step = 30;
                    }
                    break;

                case 30:
                    driveMoveCompleted = backwards(2);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        setWristOut();
                        step = 35;
                    }
                    break;

                case 35:
                    //drops on right Spike Mark
                    rightClaw.setPosition(CLAW_OPEN);
                    sleep(750);
                    setWristIn();
                    sleep(150);
                    startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    step = 40;
                    break;

                case 40:
                    driveMoveCompleted = turnRight(startAngle, 0);
                    moveELevatorToTrgtPos(300);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        setWristIn();
                        step = 45;
                    }
                    break;

                case 45:
                    driveMoveCompleted = backwards(20);
                    moveELevatorToTrgtPos(300);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        setWristIn();
                        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        step = 50;
                    }
                    break;

                case 50:
                    driveMoveCompleted = turnLeft(startAngle, 90);
                    elevatorMoveCompleted = moveELevatorToTrgtPos(300);

                    if (driveMoveCompleted && elevatorMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        setWristIn();
                        step = 55;
                    }
                    break;

                case 55:
                    driveMoveCompleted = backwards(75);
                    moveELevatorToTrgtPos(300);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        setWristIn();
                        step = 60;
                    }
                    break;

                case 60:
                    driveMoveCompleted = strafeRight(28);
                    moveELevatorToTrgtPos(0);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        step = 65;
                    }
                    break;

                case 65:
                    //sets up med score
                    score();
                    step = 70;
                    break;

                case 70:
                    driveMoveCompleted = backwards(6);
                    moveELevatorToTrgtPos(0);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        step = 80;
                    }
                    break;

                case 80:
                    //drops on right area
                    leftClaw.setPosition(CLAW_OPEN);
                    sleep(750);
                    step = 90;
                    break;

                case 90:
                    // must have elevator at "0" at end of autonomous for teleOp to work correctly!!!
                    // therefore on this call to move elevator, be absolutely sure it has finished getting there!!!
                    elevatorMoveCompleted = moveELevatorToTrgtPos(0);
                    driveMoveCompleted = forward(4);

                    if (elevatorMoveCompleted && driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        step = 999;
                    }
                    break;

                case 999:
                    //resets back to original position
                    home();
                    step = 1000;
                    break;

                case 1000:
                    // case just to exit while loop
                    break;
            }


            updateTelemetry();

        } while (step < 1000);

    }


    void propIsCenter() {
        do {
            switch (step) {
                case 0:
                    resetDriveEncoders();
                    step = 10;
                    break;

                case 10:
                    moveELevatorToTrgtPos(150);
                    setWristOut();
                    driveMoveCompleted = forward(24);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        step = 20;
                    }
                    break;

                case 20:
                    //drops on right Spike Mark
                    rightClaw.setPosition(CLAW_OPEN);
                    sleep(750);
                    setWristIn();
                    sleep(150);
                    startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    step = 40;
                    break;

                case 40:
                    driveMoveCompleted = backwards(20);
                    moveELevatorToTrgtPos(300);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        setWristIn();
                        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        step = 50;
                    }
                    break;

                case 50:
                    driveMoveCompleted = turnLeft(startAngle, 90);
                    elevatorMoveCompleted = moveELevatorToTrgtPos(300);

                    if (driveMoveCompleted && elevatorMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        setWristIn();
                        step = 55;
                    }
                    break;

                case 55:
                    moveELevatorToTrgtPos(300);
                    driveMoveCompleted = backwards(75);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        setWristIn();
                        step = 60;
                    }
                    break;

                case 60:
                    driveMoveCompleted = strafeRight(26);
                    moveELevatorToTrgtPos(0);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        step = 65;
                    }
                    break;

                case 65:
                    //sets up med score
                    score();
                    step = 70;
                    break;

                case 70:
                    driveMoveCompleted = backwards(6);
                    moveELevatorToTrgtPos(0);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        step = 80;
                    }
                    break;

                case 80:
                    //drops on right area
                    leftClaw.setPosition(CLAW_OPEN);
                    sleep(750);
                    step = 90;
                    break;

                case 90:
                    // must have elevator at "0" at end of autonomous for teleOp to work correctly!!!
                    // therefore on this call to move elevator, be absolutely sure it has finished getting there!!!
                    elevatorMoveCompleted = moveELevatorToTrgtPos(0);
                    driveMoveCompleted = forward(4);

                    if (elevatorMoveCompleted && driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        step = 999;
                    }
                    break;

                case 999:
                    //resets back to original position
                    home();
                    step = 1000;
                    break;

                case 1000:
                    // case just to exit while loop
                    break;
            }


            updateTelemetry();

        } while (step < 1000);
    }


    void propIsRight() {
        do {
            switch (step) {
                case 0:
                    resetDriveEncoders();
                    step = 10;
                    break;

                case 10:
                    moveELevatorToTrgtPos(150);
                    setWristOut();
                    driveMoveCompleted = forward(24);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        step = 20;
                    }
                    break;

                case 20:
                    moveELevatorToTrgtPos(150);
                    setWristOut();
                    driveMoveCompleted = turnRight(startAngle, 90);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        step = 30;
                    }
                    break;

                case 30:
                    driveMoveCompleted = backwards(2);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        setWristOut();
                        step = 35;
                    }
                    break;

                case 35:
                    //drops on right Spike Mark
                    rightClaw.setPosition(CLAW_OPEN);
                    sleep(750);
                    setWristIn();
                    sleep(150);
                    startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    step = 40;
                    break;

                case 40:
                    driveMoveCompleted = turnLeft(startAngle, 0);
                    moveELevatorToTrgtPos(300);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        setWristIn();
                        step = 45;
                    }
                    break;

                case 45:
                    driveMoveCompleted = backwards(20);
                    moveELevatorToTrgtPos(300);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        setWristIn();
                        startAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        step = 50;
                    }
                    break;


                case 50:
                    driveMoveCompleted = turnLeft(startAngle, 90);
                    elevatorMoveCompleted = moveELevatorToTrgtPos(300);

                    if (driveMoveCompleted && elevatorMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        setWristIn();
                        step = 55;
                    }
                    break;


                case 55:
                    driveMoveCompleted = backwards(75);
                    moveELevatorToTrgtPos(300);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        setWristIn();
                        step = 60;
                    }
                    break;


                case 60:
                    driveMoveCompleted = strafeRight(17);
                    moveELevatorToTrgtPos(0);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        step = 65;
                    }
                    break;


                case 65:
                    //sets up med score
                    score();
                    step = 70;
                    break;

                case 70:
                    driveMoveCompleted = backwards(6);
                    moveELevatorToTrgtPos(0);

                    if (driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        step = 80;
                    }
                    break;

                case 80:
                    //drops on right area
                    leftClaw.setPosition(CLAW_OPEN);
                    sleep(750);
                    step = 90;
                    break;

                case 90:
                    // must have elevator at "0" at end of autonomous for teleOp to work correctly!!!
                    // therefore on this call to move elevator, be absolutely sure it has finished getting there!!!
                    elevatorMoveCompleted = moveELevatorToTrgtPos(0);
                    driveMoveCompleted = forward(4);

                    if (elevatorMoveCompleted && driveMoveCompleted) {
                        stopDriveMotors();
                        resetDriveEncoders();
                        step = 999;
                    }
                    break;

                case 999:
                    //resets back to original position
                    home();
                    step = 1000;
                    break;

                case 1000:
                    // case just to exit while loop
                    break;
            }


            updateTelemetry();

        }
        while (step < 1000);

    }


    void autoMec(double y, double x, double rx) {

        telemetry.update();


        y = .75 * y;
        x = .75 * x;
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
            // initial heading should only be 0, +/-90 or +/-180, so force to these angles!!!!
            // if initialHeading is close to these numbers, set to the angle
            if (Math.abs(initialHeading -10) < 10) initialHeading = 0.0;
            if (Math.abs(initialHeading -90) < 10) initialHeading = 90.0;
            if (Math.abs(initialHeading +90) < 10) initialHeading = -90.0;
            if (Math.abs(initialHeading -180) < 10) initialHeading = 180.0;
            if (Math.abs(initialHeading +180) < 10) initialHeading = -180.0;
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


    boolean turnToAngle(double startAngle, double finalAngle) {
    int counter;
    double angleAccumulator;
    double timeToTurn = 1200;
        if(startTurn) {
            startTurn = false;
            timeToTurn = Math.abs(1200*(finalAngle-startAngle)/90);
            myStopwatch.reset();
        }

        // Use a timer just in case gyro doesnt work!!!
        if(myStopwatch.time(TimeUnit.MILLISECONDS) <= timeToTurn) {
            curAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            deltaAngle = curAngle - finalAngle;
            unsignedDelta = Math.abs(deltaAngle);
            if (unsignedDelta > 30.0) autoMec(0.0, 0.0, deltaAngle / unsignedDelta);
            else autoMec(0.0, 0.0, deltaAngle / 30.0);
            if (unsignedDelta > 2.5) return (false);
            else {startTurn = true; return (true);}
        }
        else {startTurn = true; return (true);}
    }


    boolean driveStraight(double inches) {
        encoderCounts = (int) (countsPerInch * inches);
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

        if (unsignedDelta > (double) countsPerInch / 2) return (false);
        else return (true);
    }


    boolean strafe(double inches) {
        encoderCounts = (int) (countsPerInch * inches);
        curPos = frontLeftMotor.getCurrentPosition();
        ;
        deltaPos = encoderCounts - curPos;
        unsignedDelta = Math.abs(deltaPos);
        if (unsignedDelta > 10 * countsPerInch)
            autoMec(0.0, (double) deltaPos / unsignedDelta, 0.0);
        else if (unsignedDelta > 2 * countsPerInch)
            autoMec(0.0, (double) deltaPos / (double) (10 * countsPerInch), 0.0);
        else autoMec(0.2 * (double) deltaPos / unsignedDelta, 0.0, 0.0);

        if (unsignedDelta > (double) countsPerInch / 2) return (false);
        else return (true);
    }

    boolean rightDiagonal(double inches) {
        encoderCounts = (int) (countsPerInch * inches * 1.7);
        curPos = frontLeftMotor.getCurrentPosition();
        ;
        deltaPos = encoderCounts - curPos;
        unsignedDelta = Math.abs(deltaPos);
        //full speed
        if (unsignedDelta > 10 * countsPerInch)
            autoMec((double) deltaPos / unsignedDelta, deltaPos / unsignedDelta, 0.0);
            // slow down last 10 inches
        else if (unsignedDelta > 2 * countsPerInch)
            autoMec((double) deltaPos / (double) (10 * countsPerInch), deltaPos / (double) (10 * countsPerInch), 0.0);
            // min speed of .2 when real close
        else
            autoMec(0.2 * (double) deltaPos / unsignedDelta, 0.2 * (double) deltaPos / unsignedDelta, 0.0);

        if (unsignedDelta > (double) countsPerInch / 2) return (false);
        else return (true);
    }

    boolean leftDiagonal(double inches) {
        encoderCounts = (int) (countsPerInch * inches * 1.7);
        curPos = frontRightMotor.getCurrentPosition();
        ;
        deltaPos = encoderCounts - curPos;
        unsignedDelta = Math.abs(deltaPos);
        //full speed
        if (unsignedDelta > 10 * countsPerInch)
            autoMec((double) deltaPos / unsignedDelta, -deltaPos / unsignedDelta, 0.0);
            // slow down last 10 inches
        else if (unsignedDelta > 2 * countsPerInch)
            autoMec((double) deltaPos / (double) (10 * countsPerInch), -deltaPos / (double) (10 * countsPerInch), 0.0);
            // min speed of .2 when real close
        else
            autoMec(0.2 * (double) deltaPos / unsignedDelta, -0.2 * (double) deltaPos / unsignedDelta, 0.0);

        if (unsignedDelta > (double) countsPerInch / 2) return (false);
        else return (true);
    }

    void resetDriveEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(50);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(50);
    }

    void stopDriveMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(50);
    }


    boolean forward(double inches) {
        return driveStraight(inches);
    }

    boolean backwards(double inches) {
        return driveStraight(-inches);
    }

    boolean turnRight(double startAngle, double degrees) {
        return turnToAngle(startAngle, -degrees);
    }

    boolean turnLeft(double startAngle, double degrees) {
        return turnToAngle(startAngle, degrees);
    }

    boolean strafeRight(double inches) {
        return strafe(inches);
    }

    boolean strafeLeft(double inches) {
        return strafe(-inches);
    }




    boolean moveELevatorToTrgtPos(int elevTragetPos) {
        double flip = 1;
        boolean complete;
        int elevEncCloseTolerance = 20;
        int elevEncStopTolerance = 3;
        if(leftElevator.getCurrentPosition() > elevTragetPos) flip = -1.0;
        if(Math.abs(leftElevator.getCurrentPosition() - elevTragetPos) > elevEncCloseTolerance) {
            leftElevator.setPower(flip);
            rightElevator.setPower(flip);
            complete = false;
        }
        else if(Math.abs(leftElevator.getCurrentPosition() - elevTragetPos) > elevEncStopTolerance){  // slow down when close
            leftElevator.setPower(0.33*flip);
            rightElevator.setPower(0.33*flip);
            complete = false;
        }
        else {
            leftElevator.setPower(0.0);
            rightElevator.setPower(0.0);
            complete = true;
        }

        return (complete);
    }



    void setWristOut() {
        leftWrist.setPosition(WRIST_OUT);
        rightWrist.setPosition(WRIST_OUT);
//        sleep(750);
    }

    void setWristIn() {
        leftWrist.setPosition(WRIST_IN);
        rightWrist.setPosition(WRIST_IN);
//        sleep(750);
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
    }


    /////////////////////////////////////////////////////////////////////////
    // Telemetry
    /////////////////////////////////////////////////////////////////////////
    void updateTelemetry(){
//                telemetry.addLine(String.format("Lt/Rt Frt Mtrs: %4.2f  /  %4.2f ",
//                        frontLeftMotor.getPower(),frontRightMotor.getPower()));
//                telemetry.addLine(String.format("Lt/Rt Bck Mtrs: %4.2f  /  %4.2f ",
//                        backLeftMotor.getPower(),backRightMotor.getPower()));
//                telemetry.addLine(String.format("Lt/Rt ElevMtrs: %4.2f /  %4.2f ",
//                        leftElevator.getPower(),rightElevator.getPower()));
        telemetry.addLine(String.format("Lt/Rt ElbowPos: %4.2f /  %4.2f ",
                leftElbow.getPosition(),rightElbow.getPosition()));
        telemetry.addLine(String.format("Lt/Rt WristPos: %4.2f /  %4.2f ",
                leftWrist.getPosition(),rightWrist.getPosition()));
        telemetry.addLine(String.format("Lt/Rt ClawPos: %4.2f /  %4.2f ",
                leftClaw.getPosition(),rightClaw.getPosition()));
        telemetry.addLine();
        telemetry.addLine();

        telemetry.addLine(String.format("Lt/Rt Frt Encdrs: %d  /  %d ",
                frontLeftMotor.getCurrentPosition(),frontRightMotor.getCurrentPosition()));
        telemetry.addLine(String.format("Lt/Rt Bck Encdrs: %d  /  %d ",
                backLeftMotor.getCurrentPosition(),backRightMotor.getCurrentPosition()));
        telemetry.addLine();
        telemetry.addLine(String.format("Lt/Rt ElevEncdrs: %d  /  %d  /  %4.2f",
                leftElevator.getCurrentPosition(),rightElevator.getCurrentPosition(),
                gamepad2.right_stick_y));

        botHeading =imu.getRobotYawPitchRollAngles().

                getYaw(AngleUnit.DEGREES);
        telemetry.addLine(String.format("Heading / Error: %5.1f / %5.1f ",
                botHeading,deltaHeading));

        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine(String.format("Step: %d ", step));


        telemetry.update();
    }




}

