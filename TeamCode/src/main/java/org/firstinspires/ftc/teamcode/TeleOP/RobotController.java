package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor;


// AS TODO Fix issue: Robot cannot move
@TeleOp(name = "RobotController")
public class RobotController extends LinearOpMode {

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */

    static final double CLAW_OPEN = 0.5;
    static final double CLAW_CLOSED = 1.0;
    static final double ELBOW_UP = 0.21;        //was .25 too little, .15 too much
    static final double ELBOW_DOWN = 0.79;      //was .75 too little, .9 too much
    static final double ELBOW_HANG = 0.36;      //
    static final double ELBOW_SCORE_LOW = ELBOW_UP;
    static final double WRIST_HOME = 0.15;
    static final double WRIST_OUT = 1.0;
    static final double WRIST_IN = 0.05;
    static final double WRIST_FLOOR_PICKUP = 0.85;   //.9 too low
    static final double WRIST_SCORE_HIGH = 0.35;
    static final double WRIST_SCORE_LOW = 0.5;
    static final int ELEV_FLOOR = 350;
    static final int ELEV_HOME = 0;
    static final int ELEV_SCORE_LOW = -50;
    static final int ELEV_TOP = -650;
    static final double ELBOW_TOLERANCE = 0.02;
    static final double ELBOW_SPEED = 0.6;


    @Override
    public void runOpMode() {

        // Declare our motors
        // Make sure your ID's match your configuration
        // TA DONE: Configure HW so that names match
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FrtLtMtr");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BckLtMtr");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FrtRtMtr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BckRtMtr");

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
        IMU imu = hardwareMap.get(IMU.class, "IMU");
        // Adjust the orientation parameters to match your robot
        // TA DONE: Verify IMU orientation matches code below
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // TA DONE: Configure HW so that names match
        DcMotor leftElevator   = hardwareMap.get(DcMotor.class, "LtElevator");
        DcMotor rightElevator  = hardwareMap.get(DcMotor.class, "RtElevator");
        Servo   leftElbow  = hardwareMap.get(Servo.class, "LtElbow");
        Servo   rightElbow = hardwareMap.get(Servo.class, "RtElbow");
        Servo   leftWrist  = hardwareMap.get(Servo.class, "LtWrist");
        Servo   rightWrist = hardwareMap.get(Servo.class, "RtWrist");
        Servo   leftClaw   = hardwareMap.get(Servo.class, "LtClaw");
        Servo   rightClaw  = hardwareMap.get(Servo.class, "RtClaw");

        //TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "TouchSensor");

        // TA TODO: test out directions - esp Elevator - it was different in teleop and Auto
        //          FIXED FOR TELEOP - HAVE TO FIX AUTO
        leftElevator.setDirection(DcMotor.Direction.REVERSE);
        leftElbow.setDirection(Servo.Direction.REVERSE);
        leftWrist.setDirection(Servo.Direction.REVERSE);
        leftClaw.setDirection(Servo.Direction.REVERSE);

        leftWrist.setPosition(WRIST_HOME);
        rightWrist.setPosition(WRIST_HOME);
        leftElbow.setPosition(ELBOW_DOWN);
        rightElbow.setPosition(ELBOW_DOWN);
        leftClaw.setPosition(CLAW_CLOSED);
        rightClaw.setPosition(CLAW_CLOSED);



        waitForStart();
        if (opModeIsActive()) {

            leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            boolean isRotating;
            boolean firstTime = true;
            double initialHeading = 0.0;

            boolean isScoreLowReady = true;
            boolean isScoreHighReady = true;
            boolean isMoveToFloorReady = true;
            boolean isMoveUnderBarReady = true;
            boolean isMoveToHomeReady = true;
            boolean isInTolerance;
            int retryCount = 0;
            // TA TODO: Test these empirical constants
            int retryCountLimit = 3;
            int elevEncTolerance = 5;
            while (opModeIsActive()) {

                // These button choices were made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.options) {
                    imu.resetYaw();
                }

                if (gamepad1.guide) {
                    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sleep(50);
                    frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                /////////////////////////////////////////////////////////////////////////
                // Start of Mecanum Drive Section of Code
                /////////////////////////////////////////////////////////////////////////
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                //Set joystick dead bands
                // TA TODO: test to optimize this empirical constant
                double db = 0.10;
                if ((y < db) && (y > -db)) y = 0;
                if ((x < db) && (x > -db)) x = 0;
                if ((rx < db) && (rx > -db)) rx = 0;

                // slow down for more accurate movement when triggers are depressed
                if (gamepad1.right_trigger > 0.33 || gamepad1.left_trigger > 0.33) {
                    y = y / 2.0;
                    x = x / 2.0;
                    rx = rx / 2.0;
                }

                //////////////////////////////////////////////////////////////////////////////////
                // the code section below is correcting for robot rotation when it shouldn't happen
                //////////////////////////////////////////////////////////////////////////////////
                if (rx == 0.0) isRotating = false;
                else {
                    isRotating = true;
                    firstTime = true;
                }

                if ((!isRotating) && (firstTime)) {
                    initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    firstTime = false;
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double deltaHeading = botHeading - initialHeading;
                // TA TODO: test to optimize this empirical constant and polarity of deltaHeading

                //*************************************************************************************
                // TA TODO: UPDATE THIS SECTION WHEN imu FIX KNOWN !!!!
                // IMU is not working reliably per FTC chat site (probably ESD issue) eliminate for now
                botHeading = 0.0;
                deltaHeading = 0.0;
                //*************************************************************************************
                if ((!isRotating) && (Math.abs(deltaHeading) > 0.04))//0.04 rads ~= 2 deg
                {
                    rx = rx + deltaHeading;
                }
                //////////////////////////////////////////////////////////////////////////////////
                //////////////////////////////////////////////////////////////////////////////////


                // Rotate the movement direction counter to the robot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                // TA TODO: test to optimize this empirical constant
                rotX = rotX * 1.1;  // Counteract imperfect strafing

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

                /////////////////////////////////////////////////////////////////////////
                // End of Mecanum Drive Section of Code
                /////////////////////////////////////////////////////////////////////////
//                leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if ( ((gamepad2.left_stick_y > +db) && (leftElevator.getCurrentPosition() < ELEV_FLOOR))
                  || ((gamepad2.left_stick_y < -db) && (leftElevator.getCurrentPosition() > ELEV_TOP)) ) {
                    leftElevator.setPower(gamepad2.left_stick_y);
                    rightElevator.setPower(gamepad2.left_stick_y);
                }
                else {
                    leftElevator.setPower(0.0);
                    rightElevator.setPower(0.0);
                }

                //HANG
                if( (gamepad1.x)  ) {
                    leftWrist.setPosition(WRIST_HOME);
                    rightWrist.setPosition(WRIST_HOME);
                    leftElbow.setPosition(ELBOW_HANG);
                    rightElbow.setPosition(ELBOW_HANG);
                    //move elevator home here
                }

                //home
                if( (gamepad2.x)  ) {
                    leftWrist.setPosition(WRIST_HOME);
                    rightWrist.setPosition(WRIST_HOME);
                    sleep(150);
                    //elbow down - elbow moves too fast and smashes, try a two step
                    if(leftElbow.getPosition() != ELBOW_DOWN) {
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
                 }

//                //floor
                if( (gamepad2.a)  ) {
                    leftWrist.setPosition(WRIST_HOME);
                    rightWrist.setPosition(WRIST_HOME);
                    sleep(150);
                    //elbow down - elbow moves too fast and smashes, try a two step
                    if(leftElbow.getPosition() != ELBOW_DOWN) {
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
                    leftWrist.setPosition(WRIST_FLOOR_PICKUP);
                    rightWrist.setPosition(WRIST_FLOOR_PICKUP);
                }



                //score high
                if( (gamepad2.y)  ) {
                    //elbow up - elbow moves too fast and smashes, try a two step
                    if(leftElbow.getPosition() != ELBOW_UP) {
                        leftElbow.setPosition(ELBOW_UP + 0.4);
                        rightElbow.setPosition(ELBOW_UP + 0.4);
                        sleep(250);
                        leftElbow.setPosition(ELBOW_UP + 0.2);
                        rightElbow.setPosition(ELBOW_UP + 0.2);
                        sleep(250);
                        leftElbow.setPosition(ELBOW_UP);
                        rightElbow.setPosition(ELBOW_UP);
                    }
                    leftWrist.setPosition(WRIST_SCORE_HIGH);
                    rightWrist.setPosition(WRIST_SCORE_HIGH);
                }

                //score low
                if( (gamepad2.b) ) {
                    //elbow up - elbow moves too fast and smashes, try a two step
                    if(leftElbow.getPosition() != ELBOW_UP) {
                        leftElbow.setPosition(ELBOW_UP + 0.4);
                        rightElbow.setPosition(ELBOW_UP + 0.4);
                        sleep(250);
                        leftElbow.setPosition(ELBOW_UP + 0.2);
                        rightElbow.setPosition(ELBOW_UP + 0.2);
                        sleep(250);
                        leftElbow.setPosition(ELBOW_UP);
                        rightElbow.setPosition(ELBOW_UP);
                    }
                    leftWrist.setPosition(WRIST_SCORE_LOW);
                    rightWrist.setPosition(WRIST_SCORE_LOW);
                }

                if(gamepad2.left_trigger > 0.33){
                    //close claw
                    leftClaw.setPosition(CLAW_CLOSED);
                }
                else if(gamepad2.left_bumper){
                    //open claw
                    leftClaw.setPosition(CLAW_OPEN);
                }

                if(gamepad2.right_trigger > 0.33){
                    //close claw
                    rightClaw.setPosition(CLAW_CLOSED);
                }
                else if(gamepad2.right_bumper){
                    //open claw
                    rightClaw.setPosition(CLAW_OPEN);
                }

                if(gamepad2.right_bumper && gamepad2.left_bumper){
                    leftClaw.setPosition(CLAW_OPEN);
                    rightClaw.setPosition(CLAW_OPEN);

                }
                if(gamepad2.right_trigger > 0.33 && gamepad2.left_trigger > 0.33){
                    leftClaw.setPosition(CLAW_CLOSED);
                    rightClaw.setPosition(CLAW_CLOSED);

                }


                if(gamepad2.dpad_up){
                }
                if(gamepad2.dpad_down){
                }
                if(gamepad2.dpad_left){
                    //wrist up
                    leftWrist.setPosition(WRIST_HOME);
                    rightWrist.setPosition(WRIST_HOME);
                }
                if(gamepad2.dpad_right){
                    //wrist down
                    leftWrist.setPosition(WRIST_FLOOR_PICKUP);
                    rightWrist.setPosition(WRIST_FLOOR_PICKUP);
                }

                telemetry.addLine(String.format("Lt/Rt Frt Mtrs: %f4.2, %f4.2 ",
                        frontLeftMotor.getPower(),frontRightMotor.getPower()));
                telemetry.addLine(String.format("Lt/Rt Bck Mtrs: %f4.2, %f4.2 ",
                        backLeftMotor.getPower(),backRightMotor.getPower()));
                telemetry.addLine(String.format("Lt/Rt ElevMtrs: %f4.2, %f4.2 ",
                        leftElevator.getPower(),rightElevator.getPower()));
                telemetry.addLine(String.format("Lt/Rt ElbowPos: %f4.0, %f4.0 ",
                        leftElbow.getPosition(),rightElbow.getPosition()));
                telemetry.addLine(String.format("Lt/Rt WristPos: %f4.0, %f4.0 ",
                        leftWrist.getPosition(),rightWrist.getPosition()));



                telemetry.addLine(String.format("Lt/Rt Frt Encdrs: %d, %d ",
                        frontLeftMotor.getCurrentPosition(),frontRightMotor.getCurrentPosition()));
                telemetry.addLine(String.format("Lt/Rt Bck Encdrs: %d, %d ",
                        backLeftMotor.getCurrentPosition(),backRightMotor.getCurrentPosition()));
                telemetry.addLine(String.format("Lt/Rt ElevEncdrs: %d, %d %f4.2",
                        leftElevator.getCurrentPosition(),rightElevator.getCurrentPosition(), gamepad2.right_stick_y));

                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                telemetry.addLine(String.format("Heading / Error: %f5.1, %f5.1 ",
                        botHeading, deltaHeading));


                telemetry.update();
            }
        }
    }
}