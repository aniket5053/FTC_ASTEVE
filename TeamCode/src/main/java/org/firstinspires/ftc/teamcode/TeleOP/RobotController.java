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

@TeleOp(name = "RobotController")
public class RobotController extends LinearOpMode {

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */

    @Override
    public void runOpMode() {

        // Declare our motors
        // Make sure your ID's match your configuration
        // TA TODO: Configure HW so that names match
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FrtLtMtr");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BckLtMtr");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FrtRtMtr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BckRtMtr");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        // TA TODO: test out directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        // TA TODO: Configure HW so that names match
        IMU imu = hardwareMap.get(IMU.class, "IMU");
        // Adjust the orientation parameters to match your robot
        // TA TODO: Verify IMU orientation matches code below
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // TA TODO: Configure HW so that names match
        DcMotor leftElevator   = hardwareMap.get(DcMotor.class, "LtElevator");
        DcMotor rightElevator  = hardwareMap.get(DcMotor.class, "RtElevator");
        Servo   leftElbow  = hardwareMap.get(Servo.class, "LtElbow");
        Servo   rightElbow = hardwareMap.get(Servo.class, "RtElbow");
        Servo   leftWrist  = hardwareMap.get(Servo.class, "LtWrist");
//        Servo   rightWrist = hardwareMap.get(Servo.class, "RtWrist");
        Servo   leftClaw   = hardwareMap.get(Servo.class, "LtClaw");
        Servo   rightClaw  = hardwareMap.get(Servo.class, "RtClaw");

        //TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "TouchSensor");

        // TA TODO: test out directions - esp Elevator - it was different in teleOp and Auto
        rightElbow.setDirection(Servo.Direction.REVERSE);
        leftElevator.setDirection(DcMotor.Direction.REVERSE);
        leftClaw.setDirection(Servo.Direction.REVERSE);

        leftWrist.setPosition(1);
        leftElbow.setPosition(0.9);
        rightElbow.setPosition(0.9);
        leftClaw.setPosition(1);
        rightClaw.setPosition(1);



        waitForStart();
        if (opModeIsActive()) {

            leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.options) {
                    imu.resetYaw();
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
                if( (y < db) && (y > -db) ) y = 0;
                if( (x < db) && (x > -db) ) x = 0;
                if( (rx < db) && (rx > -db) ) rx = 0;

                // slow down for more accurate movement when triggers are depressed
                if(gamepad1.right_trigger > 0.33 || gamepad1.left_trigger > 0.33) {
                    y = y/2.0;
                    x = x/2.0;
                    rx = rx/2.0;
                }

                //////////////////////////////////////////////////////////////////////////////////
                // the code section below is correcting for robot rotation when it shouldn't happen
                //////////////////////////////////////////////////////////////////////////////////
                if(rx == 0.0) isRotating = false;
                else { isRotating = true; firstTime = true; }

                if( (!isRotating) && (firstTime) ) {
                    initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    firstTime = false;
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double deltaHeading = botHeading - initialHeading;
                // TA TODO: test to optimize this empirical constant and polarity of deltaHeading
                if( (!isRotating) && (Math.abs(deltaHeading) > 0.04) )//0.04 rads ~= 2 deg
                {   rx = rx + deltaHeading; }
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


                leftElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                leftElevator.setPower(gamepad2.right_stick_y);
                rightElevator.setPower(gamepad2.right_stick_y);


                //home
                if( (gamepad1.x) || !isMoveToHomeReady ) {
                    isMoveToHomeReady = false;
                    leftWrist.setPosition(1);
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

                    if( !rightElevator.isBusy() && !leftElevator.isBusy() ) {
                        retryCount++;
                        isInTolerance = (Math.abs(leftElevator.getCurrentPosition() - 0) <= elevEncTolerance)
                                && (Math.abs(rightElevator.getCurrentPosition() - 0) <= elevEncTolerance);
                        if( (retryCount > retryCountLimit) || (isInTolerance) ) {
                            leftElevator.setPower(0);
                            rightElevator.setPower(0);
                            retryCount = 0;
                            isMoveToHomeReady = true;
                        }
                    }
                }

                //under-bar crossing
                else if( (gamepad2.x) || !isMoveUnderBarReady ) {
                    isMoveUnderBarReady = false;
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

                    if( !rightElevator.isBusy() && !leftElevator.isBusy() ) {
                        retryCount++;
                        isInTolerance = (Math.abs(leftElevator.getCurrentPosition() - 300) <= elevEncTolerance)
                                && (Math.abs(rightElevator.getCurrentPosition() - 300) <= elevEncTolerance);
                        if ((retryCount > retryCountLimit) || (isInTolerance)) {
                            leftElevator.setPower(0);
                            rightElevator.setPower(0);
                            retryCount = 0;
                            isMoveUnderBarReady = true;
                        }
                    }
                 }

                //floor
                else if( (gamepad2.a) || !isMoveToFloorReady ) {
                    isMoveToFloorReady = false;
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

                    if( !rightElevator.isBusy() && !leftElevator.isBusy() ) {
                        retryCount++;
                        isInTolerance = (Math.abs(leftElevator.getCurrentPosition() - 300) <= elevEncTolerance)
                                && (Math.abs(rightElevator.getCurrentPosition() - 300) <= elevEncTolerance);
                        if ((retryCount > retryCountLimit) || (isInTolerance)) {
                            leftElevator.setPower(0);
                            rightElevator.setPower(0);
                            retryCount = 0;
                            isMoveToFloorReady = true;
                        }
                    }
                }


                //score high
                else if( (gamepad2.y) || !isScoreHighReady ) {
                    isScoreHighReady = false;
                    leftElbow.setPosition(0);
                    rightElbow.setPosition(0);
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

                    if( !rightElevator.isBusy() && !leftElevator.isBusy() ) {
                        retryCount++;
                        isInTolerance = (Math.abs(leftElevator.getCurrentPosition() - -700) <= elevEncTolerance)
                                && (Math.abs(rightElevator.getCurrentPosition() - -700) <= elevEncTolerance);
                        if ((retryCount > retryCountLimit) || (isInTolerance)) {
                            leftElevator.setPower(0);
                            rightElevator.setPower(0);
                            retryCount = 0;
                            isScoreHighReady = true;
                        }
                    }
                }

                //score low
                else if( (gamepad2.b) || !isScoreLowReady ) {
                    isScoreLowReady = false;
                    leftElbow.setPosition(0.1);
                    rightElbow.setPosition(0.1);
                    leftWrist.setPosition(.2);
                    //move elevator up
                    leftElevator.setTargetPosition(-50);
                    rightElevator.setTargetPosition(-50);
                    leftElevator.setPower(1);
                    rightElevator.setPower(1);
                    leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if( !rightElevator.isBusy() && !leftElevator.isBusy() ) {
                        retryCount++;
                        isInTolerance = (Math.abs(leftElevator.getCurrentPosition() - -50) <= elevEncTolerance)
                                && (Math.abs(rightElevator.getCurrentPosition() - -50) <= elevEncTolerance);
                        if ((retryCount > retryCountLimit) || (isInTolerance)) {
                            leftElevator.setPower(0);
                            rightElevator.setPower(0);
                            retryCount = 0;
                            isScoreLowReady = true;
                        }
                    }
                }

                if(gamepad2.left_trigger > 0.33){
                    //close claw
                    leftClaw.setPosition(1);
                }
                else if(gamepad2.left_bumper){
                    //open claw
                    leftClaw.setPosition(0.5);
                }

                if(gamepad2.right_trigger > 0.33){
                    //close claw
                    rightClaw.setPosition(1);
                }
                else if(gamepad2.right_bumper){
                    //open claw
                    rightClaw.setPosition(0.5);
                }

                if(gamepad2.right_bumper && gamepad2.left_bumper){
                    leftClaw.setPosition(0.5);
                    rightClaw.setPosition(0.5);

                }
                if(gamepad2.right_trigger > 0.33 && gamepad2.left_trigger > 0.33){
                    leftClaw.setPosition(1);
                    rightClaw.setPosition(1);

                }


                if(gamepad2.dpad_up){
                    //elbow up
                    leftElbow.setPosition(0.1);
                    rightElbow.setPosition(0.1);

                }
                if(gamepad2.dpad_down){
                    //elbow down
                    leftElbow.setPosition(1);
                    rightElbow.setPosition(1);
                }
                if(gamepad2.dpad_left){
                    //wrist up
                    leftWrist.setPosition(0.75);
                }
                if(gamepad2.dpad_right){
                    //wrist down
                    leftWrist.setPosition(0.05);
                }





                telemetry.addData("FrtLt Power", frontLeftMotor.getPower());
                telemetry.addData("BckLt Power", backLeftMotor.getPower());
                telemetry.addData("FrtRt Power", frontRightMotor.getPower());
                telemetry.addData("BckRt Power", backRightMotor.getPower());
                telemetry.addData("LeftEl", leftElevator.getPower());
                telemetry.addData("RightEl", rightElevator.getPower());
                telemetry.addData("leftElbow",leftElbow.getPosition() );
                telemetry.addData("rightElbow",rightElbow.getPosition() );
                telemetry.addData("LeftWrist",leftWrist.getPosition() );
                telemetry.addData("RightWrist",leftWrist.getPosition() );
                telemetry.addData("LeftEl Encoder",leftElevator.getCurrentPosition());
                telemetry.addData("FrtLt Encoder", frontLeftMotor.getCurrentPosition());
                telemetry.addData("BckRt Encoder", backRightMotor.getCurrentPosition());
                telemetry.addData("Heading", botHeading);


                telemetry.update();
            }
        }
    }
}