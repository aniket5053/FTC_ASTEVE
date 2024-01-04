package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.old_code_versions.RIGHT_BACK;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;
//import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "RobotController")
public class RobotController extends LinearOpMode {

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    // Lt and Rt claw servo positions are slightly offset therefore correct via SW
    static final int ELEV_BOT = 340;
    static final int ELEV_FLOOR_PICKUP = 325;
    static final int ELEV_HOME = 300;           // NEED TO GET UNDER TRUSS
    static final int ELEV_START = 0;
    static final int ELEV_HANG = -550;
    static final int ELEV_SCORE_LOW = 300;
    static final int ELEV_SCORE_MED = 0;
    static final int ELEV_SCORE_HIGH = -550;
    static final int ELEV_SCORE_VERYHIGH = -600;
    static final int ELEV_TOP = -600;

    static final double ELBOW_UP = 0.11;        //
    static final double ELBOW_DOWN = 0.79;      //was .75 too little, .9 too much
    static final double ELBOW_HANG = 0.36;      //
    static final double ELBOW_SCORE_LOW = 0.17;        //
    static final double ELBOW_SCORE_MED = 0.22;        //
    static final double ELBOW_SCORE_HIGH = 0.25;        //
    static final double ELBOW_SCORE_VERYHIGH = 0.27;        //

    static final double WRIST_HOME = 0.15;
    static final double WRIST_FLOOR_PICKUP = 0.9;   //.9 too low
    static final double WRIST_SCORE_LOW = 0.44;
    static final double WRIST_SCORE_MED = 0.50;
    static final double WRIST_SCORE_HIGH = 0.35;
    static final double WRIST_SCORE_VERYHIGH = 0.50;

    static final double LEFT_CLAW_OPEN = 0.45;
    static final double LEFT_CLAW_CLOSED = .78;
    static final double RIGHT_CLAW_OPEN = 0.5;
    static final double RIGHT_CLAW_CLOSED = .9;


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
        double botHeading = 0.0;
        double deltaHeading = 0.0;

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

        leftElevator.setPower(0.0);
        rightElevator.setPower(0.0);
        int elevTragetPos = 0;
        boolean startEvelCmd = false;
        int elevEncCloseTolerance = 20;
        int elevEncStopTolerance = 3;

        leftElbow.setPosition(ELBOW_DOWN);
        rightElbow.setPosition(ELBOW_DOWN);
        double elbowTargetPos = ELBOW_DOWN;
        double prevElbowTargetPos = ELBOW_DOWN;
        double deltaElbowPos = 0.0;
        ElapsedTime myStopwatch = new ElapsedTime();
        myStopwatch.reset();
        double delayTime = 1000;
        int currentState = 0;

        leftWrist.setPosition(WRIST_HOME);
        rightWrist.setPosition(WRIST_HOME);

        leftClaw.setPosition(LEFT_CLAW_CLOSED);
        rightClaw.setPosition(RIGHT_CLAW_CLOSED);


        waitForStart();
        if (opModeIsActive()) {

            leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            boolean isRotating;
            boolean firstTime = true;
            double initialHeading = 0.0;

            while (opModeIsActive()) {

                // These button choices were made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.options) {
                    imu.resetYaw();
                    initialHeading = 0.0;
                    botHeading = 0.0;
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
                    initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    firstTime = false;
                }

                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                deltaHeading = botHeading - initialHeading;
                if (deltaHeading > 340.0) deltaHeading = deltaHeading -360;
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
                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
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

                /////////////////////////////////////////////////////////////////////////
                // End of Mecanum Drive Section of Code
                /////////////////////////////////////////////////////////////////////////

                /////////////////////////////////////////////////////////////////////////
                // OPERATOR INPUTS - Determines movement of Elevator, Elbow, Wrist and Claws
                /////////////////////////////////////////////////////////////////////////

                //HOME
                if( (gamepad2.x)  ) {
                    startEvelCmd = true;
                    elevTragetPos = ELEV_HOME;
                    elbowTargetPos = ELBOW_DOWN;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_HOME);
                    rightWrist.setPosition(WRIST_HOME);
                    // no claw movement
                }

                //HANG
                if( (gamepad2.y)  ) {
                    startEvelCmd = true;
                    elevTragetPos = ELEV_HANG;
                    elbowTargetPos = ELBOW_HANG;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_HOME);
                    rightWrist.setPosition(WRIST_HOME);
                    leftClaw.setPosition(LEFT_CLAW_CLOSED);
                    rightClaw.setPosition(RIGHT_CLAW_CLOSED);
                }

                //FLOOR PICKUP
                if( (gamepad2.a)  ) {
                    startEvelCmd = true;
                    elevTragetPos = ELEV_FLOOR_PICKUP;
                    elbowTargetPos = ELBOW_DOWN;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_FLOOR_PICKUP);
                    rightWrist.setPosition(WRIST_FLOOR_PICKUP);
                    leftClaw.setPosition(LEFT_CLAW_OPEN);
                    rightClaw.setPosition(RIGHT_CLAW_OPEN);
                }

                //STARTING POSITION
                if( (gamepad2.b)  ) {
                    startEvelCmd = true;
                    elevTragetPos = ELEV_START;
                    elbowTargetPos = ELBOW_DOWN;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_HOME);
                    rightWrist.setPosition(WRIST_HOME);
                    // no claw movement
                }

                //Use DPAD for 4 scoring positions
                 // DPAD DOWN = Score Low
                if(gamepad2.dpad_down){
                    startEvelCmd = true;
                    elevTragetPos = ELEV_SCORE_LOW;
                    elbowTargetPos = ELBOW_SCORE_LOW;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_SCORE_LOW);
                    rightWrist.setPosition(WRIST_SCORE_LOW);
                    leftClaw.setPosition(LEFT_CLAW_CLOSED);
                    rightClaw.setPosition(RIGHT_CLAW_CLOSED);
                }

                //DPAD LEFT = Score Med
                if(gamepad2.dpad_left){
                    startEvelCmd = true;
                    elevTragetPos = ELEV_SCORE_MED;
                    elbowTargetPos = ELBOW_SCORE_MED;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_SCORE_MED);
                    rightWrist.setPosition(WRIST_SCORE_MED);
                    leftClaw.setPosition(LEFT_CLAW_CLOSED);
                    rightClaw.setPosition(RIGHT_CLAW_CLOSED);
                }

                // DPAD UP = Score High
                if(gamepad2.dpad_up){
                    startEvelCmd = true;
                    elevTragetPos = ELEV_SCORE_HIGH;
                    elbowTargetPos = ELBOW_SCORE_HIGH;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_SCORE_HIGH);
                    rightWrist.setPosition(WRIST_SCORE_HIGH);
                    leftClaw.setPosition(LEFT_CLAW_CLOSED);
                    rightClaw.setPosition(RIGHT_CLAW_CLOSED);
                }
                // DPAD RIGHT = Score Very High
                if(gamepad2.dpad_right){
                    startEvelCmd = true;
                    elevTragetPos = ELEV_SCORE_VERYHIGH;
                    elbowTargetPos = ELBOW_SCORE_VERYHIGH;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_SCORE_VERYHIGH);
                    rightWrist.setPosition(WRIST_SCORE_VERYHIGH);
                    leftClaw.setPosition(LEFT_CLAW_CLOSED);
                    rightClaw.setPosition(RIGHT_CLAW_CLOSED);
                }


                /////////////////////////////////////////////////////////////////////////
                // Elevator Movement Control
                /////////////////////////////////////////////////////////////////////////
                // MANUAL CONTROL OF ELEVATOR OVERRIDES AUTOMATIC CONTROL
                if ( ((gamepad2.left_stick_y > +db) && (leftElevator.getCurrentPosition() < ELEV_BOT))
                  || ((gamepad2.left_stick_y < -db) && (leftElevator.getCurrentPosition() > ELEV_TOP)) ) {
                    leftElevator.setPower(gamepad2.left_stick_y);
                    rightElevator.setPower(gamepad2.left_stick_y);
                    startEvelCmd = false;
                }
                else if(startEvelCmd)
                {
                    double flip = 1;
                    if(leftElevator.getCurrentPosition() > elevTragetPos) flip = -1.0;
                    if(Math.abs(leftElevator.getCurrentPosition() - elevTragetPos) > elevEncCloseTolerance) {
                        leftElevator.setPower(flip);
                        rightElevator.setPower(flip);
                    }
                    else if(Math.abs(leftElevator.getCurrentPosition() - elevTragetPos) > elevEncStopTolerance){  // slow down when close
                        leftElevator.setPower(0.33*flip);
                        rightElevator.setPower(0.33*flip);
                    }
                    else {
                        leftElevator.setPower(0.0);
                        rightElevator.setPower(0.0);
                    }
                }
                else {
                    leftElevator.setPower(0.0);
                    rightElevator.setPower(0.0);
                }


                /////////////////////////////////////////////////////////////////////////
                // Elbow Movement Control
                /////////////////////////////////////////////////////////////////////////

                if( elbowTargetPos != prevElbowTargetPos )
                {
                    switch (currentState) {
                        case 0:
                            deltaElbowPos = elbowTargetPos - prevElbowTargetPos;
                            leftElbow.setPosition(elbowTargetPos - deltaElbowPos/2);
                            rightElbow.setPosition(elbowTargetPos - deltaElbowPos/2);
                            currentState = 1;
                            delayTime = Math.abs(deltaElbowPos) *1000;
                            myStopwatch.reset();
                            break;
                        case 1:
                            if (myStopwatch.time(TimeUnit.MILLISECONDS) >= delayTime) {
                                leftElbow.setPosition(elbowTargetPos - deltaElbowPos/4);
                                rightElbow.setPosition(elbowTargetPos - deltaElbowPos/4);
                                myStopwatch.reset();
                                currentState = 2;
                            }
                            break;
                        case 2:
                            if (myStopwatch.time(TimeUnit.MILLISECONDS) >= delayTime/2) {
                                leftElbow.setPosition(elbowTargetPos - deltaElbowPos/8);
                                rightElbow.setPosition(elbowTargetPos - deltaElbowPos/8);
                                myStopwatch.reset();
                                currentState = 3;
                            }
                            break;
                        case 3:
                            if (myStopwatch.time(TimeUnit.MILLISECONDS) >= delayTime/4) {
                                leftElbow.setPosition(elbowTargetPos);
                                rightElbow.setPosition(elbowTargetPos);
                                myStopwatch.reset();
                                currentState = 99;
                            }
                            break;
                        case 99:
                            prevElbowTargetPos = elbowTargetPos;
                            break;
                    }

                }

                /////////////////////////////////////////////////////////////////////////
                // Lt / Rt Claw Movement Control
                /////////////////////////////////////////////////////////////////////////

                if(gamepad2.left_trigger > 0.33){
                    //close claw
                    leftClaw.setPosition(LEFT_CLAW_CLOSED);
                }
                else if(gamepad2.left_bumper){
                    //open claw
                    leftClaw.setPosition(LEFT_CLAW_OPEN);
                }

                if(gamepad2.right_trigger > 0.33){
                    //close claw
                    rightClaw.setPosition(RIGHT_CLAW_CLOSED);
                }
                else if(gamepad2.right_bumper){
                    //open claw
                    rightClaw.setPosition(RIGHT_CLAW_OPEN);
                }

                if(gamepad2.right_bumper && gamepad2.left_bumper){
                    leftClaw.setPosition(LEFT_CLAW_OPEN);
                    rightClaw.setPosition(RIGHT_CLAW_OPEN);

                }
                if(gamepad2.right_trigger > 0.33 && gamepad2.left_trigger > 0.33){
                    leftClaw.setPosition(LEFT_CLAW_CLOSED);
                    rightClaw.setPosition(RIGHT_CLAW_CLOSED);

                }


                /////////////////////////////////////////////////////////////////////////
                // Telemetry
                /////////////////////////////////////////////////////////////////////////

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

                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                telemetry.addLine(String.format("Heading / Error: %5.1f / %5.1f ",
                        botHeading, deltaHeading));

                telemetry.update();

            }   // end of While - Opmode is active
        }   // end of If Opmode is active
    }   // end of runOpMode Methode
}   // end of RobotController Class