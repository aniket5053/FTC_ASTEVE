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

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor leftElevator;
    DcMotor rightElevator;
    Servo leftElbow;
    Servo rightElbow;
    Servo leftWrist;
    Servo rightWrist;
    Servo leftClaw;
    Servo rightClaw;
    IMU imu;
    double botHeading = 0.0;
    double deltaHeading = 0.0;

    boolean isRotating;
    boolean fieldCentric = true;
    boolean firstTime = true;
    boolean toggleswitch = false;
    boolean firstGyro = true;
    double yawOffset = 0.0;
    double initialHeading = 0.0;

    // Vars for auto movement
    double curAngle;
    double deltaAngle, unsignedDelta;
    int curPos = 0;
    double countsPerRev =560;
    double pi = 3.1415927;
    double wheelDiameter = 3;
    double countsPerInch = countsPerRev/(wheelDiameter*pi);
    int encoderCounts ;
    int deltaPos;
    ElapsedTime myStopwatch = new ElapsedTime();




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
    static final int ELEV_SCORE_MED = 150;
    static final int ELEV_SCORE_HIGH = 0;
    static final int ELEV_SCORE_VERYHIGH = -600;
    static final int ELEV_TOP = -600;

    static final double ELBOW_UP = 0.11;        //
    static final double ELBOW_DOWN = 0.79;      //was .75 too little, .9 too much
    static final double ELBOW_HANG = 0.36;      //
    static final double ELBOW_SCORE_LOW = 0.17;        //
    static final double ELBOW_SCORE_MED = 0.17;        //
    static final double ELBOW_SCORE_HIGH = 0.21;        //
    static final double ELBOW_SCORE_VERYHIGH = 0.25;        //

    static final double WRIST_HOME = 0.15;
    static final double WRIST_FLOOR_PICKUP = 0.9;   //.9 too low
    static final double WRIST_SCORE_LOW = 0.47;
    static final double WRIST_SCORE_MED = 0.50;
    static final double WRIST_SCORE_HIGH = 0.5;  // was .35
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
        frontLeftMotor = hardwareMap.dcMotor.get("FrtLtMtr");
        backLeftMotor = hardwareMap.dcMotor.get("BckLtMtr");
        frontRightMotor = hardwareMap.dcMotor.get("FrtRtMtr");
        backRightMotor = hardwareMap.dcMotor.get("BckRtMtr");

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

        // TA DONE: Configure HW so that names match
        leftElevator   = hardwareMap.get(DcMotor.class, "LtElevator");
        rightElevator  = hardwareMap.get(DcMotor.class, "RtElevator");
        leftElbow  = hardwareMap.get(Servo.class, "LtElbow");
        rightElbow = hardwareMap.get(Servo.class, "RtElbow");
        leftWrist  = hardwareMap.get(Servo.class, "LtWrist");
        rightWrist = hardwareMap.get(Servo.class, "RtWrist");
        leftClaw   = hardwareMap.get(Servo.class, "LtClaw");
        rightClaw  = hardwareMap.get(Servo.class, "RtClaw");

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
        boolean moveBack = false;
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

            while (opModeIsActive()) {

                // These button choices were made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.options) {
                    imu.resetYaw();
                    initialHeading = 0.0;
                    botHeading = 0.0;
                }

                if (gamepad1.start) {
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

                if (gamepad2.start) {
                    leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sleep(50);
                    leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    sleep(50);
                }

                //small forward move
                if( (gamepad1.y)  ) {
                    driveStraight(3);
                }

                //small backward move
                if( (gamepad1.a)  ) {
                    driveStraight(-3);
                }

                //small backward move
                if( (gamepad1.x)  ) {
                    strafe(-3);
                }

                //small backward move
                if( (gamepad1.b)  ) {
                    strafe(3);
                }


                /////////////////////////////////////////////////////////////////////////
                // Start of Mecanum Drive Section of Code
                /////////////////////////////////////////////////////////////////////////
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                //Set joystick dead bands
                // TA TODO: test to optimize this empirical constant
                double db = 0.07;
                if ((y < db) && (y > -db)) y = 0;
                if ((x < db) && (x > -db)) x = 0;
                if ((rx < db) && (rx > -db)) rx = 0;

                //////////////////////////////////////////////////////////////////////////////////
                // the code section below is correcting for robot rotation when it shouldn't happen
                //////////////////////////////////////////////////////////////////////////////////
//                if (rx == 0.0) isRotating = false;
//                else {
//                    isRotating = true;
//                    firstTime = true;
//                }
//
//                if ((!isRotating) && (firstTime)) {
//                    initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//                    firstTime = false;
//                }
//
//                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//                deltaHeading = botHeading - initialHeading;
//                if (deltaHeading > 340.0) deltaHeading = deltaHeading -360;
//                if (deltaHeading < -340.0) deltaHeading = (360 + deltaHeading);
                // TA TODO: test to optimize this empirical constant and polarity of deltaHeading
                //////////////////////////////////////////////////////////////////////////////////
                //////////////////////////////////////////////////////////////////////////////////

                // Rotate the movement direction counter to the robot's rotation
                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                //////////////////////////////////////////////////////////////////////////////////
                // the code section below is for driver overrides of normal control
                //////////////////////////////////////////////////////////////////////////////////


                //  slow down for more accurate movement when a trigger is depressed
                if ( (gamepad1.right_trigger > 0.33) || (gamepad1.left_trigger > 0.33) ){
                    y = y / 2.0;
                    x = x / 2.0;
                    rx = rx / 2.0;
                }

                // slow down AND reverse all directions (the back of the robot is now the front)
                if ( (gamepad1.left_bumper) || (gamepad1.right_bumper) ){
                    y = -y / 2.0;
                    x = -x / 2.0;
                    rx = -rx / 2.0;
                }

                // switch to robot oriented (normally field oriented),
                if(gamepad1.dpad_up) {
                    fieldCentric = false;
                }

                // switch to robot oriented (normally field oriented),
                if(gamepad1.dpad_down) {
                    fieldCentric = true;
                }

                // switch to robot oriented (normally field oriented),
                if(!fieldCentric) {
                    botHeading = 0.0;
                    deltaHeading =0.0;
                }

// Comment out trying to fix erroneous robot rotation - caused robot to spin around sometimes
//                if ((!isRotating) && (Math.abs(deltaHeading) > 2.2))// DEGREES
//                {
//                    rx = rx + deltaHeading * .02;
//                }
//
                //////////////////////////////////////////////////////////////////////////////////
                //////////////////////////////////////////////////////////////////////////////////

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
                }

                //HANG
                if( (gamepad2.y)  ) {
                    startEvelCmd = true;
                    elevTragetPos = ELEV_HANG;
                    elbowTargetPos = ELBOW_HANG;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_HOME);
                    rightWrist.setPosition(WRIST_HOME);
                }

                //FLOOR PICKUP
                if( (gamepad2.a)  ) {
                    startEvelCmd = true;
                    elevTragetPos = ELEV_FLOOR_PICKUP;
                    elbowTargetPos = ELBOW_DOWN;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_FLOOR_PICKUP);
                    //TA TODO: right side is higher in the air - try to push down a little w/ .02 offset
                    rightWrist.setPosition(WRIST_FLOOR_PICKUP+.02);
                }

                //STARTING POSITION
                if( (gamepad2.b)  ) {
                    startEvelCmd = true;
                    elevTragetPos = ELEV_START;
                    elbowTargetPos = ELBOW_DOWN;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_HOME);
                    rightWrist.setPosition(WRIST_HOME);
                }

                //Use DPAD for 4 scoring positions
                 // DPAD DOWN = Score Low
                if(gamepad2.dpad_down){
    //                driveStraight(3);
                    startEvelCmd = true;
                    elevTragetPos = ELEV_SCORE_LOW;
                    elbowTargetPos = ELBOW_SCORE_LOW;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_SCORE_LOW);
                    rightWrist.setPosition(WRIST_SCORE_LOW);
                    moveBack = true;
                }

                //DPAD LEFT = Score Med
                if(gamepad2.dpad_left){
  //                  driveStraight(3);
                    startEvelCmd = true;
                    elevTragetPos = ELEV_SCORE_MED;
                    elbowTargetPos = ELBOW_SCORE_MED;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_SCORE_MED);
                    rightWrist.setPosition(WRIST_SCORE_MED);
                    moveBack = true;
                }

                // DPAD UP = Score High
                if(gamepad2.dpad_up){
//                    driveStraight(3);
                    startEvelCmd = true;
                    elevTragetPos = ELEV_SCORE_HIGH;
                    elbowTargetPos = ELBOW_SCORE_HIGH;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_SCORE_HIGH);
                    rightWrist.setPosition(WRIST_SCORE_HIGH);
                    moveBack = true;
                }
                // DPAD RIGHT = Score Very High
                if(gamepad2.dpad_right){
//                    driveStraight(3);
                    startEvelCmd = true;
                    elevTragetPos = ELEV_SCORE_VERYHIGH;
                    elbowTargetPos = ELBOW_SCORE_VERYHIGH;
                    currentState = 0;
                    leftWrist.setPosition(WRIST_SCORE_VERYHIGH);
                    rightWrist.setPosition(WRIST_SCORE_VERYHIGH);
                    moveBack = true;
                }

// the code segment below adjusts the robot position toward backboard automatically
// NOT WANTED BY DRIVE TEAM RIGHT NOW, BUT KEEP JUST IN CASE NEEDED
//                if (moveBack == true) {
//                    if (currentState == 3) driveStraight(3);;
//                    if (currentState == 99) {driveStraight(-3); moveBack = false;}
//                }


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


///////////////////////////////////////////////////////////////////////////
// automated driving functions
///////////////////////////////////////////////////////////////////////////




    void autoMec(double y, double x, double rx) {

        telemetry.update();


        y=.75*y;
        x=.75*x;
        rx=.75*rx;

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
//        if ((!isRotating) && (Math.abs(deltaHeading) > 2.2))// DEGREES
//        {
//            rx = rx + deltaHeading * .02;
//        }
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


    void driveStraight(double inches){

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(50);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderCounts = (int)(countsPerInch * inches);

        do  {
            curPos = frontLeftMotor.getCurrentPosition();;
            deltaPos = encoderCounts - curPos ;
            unsignedDelta = Math.abs(deltaPos);
            //full speed
            if (unsignedDelta > 10*countsPerInch) autoMec((double) deltaPos / unsignedDelta, 0.0,  0.0 );
                // slow down last 10 inches
            else if (unsignedDelta > 2*countsPerInch) autoMec((double) deltaPos /(double) (10*countsPerInch), 0.0, 0.0  );
                // min speed of .2 when real close
            else autoMec(0.2*(double) deltaPos/ unsignedDelta, 0.0, 0.0  );
        } while (unsignedDelta > (double) countsPerInch/2);


        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(100);
    }


    void strafe(double inches){

        encoderCounts = (int)(countsPerInch * inches);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(50);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        do  {
            curPos = frontLeftMotor.getCurrentPosition();;
            deltaPos = encoderCounts - curPos ;
            unsignedDelta = Math.abs(deltaPos);
            if (unsignedDelta > 10*countsPerInch) autoMec(0.0,(double) deltaPos/ unsignedDelta,   0.0 );
            else if (unsignedDelta > 2*countsPerInch) autoMec(0.0,(double) deltaPos/(double) (10*countsPerInch),  0.0  );
            else autoMec(0.2*(double) deltaPos/ unsignedDelta, 0.0, 0.0  );
        } while (unsignedDelta > (double) countsPerInch/2);


        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(100);
    }


}   // end of RobotController Class