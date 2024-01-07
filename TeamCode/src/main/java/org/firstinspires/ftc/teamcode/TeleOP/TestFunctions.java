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

@TeleOp(name = "TestFunctions")
public class TestFunctions extends LinearOpMode {


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
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
    boolean firstTime = true;
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

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FrtLtMtr");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BckLtMtr");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FrtRtMtr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BckRtMtr");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        // TA TODO: test out directions - esp Elevator - it was different in teleop and Auto
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftElevator.setDirection(DcMotor.Direction.REVERSE);
        rightElbow.setDirection(Servo.Direction.REVERSE);
        leftClaw.setDirection(Servo.Direction.REVERSE);
        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();
        if (opModeIsActive()) {


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
                // OPERATOR INPUTS - Determines movement of Elevator, Elbow, Wrist and Claws
                /////////////////////////////////////////////////////////////////////////

                //HOME
                if( (gamepad1.x)  ) {
                    driveStraight(2);
                }

                //HANG
                if( (gamepad1.y)  ) {
                    driveStraight(6);
                }

                //FLOOR PICKUP
                if( (gamepad1.a)  ) {
                    driveStraight(-6);
                }

                //STARTING POSITION
                if( (gamepad1.b)  ) {
                    driveStraight(-2);
                }


                //HOME
                if( (gamepad2.x)  ) {

                    rightElevator.setTargetPosition(-200);
                    leftElevator.setTargetPosition(-200);


                    rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    rightElevator.setPower(0.5);
                    leftElevator.setPower(0.5);

                    driveStraight(20);


                    rightElevator.setPower(0);
                    leftElevator.setPower(0);


                }

                //HANG
                if( (gamepad2.y)  ) {
                    leftClaw.setPosition(0.5);
                    sleep(100);
                    driveStraight(12);
                }

                //FLOOR PICKUP
                if( (gamepad2.a)  ) {
                    driveStraight(-12);
                }

                //STARTING POSITION
                if( (gamepad2.b)  ) {
                    strafe(12);
                }

                if(gamepad2.left_bumper){
                    leftDiagnal(12);
                }
                if(gamepad2.right_bumper){
                    rightDiagnal(12);
                }

                //Use DPAD for 4 scoring positions
                // DPAD DOWN = Score Low
                if(gamepad2.dpad_down){
                    turnToAngle(180);
                }

                //DPAD LEFT = Score Med
                if(gamepad2.dpad_left){
                    turnToAngle(90);
                }

                // DPAD UP = Score High
                if(gamepad2.dpad_up){
                    turnToAngle(0);
                }
                // DPAD RIGHT = Score Very High
                if(gamepad2.dpad_right){
                    turnToAngle(-90);
                }




                /////////////////////////////////////////////////////////////////////////
                // Telemetry
                /////////////////////////////////////////////////////////////////////////
                telemetry.addLine(String.format("Lt/Rt Frt Encdrs: %d  /  %d ",
                        frontLeftMotor.getCurrentPosition(),frontRightMotor.getCurrentPosition()));
                telemetry.addLine(String.format("Lt/Rt Bck Encdrs: %d  /  %d ",
                        backLeftMotor.getCurrentPosition(),backRightMotor.getCurrentPosition()));

                telemetry.addLine(String.format("Lt Elevtator  %d ",
                        leftElevator.getCurrentPosition()));

                telemetry.addLine(String.format("Rt Elevtator  %d ",
                        rightElevator.getCurrentPosition()));
                telemetry.addLine(String.format("Lt/Rt Bck Encdrs: %d  /  %d ",
                        backLeftMotor.getCurrentPosition(),backRightMotor.getCurrentPosition()));
                telemetry.addLine();
                telemetry.addLine(String.format("Cur / Delta Enc: %d  /  %d ",
                        curPos,deltaPos));
                telemetry.addLine();                telemetry.addLine();
                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                telemetry.addLine(String.format("Heading / Error: %5.1f / %5.1f ",
                        botHeading, deltaHeading));
                telemetry.addLine();
                telemetry.addLine(String.format("Cur / Delta Ang: %5.1f  /  %5.1f ",
                        curAngle,deltaAngle));

                telemetry.update();

            }   // end of While - Opmode is active
        }   // end of If Opmode is active
    }   // end of runOpMode Methode




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



    void turnToAngle(double finalAngle){
        double curTime;
        double startAngle =  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        myStopwatch.reset();

        do  {
            curAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            deltaAngle = curAngle - finalAngle;
            unsignedDelta = Math.abs(deltaAngle);
            if (unsignedDelta > 30.0) autoMec(0.0, 0.0,  deltaAngle/unsignedDelta );
            else autoMec(0.0, 0.0,  deltaAngle/30.0 );
            curTime = myStopwatch.time(TimeUnit.MILLISECONDS);
    //    } while ( (unsignedDelta > 2.0) && (curTime < (finalAngle - startAngle)*200.0 ) );
        } while ( (unsignedDelta > 2.0) );

        //Check to see if its worth trying again!!
        startAngle =  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        deltaAngle = (startAngle-finalAngle);
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
            } while ((unsignedDelta > 2.0) );
        }


        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(100);
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
    void setWristOut() {
        leftWrist.setPosition(0.86);
        rightWrist.setPosition(0.86);
    }



    void rightDiagnal(double inches){

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(50);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderCounts = (int)(countsPerInch * inches *1.7);

        do  {
            curPos = frontLeftMotor.getCurrentPosition();;
            deltaPos = encoderCounts - curPos ;
            unsignedDelta = Math.abs(deltaPos);
            //full speed
            if (unsignedDelta > 10*countsPerInch) autoMec((double) deltaPos / unsignedDelta, deltaPos / unsignedDelta,  0.0 );
                // slow down last 10 inches
            else if (unsignedDelta > 2*countsPerInch) autoMec((double) deltaPos /(double) (10*countsPerInch), deltaPos /(double) (10*countsPerInch), 0.0  );
                // min speed of .2 when real close
            else autoMec(0.2*(double) deltaPos/ unsignedDelta, 0.2*(double) deltaPos/ unsignedDelta, 0.0  );
        } while (unsignedDelta > (double) countsPerInch/2);


        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(100);
    }



    void leftDiagnal(double inches){

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(50);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoderCounts = (int)(countsPerInch * inches *1.7);

        do  {
            curPos = frontRightMotor.getCurrentPosition();;
            deltaPos = encoderCounts - curPos ;
            unsignedDelta = Math.abs(deltaPos);
            //full speed
            if (unsignedDelta > 10*countsPerInch) autoMec((double) deltaPos / unsignedDelta, -deltaPos / unsignedDelta,  0.0 );
                // slow down last 10 inches
            else if (unsignedDelta > 2*countsPerInch) autoMec((double) deltaPos /(double) (10*countsPerInch), -deltaPos /(double) (10*countsPerInch), 0.0  );
                // min speed of .2 when real close
            else autoMec(0.2*(double) deltaPos/ unsignedDelta, -0.2*(double) deltaPos/ unsignedDelta, 0.0  );
        } while (unsignedDelta > (double) countsPerInch/2);


        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(100);
    }




}   // end of RobotController Class