package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "FunctionTestGyro")
public class FunctionTestGyro extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;

    static final long MS_PER_INCH = 18;
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

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.dcMotor.get("FrtLtMtr");
        backLeftMotor = hardwareMap.dcMotor.get("BckLtMtr");
        frontRightMotor = hardwareMap.dcMotor.get("FrtRtMtr");
        backRightMotor = hardwareMap.dcMotor.get("BckRtMtr");
        // TA TODO: Configure HW so that names match
        leftElevator   = hardwareMap.get(DcMotor.class, "LtElevator");
        rightElevator  = hardwareMap.get(DcMotor.class, "RtElevator");
        leftElbow  = hardwareMap.get(Servo.class, "LtElbow");
        rightElbow = hardwareMap.get(Servo.class, "RtElbow");
        leftWrist  = hardwareMap.get(Servo.class, "LtWrist");
        rightWrist = hardwareMap.get(Servo.class, "RtWrist");
        leftClaw   = hardwareMap.get(Servo.class, "LtClaw");
        rightClaw  = hardwareMap.get(Servo.class, "RtClaw");

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



        waitForStart();

        //move 2 ft
        driveStraight(24);
        strafe(24);
        driveStraight(-24);
        strafe(-24);

        driveStraight(12);
        turn(90);
        driveStraight(-12);
        turn(-180);




        sleep(3000);

    }


    void autoMec(double y, double x, double rx) {


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

    }



    void turn(double angle){

        double initAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double curAngle = initAngle;
        double finalAngle = initAngle + angle;
        double deltaAngle, unsignedDelta;
        do  {
            curAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            deltaAngle = finalAngle - curAngle ;
            unsignedDelta = Math.abs(deltaAngle);
            if (unsignedDelta > 10.0) autoMec(0.0, 0.0,  deltaAngle/unsignedDelta );
            else autoMec(0.0, 0.0,  0.25*deltaAngle/unsignedDelta );
        } while (deltaAngle > 2.0);


        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(100);
    }


    void driveStraight(double inches){

        int initPos = 0;
        int curPos = 0;
        int countsPerInch = 100;
        int encoderCounts = (int)(countsPerInch * inches);
        int deltaPos, unsignedDelta;

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
            if (unsignedDelta > 10*countsPerInch) autoMec(deltaPos/unsignedDelta, 0.0,  0.0 );
            else if (unsignedDelta > 2*countsPerInch) autoMec(deltaPos/countsPerInch, 0.0, 0.0  );
            else autoMec(0.2*deltaPos/unsignedDelta, 0.0, 0.0  );
        } while (deltaPos > countsPerInch/2);


        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(100);
    }


    void strafe(double inches){

        int initPos = 0;
        int curPos = 0;
        int countsPerInch = 100;
        int encoderCounts = (int)(countsPerInch * inches);
        int deltaPos, unsignedDelta;

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
            if (unsignedDelta > 10*countsPerInch) autoMec(0.0,deltaPos/unsignedDelta,   0.0 );
            else if (unsignedDelta > 2*countsPerInch) autoMec(0.0,deltaPos/countsPerInch,  0.0  );
            else autoMec(0.2*deltaPos/unsignedDelta, 0.0, 0.0  );
        } while (deltaPos > countsPerInch/2);


        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(100);
    }


}