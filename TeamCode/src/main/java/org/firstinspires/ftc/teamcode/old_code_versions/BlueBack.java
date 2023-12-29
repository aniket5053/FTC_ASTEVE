package org.firstinspires.ftc.teamcode.old_code_versions;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.PixelDetector;

@Autonomous(name="BlueBack", group= "Auto")
public class BlueBack extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;

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
    static final int ELEV_FLOOR = 280;    static final int ELEV_HOME = 0;
    static final int ELEV_SCORE_LOW = -50;
    static final int ELEV_TOP = -600;


    // Declare our motors
    // Make sure your ID's match your configuration
    // TA DONE: Configure HW so that names match
    DcMotor frontLeftMotor ;
    DcMotor backLeftMotor ;
    DcMotor frontRightMotor;
    DcMotor backRightMotor ;
    // Retrieve the IMU from the hardware map
    // TA DONE: Configure HW so that names match
    IMU imu ;

    // TA DONE: Configure HW so that names match
    DcMotor leftElevator ;
    DcMotor rightElevator  ;
    Servo   leftElbow  ;
    Servo   rightElbow;
    Servo   leftWrist  ;
    Servo   rightWrist ;
    Servo   leftClaw   ;
    Servo   rightClaw  ;

    //TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "TouchSensor");



    Robot robot = new Robot();
    PixelDetector detector = new PixelDetector(telemetry, "blue");
    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration
        // TA DONE: Configure HW so that names match
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FrtLtMtr");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BckLtMtr");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FrtRtMtr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BckRtMtr");
        // Retrieve the IMU from the hardware map
        // TA DONE: Configure HW so that names match
        IMU imu = hardwareMap.get(IMU.class, "IMU");
        // Adjust the orientation parameters to match your robot
        // TA DONE: Verify IMU orientation matches code below
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

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

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


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

        leftClaw.setPosition(CLAW_CLOSED);
        rightClaw.setPosition(CLAW_CLOSED);
        sleep(500);         // dont move until claw is closed


        // Move 4ft forward (4 ft = 48 inches)
        forward(48.0);

        // Turn left 90 degrees
        turnLeft(92.0);

        // Move 4ft forward (6 ft = 72 inches)
 //       robot.moveForward(72);
        forward(90.0);


        // Turn left 90 degrees
        turnLeft(45.0);



        leftElbow.setPosition(ELBOW_UP);
        rightElbow.setPosition(ELBOW_UP);
        sleep(2000);

             // open left claw
        leftClaw.setPosition(CLAW_OPEN);
        rightClaw.setPosition(CLAW_OPEN);
        sleep(1000);

    }


    void forward(double inches){
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 55.0);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000.0 * movementsNeeded;

        // Set power to the motors for moving forward
        frontLeftMotor.setPower(1.0);
        frontRightMotor.setPower(1.0);
        backLeftMotor.setPower(1.0);
        backRightMotor.setPower(1.0);

        // Sleep for the adjusted time
        sleep((long) adjustedSleepTime);

        // Stop the motors after the sleep
        frontLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
        sleep(500);
    }


    void turnLeft(double angle){

        // Calculate the number of movements needed to reach the target angle
        double movementsNeeded = Math.abs(angle / 90.0);

        // Calculate the adjusted sleep time based on the target angle
        double adjustedSleepTime = 650.0 * movementsNeeded;
        double curAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double ANG_TOL = 2.0;
        if((Math.abs(curAngle)  - angle) <= ANG_TOL) {
            frontLeftMotor.setPower(-0.5);
            frontRightMotor.setPower(0.5);
            backLeftMotor.setPower(-0.5);
            backRightMotor.setPower(0.5);
            //       sleep((long) adjustedSleepTime);
        }
        else {
            frontLeftMotor.setPower(0.0);
            frontRightMotor.setPower(0.0);
            backLeftMotor.setPower(0.0);
            backRightMotor.setPower(0.0);
            //     sleep(500);
        }
    }

}
