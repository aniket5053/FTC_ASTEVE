package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="BLUE FRONT Team Prop", group= "Auto")
public class TEAM_PROP_BLUE_FRONT extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;

    static final double CLAW_OPEN = 0.5;
    static final double CLAW_CLOSED = 1.0;
    static final double ELBOW_UP = 0.15;        //was .25 too little, .15 too much
    static final double ELBOW_DOWN = 0.79;      //was .75 too little, .9 too much
    static final double ELBOW_HANG = 0.36;      //
    static final double ELBOW_SCORE_LOW = ELBOW_UP;
    static final double WRIST_HOME = 0.15;
    static final double WRIST_OUT = 0.86;
    static final double WRIST_IN = 0.05;
    static final double WRIST_FLOOR_PICKUP = 0.85;   //.9 too low
    static final double WRIST_SCORE_HIGH = 0.57;
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

    PixelDetector detector = new PixelDetector(telemetry, "blue");
    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration
        // TA DONE: Configure HW so that names match
        frontLeftMotor = hardwareMap.get(DcMotor.class,"FrtLtMtr");
        backLeftMotor = hardwareMap.get(DcMotor.class,"BckLtMtr");
        frontRightMotor = hardwareMap.get(DcMotor.class,"FrtRtMtr");
        backRightMotor = hardwareMap.get(DcMotor.class,"BckRtMtr");

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

        leftElevator   = hardwareMap.get(DcMotor.class, "LtElevator");
        rightElevator  = hardwareMap.get(DcMotor.class, "RtElevator");
        leftElbow  = hardwareMap.get(Servo.class, "LtElbow");
        rightElbow = hardwareMap.get(Servo.class, "RtElbow");
        leftWrist  = hardwareMap.get(Servo.class, "LtWrist");
        rightWrist = hardwareMap.get(Servo.class, "RtWrist");
        leftClaw   = hardwareMap.get(Servo.class, "LtClaw");
        rightClaw  = hardwareMap.get(Servo.class, "RtClaw");

        //TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "TouchSensor");

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

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
        switch (detector.getLocation()){
            case LEFT:
                leftClaw.setPosition(CLAW_CLOSED);
                sleep(250);
                rightClaw.setPosition(CLAW_CLOSED);
                sleep(250);
                strafeLeft(38);
                forward(24);
                turnRight(90);
                setWristOut();
                forward(7);
                leftClaw.setPosition(CLAW_OPEN);
                sleep(250);
                setWristIn();
                sleep(250);
                score();
                backward(15);
                rightClaw.setPosition(CLAW_OPEN + 0.2);
                sleep(250);
                home();
                strafeRight(24);


                break;

            case CENTER:
                leftClaw.setPosition(CLAW_CLOSED);
                sleep(250);
                rightClaw.setPosition(CLAW_CLOSED);
                sleep(250);
                setWristOut();
                sleep(100);
                forward(24);
                //backward(12);
                //setWristOut();
                //drops on center Spike Mark
                leftClaw.setPosition(CLAW_OPEN);
                sleep(250);
                setWristIn();
                backward(5);
                turnRight(90);
                backward(33);
                score();
                backward(6);
                strafeLeft(13);
                //drops on center area
                rightClaw.setPosition(CLAW_OPEN + 0.2);
                sleep(250);
                home();
                strafeRight(24);


                break;

            case RIGHT:
                leftClaw.setPosition(CLAW_CLOSED);
                sleep(250);
                rightClaw.setPosition(CLAW_CLOSED);
                sleep(250);
                forward(30);
                turnRight(90);
                setWristOut();
                //drops on right spike mark
                leftClaw.setPosition(CLAW_OPEN);
                setWristIn();
                backward(35);
                score();
                backward(8);
                //drops on right area
                rightClaw.setPosition(CLAW_OPEN + 0.2);
                sleep(250);
                home();
                strafeRight(15);
                break;

            case NOT_FOUND:





        }
        webcam.stopStreaming();



    }



    void forward(double inches){
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 37);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000 * movementsNeeded;

        // Set power to the motors for moving forward
        frontLeftMotor.setPower(1);
        frontRightMotor.setPower(1);
        backLeftMotor.setPower(1);
        backRightMotor.setPower(1);

        // Sleep for the adjusted time
        sleep((long) adjustedSleepTime);

        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(1000);
    }


    void backward(double inches){
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 37);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000 * movementsNeeded;

        // Set power to the motors for moving backward
        frontLeftMotor.setPower(-1);
        frontRightMotor.setPower(-1);
        backLeftMotor.setPower(-1);
        backRightMotor.setPower(-1);

        // Sleep for the adjusted time
        sleep((long) adjustedSleepTime);

        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(1000);
    }

    void turnLeft(double angle){

        // Calculate the number of movements needed to reach the target angle
        double movementsNeeded = Math.abs(angle /45);

        // Calculate the adjusted sleep time based on the target angle
        double adjustedSleepTime = 650 * movementsNeeded;

        frontLeftMotor.setPower(-0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(-0.5);
        backRightMotor.setPower(0.5);

        sleep((long) adjustedSleepTime);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(500);
    }

    void turnRight(double angle){

        // Calculate the number of movements needed to reach the target angle
        double movementsNeeded = Math.abs(angle /45);

        // Calculate the adjusted sleep time based on the target angle
        double adjustedSleepTime = 650 * movementsNeeded;

        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(-0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(-0.5);

        sleep((long) adjustedSleepTime);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(500);
    }
    void strafeRight(double inches){
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 37);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000 * movementsNeeded;

        frontLeftMotor.setPower(1);
        frontRightMotor.setPower(-1);
        backLeftMotor.setPower(-1);
        backRightMotor.setPower(1);

        sleep((long) adjustedSleepTime);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(500);

    }
    void strafeLeft(double inches){
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 37);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000 * movementsNeeded;

        frontLeftMotor.setPower(-1);
        frontRightMotor.setPower(1);
        backLeftMotor.setPower(1);
        backRightMotor.setPower(-1);

        sleep((long) adjustedSleepTime);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        sleep(500);

    }


    void setElbowUp(){
        leftElbow.setPosition(ELBOW_UP + 0.4);
        rightElbow.setPosition(ELBOW_UP + 0.4);
        sleep(250);
        leftElbow.setPosition(ELBOW_UP + 0.2);
        rightElbow.setPosition(ELBOW_UP + 0.2);
        sleep(250);
        leftElbow.setPosition(ELBOW_UP);
        rightElbow.setPosition(ELBOW_UP);
    }

    void setElbowDown(){
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

    void setWristOut(){
        leftWrist.setPosition(WRIST_OUT);
        rightWrist.setPosition(WRIST_OUT);
        sleep(1000);
    }
    void setWristIn(){
        leftWrist.setPosition(WRIST_IN);
        rightWrist.setPosition(WRIST_IN);
        sleep(1000);
    }

    void setWristScoreHigh(){
        leftWrist.setPosition(WRIST_SCORE_HIGH);
        rightWrist.setPosition(WRIST_SCORE_HIGH);
    }


    void setWristScoreLow(){
        leftWrist.setPosition(WRIST_SCORE_LOW);
        rightWrist.setPosition(WRIST_SCORE_LOW);
    }

    void score(){
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
        sleep(2000);
    }

    void home(){
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
        sleep(1500);
    }



}

