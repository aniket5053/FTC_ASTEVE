package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@Autonomous(name = "FunctionsTest")
public class FunctionsTest extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;

    static final long MS_PER_INCH = 18;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor leftElevator;
    private DcMotor rightElevator;
    private Servo leftElbow;
    private Servo rightElbow;
    private Servo leftWrist;
    private Servo leftClaw;
    private Servo rightClaw;


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FrtLtMtr");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BckLtMtr");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FrtRtMtr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BckRtMtr");
        // TA TODO: Configure HW so that names match
        DcMotor leftElevator   = hardwareMap.get(DcMotor.class, "LtElevator");
        DcMotor rightElevator  = hardwareMap.get(DcMotor.class, "RtElevator");
        Servo   leftElbow  = hardwareMap.get(Servo.class, "LtElbow");
        Servo   rightElbow = hardwareMap.get(Servo.class, "RtElbow");
        Servo   leftWrist  = hardwareMap.get(Servo.class, "LtWrist");
//        Servo   rightWrist = hardwareMap.get(Servo.class, "RtWrist");
        Servo   leftClaw   = hardwareMap.get(Servo.class, "LtClaw");
        Servo   rightClaw  = hardwareMap.get(Servo.class, "RtClaw");

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



        waitForStart();

        //move 4 ft
        forward(48);

        //turn 90
        turnRight(90);


        //move wrist to drop pixel
        leftWrist.setPosition(degreesToPosition(0));

        //open both claws
        clawsDegrees(0);

        //bring wrist back up
        leftWrist.setPosition(degreesToPosition(180));

        //move elbow up
        elbowDegrees(0.1);

        //turn left 45
        turnLeft(45);


        sleep(3000);

    }


    void turnLeft(double angle){

        // Calculate the number of movements needed to reach the target angle
        double movementsNeeded = Math.abs(angle / 90);

        // Calculate the adjusted sleep time based on the target angle
        double adjustedSleepTime = 650 * movementsNeeded;
        frontLeftMotor.setPower(-0.5);
        frontRightMotor.setPower(0.5);
        sleep((long) adjustedSleepTime);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        sleep(500);
    }
    void turnRight(double angle){

        // Calculate the number of movements needed to reach the target angle
        double movementsNeeded = Math.abs(angle / 90);

        // Calculate the adjusted sleep time based on the target angle
        double adjustedSleepTime = 650 * movementsNeeded;
        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(-0.5);
        sleep((long) adjustedSleepTime);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        sleep(500);
    }
    void forward(double inches){
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 55);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000 * movementsNeeded;

        // Set power to the motors for moving forward
        frontLeftMotor.setPower(1);
        frontRightMotor.setPower(1);

        // Sleep for the adjusted time
        sleep((long) adjustedSleepTime);

        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        sleep(500);
    }

    void backward(double inches){
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 55);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000 * movementsNeeded;

        // Set power to the motors for moving backward
        frontLeftMotor.setPower(-1);
        frontRightMotor.setPower(-1);

        // Sleep for the adjusted time
        sleep((long) adjustedSleepTime);

        // Stop the motors after the sleep
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        sleep(500);
    }

    private double degreesToPosition(double degrees) {
        // Assuming your servo has a range of motion of 180 degrees
        double minDegrees = 0.0;
        double maxDegrees = 180.0;

        // Convert degrees to the servo position scale (0 to 1)
        return (degrees - minDegrees) / (maxDegrees - minDegrees);
    }
    private void elbowDegrees(double degrees) {
        double position = degreesToPosition(degrees);

        leftElbow.setPosition(position);
        rightElbow.setPosition(position);

        sleep(1000);
    }

    public void clawsDegrees(double degrees){
        double position = degreesToPosition(degrees);

        leftClaw.setPosition(position);
        rightClaw.setPosition(position);

        sleep(1000);

    }


}