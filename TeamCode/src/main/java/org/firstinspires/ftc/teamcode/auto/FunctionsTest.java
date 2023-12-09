package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@Autonomous(name = "FunctionsTest")
public class FunctionsTest extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;

    static final long MS_PER_INCH = 18;
    private DcMotor LEFTDRIVE;
    private DcMotor RIGHTDRIVE;
    private DcMotor LEFTAXLE;
    private DcMotor RIGHTAXLE;
    private Servo ELBOW1;
    private Servo ELBOW2;
    private Servo WRIST1;
    private Servo CLAWLEFT;
    private Servo CLAWRIGHT;


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        LEFTDRIVE = hardwareMap.get(DcMotor.class, "LEFT DRIVE");
        RIGHTDRIVE = hardwareMap.get(DcMotor.class, "RIGHT DRIVE");
        LEFTAXLE = hardwareMap.get(DcMotor.class, "LEFT AXLE");
        RIGHTAXLE = hardwareMap.get(DcMotor.class, "RIGHT AXLE");
        ELBOW1 = hardwareMap.get(Servo.class, "ELBOW1");
        ELBOW2 = hardwareMap.get(Servo.class, "ELBOW2");
        WRIST1 = hardwareMap.get(Servo.class, "WRIST");
        CLAWLEFT = hardwareMap.get(Servo.class, "CLAW LEFT");
        CLAWRIGHT = hardwareMap.get(Servo.class, "CLAW RIGHT");

        LEFTDRIVE.setDirection(DcMotor.Direction.REVERSE);
        LEFTAXLE.setDirection(DcMotor.Direction.REVERSE);
        ELBOW2.setDirection(Servo.Direction.REVERSE);
        CLAWLEFT.setDirection(Servo.Direction.REVERSE);



        waitForStart();

        //move 4 ft
        forward(48);

        //turn 90
        turnRight(90);


        //move wrist to drop pixel
        WRIST1.setPosition(degreesToPosition(0));

        //open both claws
        clawsDegrees(0);

        //bring wrist back up
        WRIST1.setPosition(degreesToPosition(180));

        //move elbow up
        elbowDegrees(0);

        //turn left 45
        turnLeft(45);


    }


    void turnLeft(double angle){

        // Calculate the number of movements needed to reach the target angle
        double movementsNeeded = Math.abs(angle / 90);

        // Calculate the adjusted sleep time based on the target angle
        double adjustedSleepTime = 650 * movementsNeeded;
        LEFTDRIVE.setPower(-0.5);
        RIGHTDRIVE.setPower(0.5);
        sleep((long) adjustedSleepTime);
        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
        sleep(500);
    }
    void turnRight(double angle){

        // Calculate the number of movements needed to reach the target angle
        double movementsNeeded = Math.abs(angle / 90);

        // Calculate the adjusted sleep time based on the target angle
        double adjustedSleepTime = 650 * movementsNeeded;
        LEFTDRIVE.setPower(0.5);
        RIGHTDRIVE.setPower(-0.5);
        sleep((long) adjustedSleepTime);
        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
        sleep(500);
    }
    void forward(double inches){
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 55);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000 * movementsNeeded;

        // Set power to the motors for moving forward
        LEFTDRIVE.setPower(1);
        RIGHTDRIVE.setPower(1);

        // Sleep for the adjusted time
        sleep((long) adjustedSleepTime);

        // Stop the motors after the sleep
        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
        sleep(500);
    }

    void backward(double inches){
        // Calculate the number of movements needed to reach the target distance
        double movementsNeeded = Math.abs(inches / 55);

        // Calculate the adjusted sleep time based on the target distance
        double adjustedSleepTime = 1000 * movementsNeeded;

        // Set power to the motors for moving backward
        LEFTDRIVE.setPower(-1);
        RIGHTDRIVE.setPower(-1);

        // Sleep for the adjusted time
        sleep((long) adjustedSleepTime);

        // Stop the motors after the sleep
        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
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

        ELBOW1.setPosition(position);
        ELBOW2.setPosition(position);

        sleep(1000);
    }

    public void clawsDegrees(double degrees){
        double position = degreesToPosition(degrees);

        CLAWLEFT.setPosition(position);
        CLAWRIGHT.setPosition(position);

        sleep(1000);

    }


}