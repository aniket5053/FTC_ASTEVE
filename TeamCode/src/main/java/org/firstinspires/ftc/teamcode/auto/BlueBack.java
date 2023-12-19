package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="BlueBack", group= "Auto")
public class BlueBack extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor leftElevator;
    DcMotor rightElevator;
    Servo leftElbow;
    Servo rightElbow;
    Servo leftWrist;
    Servo leftClaw;

    Servo rightClaw;

    Robot robot = new Robot();
    PixelDetector detector = new PixelDetector(telemetry, "blue");
    @Override
    public void runOpMode() throws InterruptedException {
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

        leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        leftClaw.setPosition(1);
        rightClaw.setPosition(1);
        sleep(1000);


        // Move 4ft forward (4 ft = 48 inches)
//        robot.moveForward(48);
        forward(48);

        // Turn left 90 degrees
        turnLeft(92);

        // Move 4ft forward (6 ft = 72 inches)
 //       robot.moveForward(72);
        forward(90);


        // Turn left 90 degrees
        turnLeft(45);



        leftElbow.setPosition(0.1);
        rightElbow.setPosition(0.1);
        sleep(2000);

             // open left claw
        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0.5);
        sleep(2000);

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

}
