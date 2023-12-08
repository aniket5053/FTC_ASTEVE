package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "EncoderTest")
public class EncoderTest extends LinearOpMode {

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {


        DcMotor LEFTDRIVE = hardwareMap.get(DcMotor.class, "LEFT DRIVE");
        DcMotor RIGHTDRIVE = hardwareMap.get(DcMotor.class, "RIGHT DRIVE");
        Servo ELBOW1     = hardwareMap.get(Servo.class, "ELBOW1");
        Servo   ELBOW2     = hardwareMap.get(Servo.class, "ELBOW2");
        LEFTDRIVE.setDirection(DcMotor.Direction.REVERSE);
        //LEFTAXLE.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        //moves roughly 55 inches
        LEFTDRIVE.setPower(1);
        RIGHTDRIVE.setPower(1);
        sleep(1000);

        //stops
        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
        sleep(500);

        //left 90 degrees
        LEFTDRIVE.setPower(-0.5);
        RIGHTDRIVE.setPower(0.5);
        sleep(650);

        LEFTDRIVE.setPower(0.5);
        RIGHTDRIVE.setPower(-0.5);
        sleep(650);

        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
        sleep(1000);


    }
}