package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "EncoderTest")
public class EncoderTest extends LinearOpMode {

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {


        DcMotor LEFTDRIVE = hardwareMap.get(DcMotor.class, "LEFT DRIVE");
        DcMotor RIGHTDRIVE = hardwareMap.get(DcMotor.class, "RIGHT DRIVE");

        waitForStart();

        LEFTDRIVE.setPower(1);
        RIGHTDRIVE.setPower(1);

        sleep(1000);

        LEFTDRIVE.setPower(-0.5);
        RIGHTDRIVE.setPower(0.5);
        sleep(750);

        LEFTDRIVE.setPower(0.5);
        RIGHTDRIVE.setPower(-0.5);
        sleep(750);

        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
        sleep(1000);


    }
}