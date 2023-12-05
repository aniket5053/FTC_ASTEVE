package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "EncoderTest (Blocks to Java)")
public class EncoderTest extends LinearOpMode {

    private DcMotor LEFTDRIVE;
    private DcMotor RIGHTDRIVE;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double StartTimes;

        LEFTDRIVE = hardwareMap.get(DcMotor.class, "LEFT DRIVE");
        RIGHTDRIVE = hardwareMap.get(DcMotor.class, "RIGHT DRIVE");

        // Put initialization blocks here.
        LEFTDRIVE.setDirection(DcMotor.Direction.REVERSE);
        LEFTDRIVE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RIGHTDRIVE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LEFTDRIVE.setTargetPosition(100);
        RIGHTDRIVE.setTargetPosition(100);
        LEFTDRIVE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RIGHTDRIVE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        LEFTDRIVE.setPower(0.8);
        RIGHTDRIVE.setPower(0.8);
        StartTimes = getRuntime();
        while (opModeIsActive() && getRuntime() - StartTimes < 1.5) {
            // Put loop blocks here.
            telemetry.update();
        }
        LEFTDRIVE.setPower(0);
        RIGHTDRIVE.setPower(0);
    }
}