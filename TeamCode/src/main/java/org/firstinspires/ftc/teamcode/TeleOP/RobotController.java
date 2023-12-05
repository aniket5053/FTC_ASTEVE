package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "RobotController")
public class RobotController extends LinearOpMode {


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        DcMotor LEFTDRIVE  = hardwareMap.get(DcMotor.class, "LEFT DRIVE");
        DcMotor RIGHTDRIVE = hardwareMap.get(DcMotor.class, "RIGHT DRIVE");
        DcMotor LEFTAXLE   = hardwareMap.get(DcMotor.class, "LEFT AXLE");
        DcMotor RIGHTAXLE  = hardwareMap.get(DcMotor.class, "RIGHT AXLE");
        Servo   ELBOW1     = hardwareMap.get(Servo.class, "ELBOW1");
        Servo   ELBOW2     = hardwareMap.get(Servo.class, "ELBOW2");
        Servo   WRIST      = hardwareMap.get(Servo.class, "WRIST");

        TouchSensor TOUCHSENSOR = hardwareMap.get(TouchSensor.class, "TOUCH SENSOR");

        LEFTDRIVE.setDirection(DcMotor.Direction.REVERSE);
        LEFTAXLE.setDirection(DcMotor.Direction.REVERSE);
        ELBOW2.setDirection(Servo.Direction.REVERSE);

        ELBOW1.setPosition(0);
        ELBOW2.setPosition(0);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                LEFTDRIVE.setPower(-gamepad1.left_stick_y);
                RIGHTDRIVE.setPower(-gamepad1.right_stick_y);

                LEFTAXLE.setPower(-gamepad2.left_stick_y);
                RIGHTAXLE.setPower(-gamepad2.left_stick_y);

                if(gamepad2.y) {
                    // move to 0 degrees.
                    ELBOW1.setPosition(0);
                    ELBOW2.setPosition(0);
                }
                if(gamepad2.b) {
                    // move to 90 degrees
                    ELBOW1.setPosition(0.5);
                    ELBOW2.setPosition(0.5);
                }
                if(gamepad2.a){
                    WRIST.setPosition(0.0);
                }
                if(gamepad2.x){
                    WRIST.setPosition(0.5);
                }

                ELBOW1.setPosition(-gamepad2.right_stick_y);
                ELBOW2.setPosition(-gamepad2.right_stick_y);
                WRIST.setPosition(gamepad2.right_trigger);
                telemetry.addData("Left Power", LEFTDRIVE.getPower());
                telemetry.addData("Right Power", RIGHTDRIVE.getPower());
                telemetry.addData("Left Axle", LEFTAXLE.getPower());
                telemetry.addData("Right Axle", RIGHTAXLE.getPower());
                telemetry.addData("ELBOW1",ELBOW1.getPosition() );
                telemetry.addData("ELBOW2",ELBOW1.getPosition() );
                telemetry.update();
            }
        }
    }
}

