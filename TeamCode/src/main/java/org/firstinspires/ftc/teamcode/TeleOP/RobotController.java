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
        Servo   WRIST1      = hardwareMap.get(Servo.class, "WRIST");
        Servo CLAWLEFT = hardwareMap.get(Servo.class, "CLAWLEFT");
        Servo CLAWRIGHT = hardwareMap.get(Servo.class, "CLAWRIGHT");

        TouchSensor TOUCHSENSOR = hardwareMap.get(TouchSensor.class, "TOUCH SENSOR");

        LEFTDRIVE.setDirection(DcMotor.Direction.REVERSE);
        LEFTAXLE.setDirection(DcMotor.Direction.REVERSE);
        ELBOW2.setDirection(Servo.Direction.REVERSE);
        CLAWLEFT.setDirection(Servo.Direction.REVERSE);


        ELBOW1.setPosition(0);
        ELBOW2.setPosition(0);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                boolean allowMotorControl = true;

                LEFTDRIVE.setPower(-gamepad1.left_stick_y);
                RIGHTDRIVE.setPower(-gamepad1.right_stick_y);

                if (TOUCHSENSOR.isPressed()) {
                    LEFTAXLE.setPower(0);
                    RIGHTAXLE.setPower(0);
                    allowMotorControl = false;
                } else {
                    allowMotorControl = true;
                }

                if(allowMotorControl){
                    LEFTAXLE.setPower(gamepad2.left_stick_y);
                    RIGHTAXLE.setPower(gamepad2.left_stick_y);

                }



                if(gamepad2.y) {
                    // move to 0 degrees.
                    ELBOW1.setPosition(0);
                    ELBOW2.setPosition(0.6);
                }
                else if(gamepad2.b) {
                    // move to 90 degrees
                    ELBOW1.setPosition(1);
                    ELBOW2.setPosition(1);
                }
                else if(gamepad2.a){
                    WRIST1.setPosition(0.0);
                }
                else if(gamepad2.x){
                    WRIST1.setPosition(0.5);
                }
                else if(gamepad2.left_bumper){
                    CLAWRIGHT.setPosition(0);
                    CLAWLEFT.setPosition(0);
                }
                else if(gamepad2.right_bumper){
                    CLAWRIGHT.setPosition(1);
                    CLAWLEFT.setPosition(1);
                }
                else if (gamepad2.dpad_left) {
                    CLAWLEFT.setPosition(0);
                }
                else if (gamepad2.dpad_right) {
                    CLAWLEFT.setPosition(1);
                }
                else if (gamepad2.dpad_up) {
                    CLAWRIGHT.setPosition(0);
                }
                else if (gamepad2.dpad_down) {
                    CLAWRIGHT.setPosition(1);
                }




                //ELBOW1.setPosition(-gamepad2.right_stick_y);
                //ELBOW2.setPosition(-gamepad2.right_stick_y);
                //WRIST.setPosition(gamepad2.right_trigger);
                telemetry.addData("Left Power", LEFTDRIVE.getPower());
                telemetry.addData("Right Power", RIGHTDRIVE.getPower());
                telemetry.addData("Left Axle", LEFTAXLE.getPower());
                telemetry.addData("Right Axle", RIGHTAXLE.getPower());
                telemetry.addData("ELBOW1",ELBOW1.getPosition() );
                telemetry.addData("ELBOW2",ELBOW2.getPosition() );
                telemetry.addData("WRIST",WRIST1.getPosition() );
                telemetry.update();
            }
        }
    }
}
