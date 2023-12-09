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


        ELBOW1.setPosition(1);
        ELBOW2.setPosition(1);

        waitForStart();
        if (opModeIsActive()) {

            //LEFTAXLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //RIGHTAXLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            while (opModeIsActive()) {

                LEFTDRIVE.setPower(-gamepad1.left_stick_y);
                RIGHTDRIVE.setPower(-gamepad1.right_stick_y);

                LEFTAXLE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                RIGHTAXLE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                LEFTAXLE.setPower(gamepad2.left_stick_y);
                RIGHTAXLE.setPower(gamepad2.left_stick_y);




                //home
                if(gamepad1.x){
                    WRIST1.setPosition(1);
                    ELBOW1.setPosition(1);
                    ELBOW2.setPosition(1);
                    //move elevator home here
                    LEFTAXLE.setTargetPosition(0);
                    RIGHTAXLE.setTargetPosition(0);
                    LEFTAXLE.setPower(1);
                    RIGHTAXLE.setPower(1);
                    LEFTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    RIGHTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    LEFTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RIGHTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (LEFTAXLE.isBusy() && RIGHTAXLE.isBusy()) {
                        LEFTDRIVE.setPower(-gamepad1.left_stick_y);
                        RIGHTDRIVE.setPower(-gamepad1.right_stick_y);
                    }
                    LEFTAXLE.setPower(0);
                    RIGHTAXLE.setPower(0);

                }

                //underbar crossing
                if(gamepad2.x){
                    ELBOW1.setPosition(1);
                    ELBOW2.setPosition(1);
                    WRIST1.setPosition(0.5);
                    //move elevator down
                    LEFTAXLE.setTargetPosition(250);  //s/b -360
                    RIGHTAXLE.setTargetPosition(250);
                    LEFTAXLE.setPower(1);
                    RIGHTAXLE.setPower(1);
                    LEFTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    RIGHTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    LEFTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RIGHTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (LEFTAXLE.isBusy() && RIGHTAXLE.isBusy()) {
                        LEFTDRIVE.setPower(-gamepad1.left_stick_y);
                        RIGHTDRIVE.setPower(-gamepad1.right_stick_y);
                    }
                    LEFTAXLE.setPower(0);
                    RIGHTAXLE.setPower(0);

                }



                //floor
                else if(gamepad2.a){
                    ELBOW1.setPosition(1);
                    ELBOW2.setPosition(1);
                    WRIST1.setPosition(0);
                    //move elevator down
                    LEFTAXLE.setTargetPosition(250);  //s/b -360
                    RIGHTAXLE.setTargetPosition(250);
                    LEFTAXLE.setPower(1);
                    RIGHTAXLE.setPower(1);
                    LEFTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    RIGHTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    LEFTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RIGHTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (LEFTAXLE.isBusy() && RIGHTAXLE.isBusy()) {
                        LEFTDRIVE.setPower(-gamepad1.left_stick_y);
                        RIGHTDRIVE.setPower(-gamepad1.right_stick_y);
                    }
                    LEFTAXLE.setPower(0);
                    RIGHTAXLE.setPower(0);

                }


                //score high
                else if(gamepad2.y){
                    ELBOW1.setPosition(0);
                    ELBOW2.setPosition(0);
                    WRIST1.setPosition(0.8);
                    //move elevator up
                    LEFTAXLE.setTargetPosition(-700);
                    RIGHTAXLE.setTargetPosition(-700);
                    LEFTAXLE.setPower(1);
                    RIGHTAXLE.setPower(1);
                    LEFTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    RIGHTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    LEFTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RIGHTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (LEFTAXLE.isBusy() && RIGHTAXLE.isBusy()) {
                        LEFTDRIVE.setPower(-gamepad1.left_stick_y);
                        RIGHTDRIVE.setPower(-gamepad1.right_stick_y);
                    }
                    LEFTAXLE.setPower(0);
                    RIGHTAXLE.setPower(0);
                }

                //score low
                else if(gamepad2.b){
                    //1024
                    ELBOW1.setPosition(0);
                    ELBOW2.setPosition(0);
                    WRIST1.setPosition(0.8);
                    //move elevator up
                    LEFTAXLE.setTargetPosition(-50);
                    RIGHTAXLE.setTargetPosition(-50);
                    LEFTAXLE.setPower(1);
                    RIGHTAXLE.setPower(1);
                    LEFTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    RIGHTAXLE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    LEFTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RIGHTAXLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (LEFTAXLE.isBusy() && RIGHTAXLE.isBusy()) {
                        LEFTDRIVE.setPower(-gamepad1.left_stick_y);
                        RIGHTDRIVE.setPower(-gamepad1.right_stick_y);
                    }
                    LEFTAXLE.setPower(0);
                    RIGHTAXLE.setPower(0);
                }

                if(gamepad2.left_bumper){
                    //open claw
                    if(CLAWLEFT.getPosition() == 1){
                        CLAWLEFT.setPosition(0);
                    }
                    else{
                        CLAWLEFT.setPosition(1);
                    }
                }

                else if(gamepad2.right_bumper){
                    //open claw
                    if(CLAWRIGHT.getPosition() == 1){
                        CLAWRIGHT.setPosition(0);
                    }
                    else{
                        CLAWRIGHT.setPosition(1);
                    }
                }
                else if(gamepad2.right_bumper && gamepad2.left_bumper){
                    if(CLAWRIGHT.getPosition() == 1){
                        CLAWRIGHT.setPosition(0);
                        CLAWLEFT.setPosition(0);
                    }
                    else{
                        CLAWRIGHT.setPosition(1);
                        CLAWLEFT.setPosition(1);

                    }

                }


                if(gamepad2.dpad_up){
                    //elbow up
                    ELBOW1.setPosition(0);
                    ELBOW2.setPosition(0);

                }
                if(gamepad2.dpad_down){
                    //elbow down
                    ELBOW1.setPosition(1);
                    ELBOW2.setPosition(1);
                }
                if(gamepad2.dpad_left){
                    //wrist up
                    WRIST1.setPosition(1);
                }
                if(gamepad2.dpad_right){
                    //wrist down
                    WRIST1.setPosition(0);
                }





                telemetry.addData("Left Power", LEFTDRIVE.getPower());
                telemetry.addData("Right Power", RIGHTDRIVE.getPower());
                telemetry.addData("Left Axle", LEFTAXLE.getPower());
                telemetry.addData("Right Axle", RIGHTAXLE.getPower());
                telemetry.addData("ELBOW1",ELBOW1.getPosition() );
                telemetry.addData("ELBOW2",ELBOW2.getPosition() );
                telemetry.addData("WRIST",WRIST1.getPosition() );
                telemetry.addData("Elevator Encoder Value",LEFTAXLE.getCurrentPosition());
                telemetry.addData("Left Drive Encoder", LEFTDRIVE.getCurrentPosition());
                telemetry.addData("RIGHT Drive Encoder", RIGHTDRIVE.getCurrentPosition());


                telemetry.update();
            }
        }
    }
}