package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="REDLUE Team Prop", group= "Auto")
public class TEAM_PROP_RED extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;
    DcMotor LEFTDRIVE;
    DcMotor RIGHTDRIVE;
    DcMotor LEFTAXLE;
    DcMotor RIGHTAXLE;
    Servo ELBOW1;
    Servo ELBOW2;
    Servo WRIST1;
    Servo CLAWLEFT;

    Servo CLAWRIGHT;


    PixelDetector detector = new PixelDetector(telemetry, "red");
    @Override
    public void runOpMode() throws InterruptedException {
//        LEFTDRIVE = hardwareMap.get(DcMotor.class, "LEFT DRIVE");
//        RIGHTDRIVE = hardwareMap.get(DcMotor.class, "RIGHT DRIVE");
//        LEFTAXLE = hardwareMap.get(DcMotor.class, "LEFT AXLE");
//        RIGHTAXLE = hardwareMap.get(DcMotor.class, "RIGHT AXLE");
//        ELBOW1 = hardwareMap.get(Servo.class, "ELBOW1");
//        ELBOW2 = hardwareMap.get(Servo.class, "ELBOW2");
//        WRIST1 = hardwareMap.get(Servo.class, "WRIST");
//        CLAWLEFT = hardwareMap.get(Servo.class, "CLAWLEFT");
//        CLAWRIGHT = hardwareMap.get(Servo.class, "CLAWRIGHT");
//
//        LEFTDRIVE.setDirection(DcMotor.Direction.REVERSE);
//        LEFTAXLE.setDirection(DcMotor.Direction.REVERSE);
//        ELBOW2.setDirection(Servo.Direction.REVERSE);
//        CLAWLEFT.setDirection(Servo.Direction.REVERSE);
//
//        LEFTAXLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RIGHTAXLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        waitForStart();
        switch (detector.getLocation()){
            case LEFT:


                break;
            case CENTER:



                break;

            case RIGHT:



                break;
            case NOT_FOUND:





        }
        webcam.stopStreaming();



    }



}
