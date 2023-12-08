package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Pixel Detector", group= "Auto")
public class AutoMode extends LinearOpMode {

    OpenCvCamera webcam;
    private DcMotor LEFTDRIVE;
    private DcMotor RIGHTDRIVE;
    private DcMotor LEFTAXLE;
    private DcMotor RIGHTAXLE;
    private Servo ELBOW1;
    private Servo ELBOW2;
    @Override
    public void runOpMode() throws InterruptedException {
        double StartTimes;

        LEFTDRIVE = hardwareMap.get(DcMotor.class, "LEFT DRIVE");
        RIGHTDRIVE = hardwareMap.get(DcMotor.class, "RIGHT DRIVE");
        LEFTAXLE   = hardwareMap.get(DcMotor.class, "LEFT AXLE");
        RIGHTAXLE  = hardwareMap.get(DcMotor.class, "RIGHT AXLE");
        ELBOW1        = hardwareMap.get(Servo.class, "ELBOW1");
        ELBOW2        = hardwareMap.get(Servo.class, "ELBOW2");

        int cameraMonitorViewId = hardwareMap.appContext.
                getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().
                createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        PixelDetector detector = new PixelDetector(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        waitForStart();
        switch (detector.getLocation()){
            case LEFT:
                LEFTDRIVE.setDirection(DcMotor.Direction.REVERSE);
                LEFTDRIVE.setPower(-0.5);
                RIGHTDRIVE.setPower(0.5);
                sleep(750);
                LEFTDRIVE.setPower(0);
                RIGHTDRIVE.setPower(0);
                break;
            case CENTER:
                LEFTDRIVE.setDirection(DcMotor.Direction.REVERSE);
                LEFTDRIVE.setPower(1);
                RIGHTDRIVE.setPower(1);
                sleep(1000);
                LEFTDRIVE.setPower(0);
                RIGHTDRIVE.setPower(0);
                break;

            case RIGHT:
                LEFTDRIVE.setDirection(DcMotor.Direction.REVERSE);
                LEFTDRIVE.setPower(0.5);
                RIGHTDRIVE.setPower(-0.5);
                sleep(750);
                LEFTDRIVE.setPower(0);
                RIGHTDRIVE.setPower(0);
                break;

            case NOT_FOUND:
                LEFTDRIVE.setPower(0);
                RIGHTDRIVE.setPower(0);


        }
        webcam.stopStreaming();

    }

}
