package org.firstinspires.ftc.teamcode.cod;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Mat;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="ScheletRosu", group="Version1")
@Disabled
public abstract class ScheletRosu extends LinearOpMode {
    Hardware hardware;
    protected ElapsedTime runTime = new ElapsedTime();

    private boolean clawStateLeft = true;
    private boolean clawStateRight = true;

    public java.util.List<Recognition> getRecognitions() {
        return null;
    }

    private static final String[] LABELS = {
            "red_tear",
    };

    private static final String TFOD_MODEL_ASSET = "redtear.tflite";
    TfodProcessor tfod;

    protected void init_auto()
    {
        hardware = new Hardware(hardwareMap, true);
        runTime = new ElapsedTime();
        hardware.clawServoLeft.setPosition(Spec.CLOSED_POS_LEFT);
        hardware.clawServoRight.setPosition(Spec.CLOSED_POS_RIGHT);
        hardware.clawServoHold.setPosition(Spec.HOLD_SAFE);
        hardware.droneServo.setPosition(Spec.DRONE_HOLD);
        initTfod();
    }
    private void initTfod(){
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .setMaxNumRecognitions(1)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

// Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

// Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

// Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

// Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

// Choose whether or not LiveView stops if no processors are enabled.
// If set "true", monitor shows solid orange screen if no processors enabled.
// If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

// Set and enable the processor.
        builder.addProcessor(tfod);

// Build the Vision Portal, using the above settings.
        VisionPortal visionPortal = builder.build();
    }
    protected void telemetryTfod() {

        java.util.List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();
        }   // end for() loop

    }
    protected void sliderAuto(int ticks){
        hardware.sliderMotor.setTargetPosition(ticks);
        hardware.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(ticks>hardware.sliderMotor.getCurrentPosition())
            hardware.sliderMotor.setPower(Spec.SLIDER_SPEED_UP);
        else
            hardware.sliderMotor.setPower(Spec.SLIDER_SPEED_DOWN);
    }
    protected void tagaAuto(int ticks, float speed_multi){
        hardware.tagaMotor.setTargetPosition(ticks);
        hardware.tagaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.tagaMotor.setPower(Spec.TAGA_SPEED_AUTO*speed_multi);
        if(hardware.tagaMotor.getCurrentPosition() > ticks-100)       // slow pe final, de testat daca merge
            hardware.tagaMotor.setPower(0.2f);
    }

    private void clawOpen(boolean rl){
        if(!rl){
            if(clawStateRight){
                hardware.clawServoRight.setPosition(Spec.OPENED_POS_RIGHT);
            }else{
                hardware.clawServoRight.setPosition(Spec.CLOSED_POS_RIGHT);
            }
            clawStateRight = !clawStateRight;
        }else{
            if(clawStateLeft){
                hardware.clawServoLeft.setPosition(Spec.OPENED_POS_LEFT);
            }else{
                hardware.clawServoLeft.setPosition(Spec.CLOSED_POS_LEFT);
            }
            clawStateLeft = !clawStateLeft;
        }
    }

    protected void clawOpenLeft(){
        clawOpen(true);
    }

    protected void clawOpenRight(){
        clawOpen(false);
    }

    protected void clawOpenBoth(){
        clawOpenRight();
        clawOpenLeft();
    }
    protected void stackGrabRed(int ticks){
        tagaAuto(ticks, 1f);
        sleep(400);
        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
        sleep(300);
        tagaFixativ();
        clawOpenBoth();
        sleep(200);
        tagaAuto(0, 0.8f);
    }

    protected void placesqc(int ticks){
        tagaAuto(ticks, 0.7f);
        sleep(1000);
        hardware.clawServoHold.setPosition(Spec.HOLD_PLACE);//+((hardware.tagaMotor.getCurrentPosition()-870)/8.33-30)/355);
        sleep(600);
        tagaFixativ();
        clawOpenBoth();
        sleep(500);
        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
        clawOpenBoth();
        tagaAuto(0, 0.5f);
        sleep(1500);
        clawOpenBoth();
    }
    protected void up(int ticks){
        tagaAuto(ticks, 1f);
        sleep(1000);
        hardware.clawServoHold.setPosition(Spec.HOLD_PLACE);//+((hardware.tagaMotor.getCurrentPosition()-870)/8.33-30)/355);
        sleep(600);
        tagaFixativ();
    }
    protected void down(){
        hardware.clawServoHold.setPosition(Spec.HOLD_ALIGN);
        clawOpenBoth();
        tagaAuto(0, 1f);
        sleep(1500);
        clawOpenBoth();
    }

    protected void tagaFixativ(){
        int tagaPos = hardware.tagaMotor.getCurrentPosition();
        if(tagaPos >= 900) hardware.tagaMotor.setPower(-0.001f);
        else if(tagaPos < 900) hardware.tagaMotor.setPower(0.001f);
    }


}
