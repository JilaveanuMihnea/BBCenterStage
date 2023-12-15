package org.firstinspires.ftc.teamcode.cod;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="pixel", group = "Version1")
public class Pixel extends OpMode {
    public java.util.List<Recognition> getRecognitions() {
        return null;
    }

    private static final String[] LABELS = {
            "blue_prop",
    };

    private static final String TFOD_MODEL_ASSET = "blueprop.tflite";
    TfodProcessor tfod;
    public void init(){
        initTfod();
    }

    public void loop(){
        telemetryTfod();
    }
    private void initTfod(){
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

// Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


// Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

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
    private void telemetryTfod() {

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
        }   // end for() loop

    }
}