package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class FindImageOnCone {
    private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    //private static final String TFOD_MODEL_FILE = "/storage/emulated/0/FIRST/tflitemodels/CustomTeamModel.tflite";
    //private static final String TFOD_MODEL_FILE = "/src/main/assets/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "Bus",
            "Cow",
            "Racket"
    };

    private static final String VUFORIA_KEY =
            "AareZG7/////AAABmTJ0xUnT50AFtHQoZbOhHWJVFYPHhtpRKUPcI002vbLJNDFkStQrbg6q7pDW/WKMU6RihSol/TNPaAwcayfgOp1GtPzns2CKdad5gcGjv5A39kGKK31xdbuL1VM+xlMiOQV/ve4ogqq8+boju02dPWrFSq0iMWgFuuk0CV6R9KXN0S7HnjXIlcbu4brJZ/TBwV28vV7YPPlkt4qsv+qfi1+YDP4vMzM/jfrZZnmFrSpwXbQnwhvCfbDFNWHQRcRbs8stzmlOYhZnKFD1qm2kHalezdQecV3kkqLn8RXV6wv/MNmYLKDB5CwV9Nu0RmiXwjRTiBFr43KPNneDeO532THiRhxC57bXMXNkr+6v/8It";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public FindImageOnCone(Telemetry opModeTelemetry, HardwareMap opModeHardware ) {
        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.5, 16.0/9.0);
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        }
    }
    public String findObject() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            boolean size = false;
            double startTime = System.currentTimeMillis();
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if(updatedRecognitions != null){
                if(updatedRecognitions.size() < 1){
                    size = false;
                } else {size = true;}
            }
            int timeout = 10000;
            while(size == false && (System.currentTimeMillis() - startTime) < timeout){
                updatedRecognitions = tfod.getUpdatedRecognitions();
                if(updatedRecognitions != null){
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());
                    telemetry.update();
                    if(updatedRecognitions.size() < 1){
                        size = false;
                    } else {size = true;}
                }
            }
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                    double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                    telemetry.addData(""," ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                    telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                    return recognition.getLabel();
                }
                telemetry.update();
            }
        }
        return null;
    }
    
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        // tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
