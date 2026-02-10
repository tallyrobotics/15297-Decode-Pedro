package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class Lime extends NextFTCOpMode {
    Limelight3A limelight;
    private String limeName;
    private Integer numOfBalls = 0;

    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);
    }

    @Override
    public void onUpdate(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            for (LLResultTypes.DetectorResult detection : detections) {
                String className = detection.getClassName(); // What was detected
                double x = detection.getTargetXDegrees(); // Where it is (left-right)
                double y = detection.getTargetYDegrees(); // Where it is (up-down)
                telemetry.addData(className, "at (" + x + ", " + y + ") degrees");
                numOfBalls++;

            }

        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.addData("Balls", numOfBalls);
        telemetry.update();
        numOfBalls = 0;
    }

}
