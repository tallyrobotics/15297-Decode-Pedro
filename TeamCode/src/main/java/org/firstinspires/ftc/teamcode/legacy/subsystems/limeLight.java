package org.firstinspires.ftc.teamcode.legacy.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class limeLight implements Subsystem {

    public static final limeLight INSTANCE = new limeLight();
    public limeLight() {

    }

    public Limelight3A limelight;
    String limename = "limelight";
    Integer balls = 0;
    Integer finalBalls = 0;



    @Override
    public void initialize()
    {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, limename);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0);
    }

    public Command On0(){
        return new InstantCommand(()-> {limelight.start();
        limelight.pipelineSwitch(0);});
    }

    public Command On1(){
        return new InstantCommand(()-> {limelight.start();
        limelight.pipelineSwitch(1);});
    }


    public Command Off(){
        return new InstantCommand(()-> {limelight.stop();});
    }
    public Integer GetBalls(){
        return finalBalls;
    }


    @Override
    public void periodic(){
        if(limelight.isRunning()){
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {

                List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
                for (LLResultTypes.DetectorResult detection : detections) {
                    balls++;
                }

            } else {
                ActiveOpMode.telemetry().addData("Limelight", "No Targets");
            }
        }

        finalBalls = balls;
        ActiveOpMode.telemetry().addData("Balls", balls);
        balls = 0;
    }
}
