package org.firstinspires.ftc.teamcode.legacy.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class flyLeftShooter implements Subsystem {

    public static final flyLeftShooter INSTANCE = new flyLeftShooter();

    private flyLeftShooter() {
    }

    public MotorEx flyLeft;
    public String flyLeftName = "flyLeft";

    private double targetRPM = 0;

    ControlSystem controlSystem = ControlSystem.builder()
            .angular(AngleType.DEGREES,
                    feedback -> feedback.posPid(0.005, 0.0, 0.05)
            )
            .basicFF()

            .build();





    public Command getDefaultCommand(){
        return new RunToVelocity(controlSystem, targetRPM, 5);
    }

    public Command flyLeftOff(){
        return new InstantCommand(() -> {targetRPM=0;});
    }

    public Command flyLeftSetRPM(double target){
        return new InstantCommand(()-> {targetRPM=target / 60;});
    }

    @Override
    public void initialize(){
        flyLeft = new MotorEx(flyLeftName).reversed();
        flyLeftOff().schedule();
    }

    @Override
    public void periodic()
    {
        flyLeft.setPower(
                controlSystem.calculate(
                        new KineticState(flyLeft.getCurrentPosition(), flyLeft.getVelocity())
                )
        );
    }



}
