package org.firstinspires.ftc.teamcode.legacy.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.bylazar.opmodecontrol.OpModeStatus;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class flyRightShooter implements Subsystem {

    public static final flyRightShooter INSTANCE = new flyRightShooter();

    private flyRightShooter() {
    }

    public MotorEx flyRight;
    public String flyRightName = "flyRight";


    private double targetRPM = 0;


    ControlSystem controlSystem = ControlSystem.builder()
            .angular(AngleType.DEGREES,
                    feedback -> feedback.posPid(0.005, 0.0, 0.05)
            )
            .basicFF()

            .build();


@NonNull
    public Command getDefaultCommand(){
        return new RunToVelocity(controlSystem, targetRPM, 5);
    }

    public Command flyRightOff(){
        return new InstantCommand(() -> {targetRPM=0;});
    }

    public Command flyRightSetRPM(double target){
        return new InstantCommand(()-> {targetRPM=target/60;});
    }

    @Override
    public void initialize(){

        flyRight = new MotorEx(flyRightName).reversed();

        flyRightOff().schedule();

    }

    @Override
    public void periodic()
    {
        flyRight.setPower(
                controlSystem.calculate(
                        new KineticState(flyRight.getCurrentPosition(), flyRight.getVelocity())
                )
        );
    }
}
