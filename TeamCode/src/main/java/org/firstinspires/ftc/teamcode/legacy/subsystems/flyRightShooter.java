package org.firstinspires.ftc.teamcode.legacy.subsystems;


import com.bylazar.opmodecontrol.OpModeStatus;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.core.commands.Command;
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

    ControlSystem controlSystem = ControlSystem.builder()
            .angular(AngleType.DEGREES,
                    feedback -> feedback.posPid(0.01, 0.0, 0.05)
            )
            .basicFF()

            .build();





    public Command flyRightOff() {
        return new SetPower(flyRight, 0);
    }

    public Command flyRightSetRPM(int targetRPM) {
        return new RunToVelocity(controlSystem, targetRPM, 25);
    }

    @Override
    public void initialize(){
        flyRight = new MotorEx(flyRightName);
        flyRight.setDirection(-1);
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
