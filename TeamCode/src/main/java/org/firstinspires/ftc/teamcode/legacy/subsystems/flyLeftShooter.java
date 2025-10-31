package org.firstinspires.ftc.teamcode.legacy.subsystems;


import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.core.commands.Command;
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

    ControlSystem controlSystem = ControlSystem.builder()
            .angular(AngleType.DEGREES,
                    feedback -> feedback.posPid(0.01, 0.0, 0.05)
            )
            .basicFF()

            .build();





    public Command flyLeftOff() {
        return new SetPower(flyLeft, 0);
    }

    public Command flyLeftSetRPM(int targetRPM) {
        return new RunToVelocity(controlSystem, targetRPM, 25);
    }

    @Override
    public void initialize(){
        flyLeft = new MotorEx(flyLeftName);
        flyLeft.setDirection(-1);
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
