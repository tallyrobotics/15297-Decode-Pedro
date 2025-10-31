package org.firstinspires.ftc.teamcode.legacy.subsystems;


import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;

import org.firstinspires.ftc.robotcore.external.Supplier;

import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.delegates.Velocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class flyRight implements Subsystem {

    public static final flyRight INSTANCE = new flyRight();

    private flyRight() {
    }

    public MotorEx intake;
    public String intakeName = "intake";
    public Double RPM = 1500.0;

//    public Command flyRightOn()
//    {
//    }

    public Command flyRightOff() {
        return new SetPower(intake, 0);
    }

    public Command flyRightSetRPM() {
        return new SetPower(intake, -1);
    }

    @Override
    public void initialize(){
        intake = new MotorEx(intakeName);
        intake.setDirection(-1);
    }

    @Override
    public void periodic()
    {

    }



}
