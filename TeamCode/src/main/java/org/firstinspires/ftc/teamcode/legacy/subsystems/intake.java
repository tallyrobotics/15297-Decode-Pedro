package org.firstinspires.ftc.teamcode.legacy.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class intake implements Subsystem {

    public static final intake INSTANCE = new intake();

    private intake() {
    }

    public MotorEx intake;
    public String intakeName = "intake";

    public Command IntakeIn() {
        return new SetPower(intake, 1);
    }

    public Command IntakeOff() {
        return new SetPower(intake, 0);
    }

    public Command IntakeOut() {
        return new SetPower(intake, -1);
    }

    @Override
    public void initialize() {
        intake = new MotorEx(intakeName).reversed();
    }

    @Override
    public void periodic() {
    }
}
