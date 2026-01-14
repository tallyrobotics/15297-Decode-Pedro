package org.firstinspires.ftc.teamcode.legacy.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class intake implements Subsystem {

    public static final intake INSTANCE = new intake();

    private intake() {
    }

    public MotorEx intake;
    public String intakeName = "intake";

    public Command IntakeIn() {
        return new SetPower(intake, 0.9);
    }

    public Command IntakeOff() {
        return new SetPower(intake, 0);
    }

    public Command IntakeOut() {
        return new SetPower(intake, -0.4);
    }

    @Override
    public void initialize() {
        intake = new MotorEx(intakeName);
        intake.getMotor().setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void periodic() {

    }
}
