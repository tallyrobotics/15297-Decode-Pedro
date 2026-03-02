package org.firstinspires.ftc.teamcode.legacy.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    private int targetRPM;

    public Command on() {
//        targetRPM = 750;
        return new SetPower(intake, 1.0);
    }

    public Command off() {
//        targetRPM = 0;
        return new SetPower(intake, 0.0);
    }
    public Command out(){
        return new SetPower(intake, -0.3);
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
