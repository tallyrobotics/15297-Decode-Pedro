package org.firstinspires.ftc.teamcode.legacy.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public abstract class flyShooter implements Subsystem {

//    public static final flyShooter INSTANCE = new flyShooter();

//    private flyShooter() {}
    public flyShooter(String motorName, boolean reverseMotor, int offset) {flyMotorName = motorName; flyReverseMotor = reverseMotor; addage = offset;}

    public MotorEx flyMotor;
    private final String flyMotorName;
    private final boolean flyReverseMotor;
    private final int addage;

//    private double targetTPS = 0;
    private double targetRPM = 0;

//    public ControlSystem controlSystem = ControlSystem.builder()
////            .velPid(0.011,0,0)
////            .basicFF(0.0005)
//            .velPid(0.0002,0,0)
//            .basicFF(0.00001)
//            .build();

//    @NonNull
//    public Command getDefaultCommand() {
//        return new RunToVelocity(controlSystem, targetTPS, 5).requires(this);
//    }

    public Command flyOff() {
//        targetTPS = 0;
//        return new RunToVelocity(controlSystem, targetTPS, 5).requires(this);
        targetRPM = 0;
        return new NullCommand();
    }

    public Command flySetRPM(double rpm) {
//        return new InstantCommand(() -> targetTPS = ((targetRPM*28)/60)*2.89); // 3:1 = 2.89:1
//        targetTPS = ((targetRPM*28)/60)*2.89; // 3:1 = 2.89:1
//        return new RunToVelocity(controlSystem, targetTPS, 5).requires(this);
        targetRPM = rpm+addage;
        return new NullCommand();
    }

    public double getTargetRPM() {//TPS() {
        return targetRPM; //TPS;
    }

    @Override
    public void initialize() {
        flyMotor = new MotorEx(flyMotorName);
        flyMotor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyMotor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (flyReverseMotor) {
            flyMotor.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    @Override
    public void periodic()
    {
        if (ActiveOpMode.isStarted()) {
//        flyMotor.setPower(controlSystem.calculate(flyMotor.getState()));
            flyMotor.getMotor().setVelocity(targetRPM);
//            ActiveOpMode.telemetry().addData(flyMotorName + " State", flyMotor.getState());
        }
        else
        {
            flyMotor.setPower(0);
        }
    }
}
