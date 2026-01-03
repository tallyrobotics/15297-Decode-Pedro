package org.firstinspires.ftc.teamcode.legacy.subsystems;

import com.bylazar.configurables.annotations.Configurable;

//import androidx.annotation.NonNull;
//import dev.nextftc.control.ControlSystem;
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.commands.utility.InstantCommand;
//import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.hardware.controllable.RunToVelocity;
//import dev.nextftc.hardware.impl.MotorEx;
@Configurable
public class flyRightShooter extends flyShooter {// implements Subsystem {
    private static double velPidP = 0.001;
    private static double velPidI = 0.0;
    private static double velPidD = 0.0;
    public static final flyRightShooter INSTANCE = new flyRightShooter("flyRight", false/*, velPidP, velPidI, velPidD*/);

    public flyRightShooter(String motorName, boolean reverseMotor/*, double p, double i, double d*/) {
        super(motorName, reverseMotor, 0/*, p, i, d*/);
    }
//    public static final flyRightShooter INSTANCE = new flyRightShooter();
//
//    private flyRightShooter() {}
//
//    public MotorEx flyRight;
//    public String flyRightName = "flyRight";
//
//    private double targetTPS = 0;
//
//    public ControlSystem controlSystem = ControlSystem.builder()
//            .velPid(0.011,0,0)
//            .basicFF(0.0005)
//            .build();
//
//    @NonNull
//    public Command getDefaultCommand() {
//        return new RunToVelocity(controlSystem, targetTPS, 5).requires(this);
//    }
//
//    public Command flyRightOff() {
//        return new InstantCommand(()-> targetTPS = 0);
//    }
//
//    public Command flyRightSetRPM(double targetRPM) {
//        return new InstantCommand(() -> targetTPS = ((targetRPM*28)/60)*2.89); // 3:1 = 2.89:1
//    }
//
//    public double getTargetTPS() {
//        return targetTPS;
//    }
//
//    @Override
//    public void initialize() {
//        flyRight = new MotorEx(flyRightName).reversed();
//    }
//
//    @Override
//    public void periodic()
//    {
//        flyRight.setPower(controlSystem.calculate(flyRight.getState()));
//    }
}
