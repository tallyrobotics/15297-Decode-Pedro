package org.firstinspires.ftc.teamcode.legacy.subsystems;

//import androidx.annotation.NonNull;
//import dev.nextftc.control.ControlSystem;
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.commands.utility.InstantCommand;
//import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.hardware.controllable.RunToVelocity;
//import dev.nextftc.hardware.impl.MotorEx;

public class flyLeftShooter extends flyShooter {// implements Subsystem {

    public static final flyLeftShooter INSTANCE = new flyLeftShooter("flyLeft", true);

    public flyLeftShooter(String motorName, boolean reverseMotor) {
        super(motorName, reverseMotor, 200);
    }

//    private flyLeftShooter() {}
//
//    public MotorEx flyLeft;
//    public String flyLeftName = "flyLeft";
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
//    public Command flyLeftOff() {
//        return new InstantCommand(()-> targetTPS = 0);
//    }
//
//    public Command flyLeftSetRPM(double targetRPM) {
//        return new InstantCommand(() -> targetTPS = ((targetRPM*28)/60)*2.89); // 3:1 = 2.89:1
//    }
//
//    public double getTargetTPS() {
//        return targetTPS;
//    }
//
//    @Override
//    public void initialize() {
//        flyLeft = new MotorEx(flyLeftName).reversed();
//    }
//
//    @Override
//    public void periodic()
//    {
//        flyLeft.setPower(controlSystem.calculate(flyLeft.getState()));
//    }
}
