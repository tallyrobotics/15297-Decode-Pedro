package org.firstinspires.ftc.teamcode.legacy.subsystems;

//import androidx.annotation.NonNull;
//import dev.nextftc.control.ControlSystem;
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.commands.utility.InstantCommand;
//import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.hardware.controllable.RunToVelocity;
//import dev.nextftc.hardware.impl.MotorEx;

public class flyRightShooter extends flyShooter { //} Subsystem {

    public static final flyRightShooter INSTANCE = new flyRightShooter("flyRight", true);

    public flyRightShooter(String motorName, boolean reverseMotor) {
        super(motorName, reverseMotor);
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
