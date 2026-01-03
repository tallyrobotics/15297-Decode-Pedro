package org.firstinspires.ftc.teamcode.legacy.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

@Configurable
public abstract class flyShooter implements Subsystem {

    public flyShooter(String motorName, boolean reverseMotor, int offset/*, double p, double i, double d*/) {flyMotorName = motorName; flyReverseMotor = reverseMotor; addage = offset; /*velPidP = p; velPidI = i; velPidD = d;*/}

    public ControlSystem controller;
    public MotorEx flyMotor;
    private final String flyMotorName;
    private final boolean flyReverseMotor;
    private final int addage;
    private static double velPidP;
    private static double velPidI;
    private static double velPidD;

    private double targetRPM = 0;

    public Command flyOff() {
        targetRPM = 0;
        return new NullCommand();
    }

    public Command flySetRPM(double rpm) {
        targetRPM = rpm+addage;
        return new NullCommand();
    }

    public double getTargetRPM() {//TPS() {
        return targetRPM; //TPS;
    }

    @Override
    public void initialize(){
//        controller = ControlSystem.builder()
//                .velSquID(velPidP,velPidI,velPidD)
//                .build();
//        controller.setGoal(new KineticState(0.0));
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
//        controller = ControlSystem.builder()
//                .velSquID(velPidP,velPidI,velPidD)
//                .build();
//        controller.setGoal(new KineticState(0.0, (targetRPM)));
        if (ActiveOpMode.isStarted()) {
            flyMotor.getMotor().setVelocity(targetRPM);
//            flyMotor.setPower(controller.calculate(new KineticState(flyMotor.getCurrentPosition(),flyMotor.getVelocity())));
        }
        else
        {
            flyMotor.setPower(0);
        }
    }
}
