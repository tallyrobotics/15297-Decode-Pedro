package org.firstinspires.ftc.teamcode.legacy.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class flyTestClass implements Subsystem {

    public flyTestClass(){};
    public MotorEx motor;
    public ControlSystem controller;



    public void initialize(){
        motor = ActiveOpMode.hardwareMap().get(MotorEx.class, "flyLeft");
        controller = ControlSystem.builder()
                .velPid(0.001, 0.0, 0.0)
                .build();
        controller.setGoal(new KineticState(0.0));
    }

    public void periodic(){

    }

}
