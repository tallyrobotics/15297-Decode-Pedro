package org.firstinspires.ftc.teamcode.legacy.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public abstract class LED implements Subsystem {


    public LED(String ledName, String colorSenseName, String distSenseName) {
        LEDname = ledName;
        colSenName = colorSenseName;
        distanceName = distSenseName;
    }

    public ServoEx led;
    public final String LEDname;
    public final String colSenName;
    public final String distanceName;
    private static final Double off = 0.0;
    private static final Double red = 0.285;
    private static final Double yellow = 0.41;
    private static final Double green = 0.46;
    private static final Double purple = 0.67;

    public Command Off() {
        return new SetPosition(led, off);
    }

    public Command Red() {
        return new SetPosition(led, red);
    }

    public Command Yellow() {
        return new SetPosition(led, yellow);
    }

    public Command Green() {
        return new SetPosition(led, green);
    }

    public Command Purple() {
        return new SetPosition(led, purple);
    }


    @Override
    public void initialize() {
        led = new ServoEx(LEDname);
        led.getServo().scaleRange(0.0, 0.8);

    }

    @Override
    public void periodic() {

    }


}

