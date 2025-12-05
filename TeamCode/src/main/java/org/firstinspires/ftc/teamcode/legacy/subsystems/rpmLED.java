package org.firstinspires.ftc.teamcode.legacy.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class rpmLED extends LED {
    public static final rpmLED INSTANCE = new rpmLED("ledRPM", "", "");

    public rpmLED(String ledName, String colorName, String distanceName) {
        super(ledName, colorName,distanceName);
    }
//
//    public rpmLED() {
//
//
//    }
//
//
//    public ServoEx led;
//    private static final Double off = 0.0;
//    private static final Double red = 0.285;
//    private static final Double yellow = 0.41;
//    private static final Double green = 0.56;
//    private static final Double purple = 0.77;
//
//    public static String color;
//
//    public Command Off() {
//        color = "off";
//        return new SetPosition(led, off);
//    }
//
//    public Command Red() {
//        color = "red";
//        return new SetPosition(led, red);
//    }
//
//    public Command Yellow() {
//        color = "yellow";
//        return new SetPosition(led, yellow);
//    }
//
//    public Command Green() {
//        color = "green";
//        return new SetPosition(led, green);
//    }
//
//    public Command Purple() {
//        color = "purple";
//        return new SetPosition(led, purple);
//    }
//
//
//    @Override
//    public void initialize() {
//        led = new ServoEx("ledRPM");
//
//
//    }
//
//    @Override
//    public void periodic() {
//
//    }
//
//
//
//
//
}
