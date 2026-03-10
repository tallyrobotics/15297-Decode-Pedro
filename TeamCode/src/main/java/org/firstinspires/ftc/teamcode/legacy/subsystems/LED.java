package org.firstinspires.ftc.teamcode.legacy.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import java.lang.Double;

public abstract class LED implements Subsystem {

    public LED(String ledName, String colorSenseName2, String colorSenseName3, String distSenseName2, String distSenseName3, Double maxDistance2, Double maxDistance3, Boolean isRpm) {
        LEDname = ledName;
        colSenName2 = colorSenseName2;
        distanceName2 = distSenseName2;
        colSenName3 = colorSenseName3;
        distanceName3 = distSenseName3;
        maxDist2 = maxDistance2;
        maxDist3 = maxDistance3;
        isRPM = isRpm;
    }

    public ServoEx led;
    public ColorSensor colSensor3;
    public ColorSensor colSensor2;
    public DistanceSensor colorDistanceSensor2;
    public DistanceSensor colorDistanceSensor3;
    public String colSenName2;
    public String distanceName2;
    public String colSenName3;
    public String distanceName3;
    public String LEDname;
    public Boolean isRPM;
    private static final Double off = 0.0;
    private static final Double red = 0.285;
    private static final Double yellow = 0.41;
    private static final Double green = 0.46;
    private static final Double blue = 0.60;
    private static final Double purple = 0.67;

    public final Double maxDist2;
    public final Double maxDist3;

    private String color = "off";
    public double distance2 = -1.0;
    public double distance3 = -1.0;
    private boolean hasBall = false;

    public String getColor()
    {
        return color;
    }

    public boolean DetectBall()
    {
        return hasBall;
    }

    public Command Off() {
            return new SetPosition(led, off);
    }

    public Command Red() {
        color = "red";
        return new SetPosition(led, red);
    }

    public Command Yellow() {
        color = "yellow";
        return new SetPosition(led, yellow);
    }

    public Command Blue() {
        color = "blue";
        return new SetPosition(led, blue);
    }

    public Command Green() {
        return new SetPosition(led, green);
    }

    public Command Purple() {
        return new SetPosition(led, purple);
    }

    @Override
    public void initialize()
    {
        led = new ServoEx(LEDname);
        if(!isRPM){
            colSensor2 = ActiveOpMode.hardwareMap().get(ColorSensor.class, colSenName2);
            colorDistanceSensor2 = ActiveOpMode.hardwareMap().get(DistanceSensor.class, distanceName2);
            colSensor3 = ActiveOpMode.hardwareMap().get(ColorSensor.class, colSenName3);
            colorDistanceSensor3 = ActiveOpMode.hardwareMap().get(DistanceSensor.class, distanceName3);
        }
    }

    @Override
    public void periodic()
    {
        if (!isRPM) {
            NormalizedRGBA myNormalizedColors;
            myNormalizedColors = ((NormalizedColorSensor) colSensor3).getNormalizedColors();
            distance2 = Double.parseDouble(JavaUtil.formatNumber(colorDistanceSensor2.getDistance(DistanceUnit.CM), 1));
            distance3 = Double.parseDouble(JavaUtil.formatNumber(colorDistanceSensor3.getDistance(DistanceUnit.CM), 1));

            if ((!Double.isNaN(distance2)&&distance2<maxDist2)||distance3<maxDist3) {
                hasBall = true;
                if (myNormalizedColors.green >= myNormalizedColors.red && myNormalizedColors.green >= myNormalizedColors.blue) {
                    color = "green";
                    Green().schedule();
                } else if (myNormalizedColors.blue >= myNormalizedColors.red && myNormalizedColors.blue >= myNormalizedColors.green) {
                    color = "purple";
                    Purple().schedule();
                } else {
                    color = "off";
                    Off().schedule();
                }
            } else {
                hasBall = false;
                color = "off";
                Off().schedule();
            }
            ActiveOpMode.telemetry().addData(colSenName2 + " distance (cm)", distance2);
            ActiveOpMode.telemetry().addData(colSenName3 + " distance (cm)", distance3);
            ActiveOpMode.telemetry().addData(colSenName3 + "color", color);
        }
    }
}