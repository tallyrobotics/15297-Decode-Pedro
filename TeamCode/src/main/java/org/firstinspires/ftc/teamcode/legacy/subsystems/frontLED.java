package org.firstinspires.ftc.teamcode.legacy.subsystems;

public class frontLED extends LED {

    public static final frontLED INSTANCE = new frontLED("ledFront", "colorFront", "colorFront", 7.0);

    public frontLED(String ledName, String colorName, String distanceName, Double maxDist) {
        super(ledName, colorName,distanceName, maxDist);
    }


}
