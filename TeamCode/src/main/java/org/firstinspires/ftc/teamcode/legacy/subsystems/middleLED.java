package org.firstinspires.ftc.teamcode.legacy.subsystems;

public class middleLED extends LED {

    public static final middleLED INSTANCE = new middleLED("ledMiddle", "colorMid", "colorMid", 4.4);

    public middleLED(String ledName, String colorName, String distanceName, Double maxDist) {
        super(ledName, colorName,distanceName, maxDist);
    }
}
