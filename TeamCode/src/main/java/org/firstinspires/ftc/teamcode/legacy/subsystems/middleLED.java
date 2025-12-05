package org.firstinspires.ftc.teamcode.legacy.subsystems;

public class middleLED extends LED {

    public static final middleLED INSTANCE = new middleLED("ledMiddle", "colorMid", "colorMid");

    public middleLED(String ledName, String colorName, String distanceName) {
        super(ledName, colorName,distanceName);
    }
}
