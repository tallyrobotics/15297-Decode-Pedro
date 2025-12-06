package org.firstinspires.ftc.teamcode.legacy.subsystems;

public class frontLED extends LED {

    public static final frontLED INSTANCE = new frontLED("ledFront", "colorFront", "colorFront");

    public frontLED(String ledName, String colorName, String distanceName) {
        super(ledName, colorName,distanceName);
    }


}
