package org.firstinspires.ftc.teamcode.legacy.subsystems;

public class backLED extends LED {

    public static final backLED INSTANCE = new backLED("ledBack", "colorBack", "colorBack");


    public backLED(String ledName, String colorName, String distanceName) {
            super(ledName, colorName,distanceName);
    }

}
