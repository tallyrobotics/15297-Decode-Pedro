package org.firstinspires.ftc.teamcode.legacy.subsystems;

public class backLED extends LED {

    public static final backLED INSTANCE = new backLED("ledBack", "colorBack", "colorBack", 5.7);


    public backLED(String ledName, String colorName, String distanceName, Double maxDist) {
            super(ledName, colorName,distanceName, maxDist);
    }

}
