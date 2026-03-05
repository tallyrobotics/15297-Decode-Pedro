package org.firstinspires.ftc.teamcode.legacy.subsystems;

public class frontLED extends LED {

    public static final frontLED INSTANCE = new frontLED("ledFront", "colorFront2", "colorFront3", "colorFront2", "colorFront3", 250.0, 4.1);

    public frontLED(String ledName, String colorName2, String colorName3, String distName2, String distName3, Double maxDist2, Double maxDist3) {
        super(ledName, colorName2,colorName3, distName2, distName3, maxDist2, maxDist3);
    }

}
