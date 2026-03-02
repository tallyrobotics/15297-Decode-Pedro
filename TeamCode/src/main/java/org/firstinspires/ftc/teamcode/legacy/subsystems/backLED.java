package org.firstinspires.ftc.teamcode.legacy.subsystems;

public class backLED extends LED {

    public static final backLED INSTANCE = new backLED("ledBack", "colorBack2", "colorBack3", "colorBack2", "colorBack3", 3.0, 3.0);


    public backLED(String ledName, String colorName2, String colorName3, String distName2, String distName3, Double maxDist2, Double maxDist3) {
            super(ledName, colorName2,colorName3, distName2, distName3, maxDist2, maxDist3);
    }

}
