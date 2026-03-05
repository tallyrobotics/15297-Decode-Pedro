package org.firstinspires.ftc.teamcode.legacy.subsystems;

public class middleLED extends LED {

    public static final middleLED INSTANCE = new middleLED("ledMiddle", "colorMiddle2", "colorMiddle3", "colorMiddle2", "colorMiddle3", 250.0, 7.0);

    public middleLED(String ledName, String colorName2, String colorName3, String distName2, String distName3, Double maxDist2, Double maxDist3) {
        super(ledName, colorName2,colorName3, distName2, distName3, maxDist2, maxDist3);
    }
}
