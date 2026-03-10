package org.firstinspires.ftc.teamcode.legacy.subsystems;

public class rpmLED extends LED {
    public static final rpmLED INSTANCE = new rpmLED("ledRPM", "", "", "", "", 0.0, 0.0);

    public rpmLED(String ledName, String colorNameR, String colorNameL, String dNR, String dNL, Double dR, Double dL) {
        super(ledName, colorNameR, colorNameL, dNR, dNL, dR, dL, true);
    }
}