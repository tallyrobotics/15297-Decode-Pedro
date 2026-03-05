package org.firstinspires.ftc.teamcode.legacy.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import java.lang.Double;

public abstract class LED implements Subsystem {

    public LED(String ledName, String colorSenseName2, String colorSenseName3, String distSenseName2, String distSenseName3, Double maxDistance2, Double maxDistance3) {
        LEDname = ledName;
        colSenName2 = colorSenseName2;
        distanceName2 = distSenseName2;
        colSenName3 = colorSenseName3;
        distanceName3 = distSenseName3;
        maxDist2 = maxDistance2;
        maxDist3 = maxDistance3;
    }

    public ServoEx led;
    public ColorSensor colSensor3;
    public ColorSensor colSensor2;
    public DistanceSensor colorDistanceSensor2;
    public DistanceSensor colorDistanceSensor3;
    public String colSenName2;
    public String distanceName2;
    public String colSenName3;
    public String distanceName3;
    public String LEDname;
    private static final Double off = 0.0;
    private static final Double red = 0.285;
    private static final Double yellow = 0.41;
    private static final Double green = 0.46;
    private static final Double blue = 0.60;
    private static final Double purple = 0.67;

    public final Double maxDist2;
    public final Double maxDist3;

    private String color = "off";
    private String color2 = "off";
    private double distance2 = -1.0;
    private String color3 = "off";
    private double distance3 = -1.0;
    private String mainColor;

    public String getColor()
    {
        return mainColor;
    }

    public double getDistance2()
    {
        return distance2;
    }

    public double getDistance3(){
        return distance3;
    }


    public Command Off(String lor) {
            return new SetPosition(led, led.getPosition());
    }

    public Command Red() {
        color = "red";
        return new SetPosition(led, red);
    }

    public Command Yellow() {
        color = "yellow";
        return new SetPosition(led, yellow);
    }

    public Command Blue() {
        color = "blue";
        return new SetPosition(led, blue);
    }

    public Command Green(String lor) {
        mainColor = "green";
        return new SetPosition(led, green);
    }

    public Command Purple(String lor) {
        mainColor = "purple";
        return new SetPosition(led, purple);
    }

    @Override
    public void initialize()
    {
        led = new ServoEx(LEDname);
        if (!"".equals(colSenName2))
        {
            colSensor2 = ActiveOpMode.hardwareMap().get(ColorSensor.class, colSenName2);
        }
        if (!"".equals(distanceName2))
        {
            colorDistanceSensor2 = ActiveOpMode.hardwareMap().get(DistanceSensor.class, distanceName2);
        }
        if (!"".equals(colSenName3))
        {
            colSensor3 = ActiveOpMode.hardwareMap().get(ColorSensor.class, colSenName3);
        }
        if (!"".equals(distanceName3))
        {
            colorDistanceSensor3 = ActiveOpMode.hardwareMap().get(DistanceSensor.class, distanceName3);
        }

    }

    @Override
    public void periodic()
    {
        if (!"".equals(colSenName2)) {
            NormalizedRGBA myNormalizedColors;
            int myColor;
//        float hue;
//        float saturation;
//        float value;

            // Save the color sensor data as a normalized color value. It's recommended
            // to use Normalized Colors over color sensor colors is because Normalized
            // Colors consistently gives values between 0 and 1, while the direct
            // Color Sensor colors are dependent on the specific sensor you're using.
            myNormalizedColors = ((NormalizedColorSensor) colSensor2).getNormalizedColors();
            // Convert the normalized color values to an Android color value.
            myColor = myNormalizedColors.toColor();
            // Use the Android color value to calculate the Hue, Saturation and Value color variables.
            // See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html for an explanation of HSV color.
//        hue = JavaUtil.colorToHue(myColor);
//        saturation = JavaUtil.colorToSaturation(myColor);
//        value = JavaUtil.colorToValue(myColor);
            // Use telemetry to display feedback on the driver station. We show the red,
            // green, and blue normalized values from the sensor (in the range of 0 to
            // 1), as well as the equivalent HSV (hue, saturation and value) values.
            distance2 = Double.parseDouble(JavaUtil.formatNumber(colorDistanceSensor2.getDistance(DistanceUnit.CM), 1));
            // If this color sensor also has a distance sensor, display the measured distance.
            // Note that the reported distance is only useful at very close
            // range, and is impacted by ambient light and surface reflectivity.
            ActiveOpMode.telemetry().addData(colSenName2 + " distance (cm)", distance2);
            ActiveOpMode.telemetry().addData(colSenName2 + " color", color2);
            if (!Double.isNaN(distance2)) {
                if (myNormalizedColors.green >= myNormalizedColors.red && myNormalizedColors.green >= myNormalizedColors.blue) {
                    color2 = "green";
                    Green("R").schedule();
                } else if (myNormalizedColors.blue >= myNormalizedColors.red && myNormalizedColors.blue >= myNormalizedColors.green) {
                    color2 = "purple";
                    Purple("R").schedule();
                } else {
                    color2 = "off";
                }
            } else {
                color2 = "off";
            }
        }
        if(!"".equals(colSenName3)){
            NormalizedRGBA myNormalizedColors;
            int myColor;
//        float hue;
//        float saturation;
//        float value;

            // Save the color sensor data as a normalized color value. It's recommended
            // to use Normalized Colors over color sensor colors is because Normalized
            // Colors consistently gives values between 0 and 1, while the direct
            // Color Sensor colors are dependent on the specific sensor you're using.
            myNormalizedColors = ((NormalizedColorSensor) colSensor3).getNormalizedColors();
            // Convert the normalized color values to an Android color value.
            myColor = myNormalizedColors.toColor();
            // Use the Android color value to calculate the Hue, Saturation and Value color variables.
            // See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html for an explanation of HSV color.
//        hue = JavaUtil.colorToHue(myColor);
//        saturation = JavaUtil.colorToSaturation(myColor);
//        value = JavaUtil.colorToValue(myColor);
            // Use telemetry to display feedback on the driver station. We show the red,
            // green, and blue normalized values from the sensor (in the range of 0 to
            // 1), as well as the equivalent HSV (hue, saturation and value) values.
            distance3 = Double.parseDouble(JavaUtil.formatNumber(colorDistanceSensor3.getDistance(DistanceUnit.CM), 1));
            // If this color sensor also has a distance sensor, display the measured distance.
            // Note that the reported distance is only useful at very close
            // range, and is impacted by ambient light and surface reflectivity.
            ActiveOpMode.telemetry().addData(colSenName3 + " distance (cm)", distance3);
            ActiveOpMode.telemetry().addData(colSenName3 + "color", color3);
            if (distance3 < maxDist3) {
                if (myNormalizedColors.green >= myNormalizedColors.red && myNormalizedColors.green >= myNormalizedColors.blue) {
                    color3 = "green";
                    Green("L").schedule();
                } else if (myNormalizedColors.blue >= myNormalizedColors.red && myNormalizedColors.blue >= myNormalizedColors.green) {
                    color3 = "purple";
                    Purple("L").schedule();
                } else {
                }
            } else {
                color3 = "off";
            }

        }
        if(color2 == "off"&&color3=="off"){
            Off("E").schedule();
        }
    }

}

