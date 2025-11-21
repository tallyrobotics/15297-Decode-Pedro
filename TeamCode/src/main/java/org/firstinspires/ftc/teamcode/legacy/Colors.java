package org.firstinspires.ftc.teamcode.legacy;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;





    @TeleOp(name = "Colors (Blocks to Java)", group = "Sensor")
    public class Colors extends LinearOpMode {

        private ColorSensor colorFront;
        private ColorSensor colorMid;
        private ColorSensor colorBack;
        private LED ledMidRed;
        private LED ledMidGreen;
        private LED ledBackRed;
        private LED ledBackGreen;
        private LED ledFrontRed;
        private LED ledFrontGreen;
        private DistanceSensor colorFront_DistanceSensor;
        private DistanceSensor colorMid_DistanceSensor;
        private DistanceSensor colorBack_DistanceSensor;

        /**
         * This OpMode shows how to use a color sensor in a generic way, regardless
         * of which particular make or model of color sensor is used. The OpMode
         * assumes that the color sensor is configured with a name of "sensor_color".
         *
         * There will be some variation in the values measured depending on the specific sensor you are using.
         *
         * If the color sensor supports adjusting the gain, you can increase the gain
         * (a multiplier to make the sensor report higher values) by holding down the
         * A button on the gamepad, and decrease the gain by holding down the B button
         * on the gamepad. The AndyMark Proximity & Color Sensor does not support this.
         *
         * If the color sensor has a light which is controllable from software, you can use the
         * X button on the gamepad to toggle the light on and off. The REV sensors don't support
         * this, but instead have a physical switch on them to turn the light on and off, beginning
         * with REV Color Sensor V2. The AndyMark Proximity & Color Sensor does not support this.
         *
         * If the color sensor also supports short-range distance measurements (usually
         * via an infrared proximity sensor), the reported distance will be written to
         * telemetry. As of September 2025, the only color sensors that support this
         * are the ones from REV Robotics and the AndyMark Proximity & Color Sensor.
         * These infrared proximity sensor measurements are only useful at very small
         * distances, and are sensitive to ambient light and surface reflectivity.
         * You should use a different sensor if you need precise distance measurements.
         */
        @Override
        public void runOpMode() {
            int gain;
            String x;

            colorFront = hardwareMap.get(ColorSensor.class, "colorFront");
            colorMid = hardwareMap.get(ColorSensor.class, "colorMid");
            colorBack = hardwareMap.get(ColorSensor.class, "colorBack");
            ledMidRed = hardwareMap.get(LED.class, "ledMidRed");
            ledMidGreen = hardwareMap.get(LED.class, "ledMidGreen");
            ledBackRed = hardwareMap.get(LED.class, "ledBackRed");
            ledBackGreen = hardwareMap.get(LED.class, "ledBackGreen");
            ledFrontRed = hardwareMap.get(LED.class, "ledFrontRed");
            ledFrontGreen = hardwareMap.get(LED.class, "ledFrontGreen");
            colorFront_DistanceSensor = hardwareMap.get(DistanceSensor.class, "colorFront");
            colorMid_DistanceSensor = hardwareMap.get(DistanceSensor.class, "colorMid");
            colorBack_DistanceSensor = hardwareMap.get(DistanceSensor.class, "colorBack");

            // Put initialization blocks here.
            frontLed("");
            midLed("");
            backLed("");
            // You can give the sensor a gain value, will be multiplied by the sensor's raw value
            // before the normalized color values are calculated. Color sensors (especially the REV
            // Color Sensor V3) can give very low values (depending on the lighting conditions),
            // which only use a small part of the 0-1 range that is available for the red,
            // green, and blue values. In brighter conditions, you should use a smaller
            // gain than in dark conditions. If your gain is too high, all of the
            // colors will report at or near 1, and you won't be able to determine what
            // color you are actually looking at. For this reason, it's better to err
            // on the side of a lower gain (but always greater than or equal to 1).
            gain = 10;
            waitForStart();
            if (opModeIsActive()) {
                // Put run blocks here.
                // Once per loop we read the color sensor data, calculate the HSV colors
                // (Hue, Saturation and Value), and report all these values via telemetry.
                while (opModeIsActive()) {
                    // Put loop blocks here.
                    telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.");
                    telemetry.addLine(" ");
                    telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value.");
                    telemetry.addLine(" ");
                    // Update the gain value if either of the A or B gamepad buttons is being held
                    // A gain of less than 1 will make the values smaller, which is not helpful.
                    if (gamepad1.a) {
                        // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                        gain = (int) (gain + 0.005);
                    } else if (gamepad1.b && gain > 1) {
                        gain = (int) (gain - 0.005);
                    }
                    telemetry.addData("Gain", gain);
                    // Tell the sensor our desired gain value (normally you would do this during initialization, not during the loop)
                    ((NormalizedColorSensor) colorFront).setGain(gain);
                    // Tell the sensor our desired gain value (normally you would do this during initialization, not during the loop)
                    ((NormalizedColorSensor) colorMid).setGain(gain);
                    // Tell the sensor our desired gain value (normally you would do this during initialization, not during the loop)
                    ((NormalizedColorSensor) colorBack).setGain(gain);
                    x = getColor("F");
                    x = getColor("M");
                    x = getColor("B");
                    telemetry.update();
                }
            }
        }

        /**
         * Describe this function...
         */
        private void midLed(String color) {
            if (color.equals("R")) {
                ledMidRed.on();
                ledMidGreen.off();
            } else if (color.equals("G")) {
                ledMidGreen.on();
                ledMidRed.off();
            } else {
                ledMidRed.off();
                ledMidGreen.off();
            }
        }

        /**
         * Describe this function...
         */
        private void backLed(String color) {
            if (color.equals("R")) {
                ledBackRed.on();
                ledBackGreen.off();
            } else if (color.equals("G")) {
                ledBackGreen.on();
                ledBackRed.off();
            } else {
                ledBackRed.off();
                ledBackGreen.off();
            }
        }

        /**
         * Describe this function...
         */
        private void frontLed(String color) {
            if (color.equals("R")) {
                ledFrontRed.on();
                ledFrontGreen.off();
            } else if (color.equals("G")) {
                ledFrontGreen.on();
                ledFrontRed.off();
            } else {
                ledFrontRed.off();
                ledFrontGreen.off();
            }
        }

        /**
         * Describe this function...
         */
        private String getColor(String location) {
            String gotColor;
            NormalizedRGBA myNormalizedColors;
            int myColor;
            float hue;
            float saturation;
            float value;
            double gotDistance;

            if (location.equals("F")) {
                // Save the color sensor data as a normalized color value. It's recommended
                // to use Normalized Colors over color sensor colors is because Normalized
                // Colors consistently gives values between 0 and 1, while the direct
                // Color Sensor colors are dependent on the specific sensor you're using.
                myNormalizedColors = ((NormalizedColorSensor) colorFront).getNormalizedColors();
                // Convert the normalized color values to an Android color value.
                myColor = myNormalizedColors.toColor();
                // Use the Android color value to calculate the Hue, Saturation and Value color variables.
                // See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html for an explanation of HSV color.
                hue = JavaUtil.colorToHue(myColor);
                saturation = JavaUtil.colorToSaturation(myColor);
                value = JavaUtil.colorToValue(myColor);
                // Use telemetry to display feedback on the driver station. We show the red,
                // green, and blue normalized values from the sensor (in the range of 0 to
                // 1), as well as the equivalent HSV (hue, saturation and value) values.
                telemetry.addLine("red" + location + " | " + JavaUtil.formatNumber(myNormalizedColors.red, 3) + "green" + location + " | " + JavaUtil.formatNumber(myNormalizedColors.green, 3) + "blue" + location + JavaUtil.formatNumber(myNormalizedColors.blue, 3));
                telemetry.addLine("hue" + location + " | " + JavaUtil.formatNumber(hue, 3) + "saturation" + location + " | " + JavaUtil.formatNumber(saturation, 3) + "value" + location + JavaUtil.formatNumber(value, 3));
                telemetry.addData("alpha" + location, Double.parseDouble(JavaUtil.formatNumber(myNormalizedColors.alpha, 3)));
                gotDistance = Double.parseDouble(JavaUtil.formatNumber(colorFront_DistanceSensor.getDistance(DistanceUnit.CM), 3));
                // If this color sensor also has a distance sensor, display the measured distance.
                // Note that the reported distance is only useful at very close
                // range, and is impacted by ambient light and surface reflectivity.
                telemetry.addData("distance (cm)" + location, gotDistance);
                if (gotDistance < 5) {
                    if (myNormalizedColors.green >= myNormalizedColors.red && myNormalizedColors.green >= myNormalizedColors.blue) {
                        frontLed("G");
                        gotColor = "G";
                    } else if (myNormalizedColors.blue >= myNormalizedColors.red && myNormalizedColors.blue >= myNormalizedColors.blue) {
                        frontLed("R");
                        gotColor = "R";
                    } else {
                        frontLed("");
                        gotColor = "";
                    }
                } else {
                    frontLed("");
                    gotColor = "";
                }
            } else if (location.equals("M")) {
                // Save the color sensor data as a normalized color value. It's recommended
                // to use Normalized Colors over color sensor colors is because Normalized
                // Colors consistently gives values between 0 and 1, while the direct
                // Color Sensor colors are dependent on the specific sensor you're using.
                myNormalizedColors = ((NormalizedColorSensor) colorMid).getNormalizedColors();
                // Convert the normalized color values to an Android color value.
                myColor = myNormalizedColors.toColor();
                // Use the Android color value to calculate the Hue, Saturation and Value color variables.
                // See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html for an explanation of HSV color.
                hue = JavaUtil.colorToHue(myColor);
                saturation = JavaUtil.colorToSaturation(myColor);
                value = JavaUtil.colorToValue(myColor);
                // Use telemetry to display feedback on the driver station. We show the red,
                // green, and blue normalized values from the sensor (in the range of 0 to
                // 1), as well as the equivalent HSV (hue, saturation and value) values.
                telemetry.addLine("red" + location + " | " + JavaUtil.formatNumber(myNormalizedColors.red, 3) + "green" + location + " | " + JavaUtil.formatNumber(myNormalizedColors.green, 3) + "blue" + location + JavaUtil.formatNumber(myNormalizedColors.blue, 3));
                telemetry.addLine("hue" + location + " | " + JavaUtil.formatNumber(hue, 3) + "saturation" + location + " | " + JavaUtil.formatNumber(saturation, 3) + "value" + location + JavaUtil.formatNumber(value, 3));
                telemetry.addData("alpha" + location, Double.parseDouble(JavaUtil.formatNumber(myNormalizedColors.alpha, 3)));
                gotDistance = Double.parseDouble(JavaUtil.formatNumber(colorMid_DistanceSensor.getDistance(DistanceUnit.CM), 3));
                // If this color sensor also has a distance sensor, display the measured distance.
                // Note that the reported distance is only useful at very close
                // range, and is impacted by ambient light and surface reflectivity.
                telemetry.addData("distance (cm)" + location, gotDistance);
                if (gotDistance < 5) {
                    if (myNormalizedColors.green >= myNormalizedColors.red && myNormalizedColors.green >= myNormalizedColors.blue) {
                        midLed("G");
                        gotColor = "G";
                    } else if (myNormalizedColors.blue >= myNormalizedColors.red && myNormalizedColors.blue >= myNormalizedColors.blue) {
                        midLed("R");
                        gotColor = "R";
                    } else {
                        midLed("");
                        gotColor = "";
                    }
                } else {
                    midLed("");
                    gotColor = "";
                }
            } else if (location.equals("B")) {
                // Save the color sensor data as a normalized color value. It's recommended
                // to use Normalized Colors over color sensor colors is because Normalized
                // Colors consistently gives values between 0 and 1, while the direct
                // Color Sensor colors are dependent on the specific sensor you're using.
                myNormalizedColors = ((NormalizedColorSensor) colorBack).getNormalizedColors();
                // Convert the normalized color values to an Android color value.
                myColor = myNormalizedColors.toColor();
                // Use the Android color value to calculate the Hue, Saturation and Value color variables.
                // See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html for an explanation of HSV color.
                hue = JavaUtil.colorToHue(myColor);
                saturation = JavaUtil.colorToSaturation(myColor);
                value = JavaUtil.colorToValue(myColor);
                // Use telemetry to display feedback on the driver station. We show the red,
                // green, and blue normalized values from the sensor (in the range of 0 to
                // 1), as well as the equivalent HSV (hue, saturation and value) values.
                telemetry.addLine("red" + location + " | " + JavaUtil.formatNumber(myNormalizedColors.red, 3) + "green" + location + " | " + JavaUtil.formatNumber(myNormalizedColors.green, 3) + "blue" + location + JavaUtil.formatNumber(myNormalizedColors.blue, 3));
                telemetry.addLine("hue" + location + " | " + JavaUtil.formatNumber(hue, 3) + "saturation" + location + " | " + JavaUtil.formatNumber(saturation, 3) + "value" + location + JavaUtil.formatNumber(value, 3));
                telemetry.addData("alpha" + location, Double.parseDouble(JavaUtil.formatNumber(myNormalizedColors.alpha, 3)));
                gotDistance = Double.parseDouble(JavaUtil.formatNumber(colorBack_DistanceSensor.getDistance(DistanceUnit.CM), 3));
                // If this color sensor also has a distance sensor, display the measured distance.
                // Note that the reported distance is only useful at very close
                // range, and is impacted by ambient light and surface reflectivity.
                telemetry.addData("distance (cm)" + location, gotDistance);
                if (gotDistance < 5) {
                    if (myNormalizedColors.green >= myNormalizedColors.red && myNormalizedColors.green >= myNormalizedColors.blue) {
                        backLed("G");
                        gotColor = "G";
                    } else if (myNormalizedColors.blue >= myNormalizedColors.red && myNormalizedColors.blue >= myNormalizedColors.blue) {
                        backLed("R");
                        gotColor = "R";
                    } else {
                        backLed("");
                        gotColor = "";
                    }
                } else {
                    backLed("");
                    gotColor = "";
                }
            } else {
                gotColor = "";
            }
            return gotColor;
        }
    }

