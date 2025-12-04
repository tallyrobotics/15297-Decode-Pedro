package org.firstinspires.ftc.teamcode.legacy.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class frontLED extends LED {

    public static final frontLED INSTANCE = new frontLED("ledFront");

    public frontLED(String ledName) {
        super(ledName, "","");
    }


}
