package org.firstinspires.ftc.teamcode.legacy.subsystems;

import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.ftc.ActiveOpMode;

public class intakeLED extends SubsystemGroup {
    public static final intakeLED INSTANCE = new intakeLED();

    private intakeLED(){
        super(
                intake.INSTANCE,
                frontLED.INSTANCE,
                middleLED.INSTANCE,
                backLED.INSTANCE
        );
    }

    @Override
    public void periodic() {
        super.periodic();
        if((frontLED.INSTANCE.getDistance()<5.5&&middleLED.INSTANCE.getDistance()<4.5&&backLED.INSTANCE.getDistance()<5.7)||
        !ActiveOpMode.isStarted())
        {
            intake.INSTANCE.IntakeOff().schedule();
        }
        else{
            intake.INSTANCE.IntakeIn().schedule();
        }
    }

    public String frontColor(){
        return frontLED.INSTANCE.getColor();
    }
    public String middleColor(){
        return middleLED.INSTANCE.getColor();
    }
    public String backColor(){
        return backLED.INSTANCE.getColor();
    }
}
