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
        if((frontLED.INSTANCE.getDistance()<5.0&&middleLED.INSTANCE.getDistance()<5.0&&backLED.INSTANCE.getDistance()<5.0)||
        !ActiveOpMode.isStarted())
        {
            intake.INSTANCE.IntakeOff().schedule();
        }
        else{
            intake.INSTANCE.IntakeIn().schedule();
        }
    }
}
