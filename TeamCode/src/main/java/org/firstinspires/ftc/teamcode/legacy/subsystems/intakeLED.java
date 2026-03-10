package org.firstinspires.ftc.teamcode.legacy.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
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

    boolean priority = false;
    public Command PriorityOn(){
        return new InstantCommand(()->{priority = true;});
    }

    public Command PriorityOff(){
        return new InstantCommand(()->{priority = false;});
    }

    @Override
    public void periodic() {
        super.periodic();
        if(((!priority&&
                ((frontLED.INSTANCE.DetectBall()))&&
                (middleLED.INSTANCE.DetectBall())&&
                (backLED.INSTANCE.DetectBall())))
                || !ActiveOpMode.isStarted())
        {
            intake.INSTANCE.off().schedule();
        }

        else{
            intake.INSTANCE.on().schedule();
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
