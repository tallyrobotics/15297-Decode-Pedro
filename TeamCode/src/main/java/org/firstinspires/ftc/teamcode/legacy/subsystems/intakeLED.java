package org.firstinspires.ftc.teamcode.legacy.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;

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
                (frontLED.INSTANCE.getDistance2() != DistanceSensor.distanceOutOfRange ||frontLED.INSTANCE.getDistance3()<frontLED.INSTANCE.maxDist3)&&
                (middleLED.INSTANCE.getDistance2()!= DistanceSensor.distanceOutOfRange ||middleLED.INSTANCE.getDistance3()<middleLED.INSTANCE.maxDist3)&&
                (backLED.INSTANCE.getDistance2()!= DistanceSensor.distanceOutOfRange ||backLED.INSTANCE.getDistance3()<backLED.INSTANCE.maxDist3)))
                || !ActiveOpMode.isStarted())
        {
//            intake.INSTANCE.IntakeOff().schedule();
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
