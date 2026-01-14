package org.firstinspires.ftc.teamcode.legacy.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import com.bylazar.configurables.annotations.Configurable;
@Configurable
public abstract class launcher implements Subsystem {

public launcher(String launchName, Boolean isReversed) {name = launchName; reverse = isReversed;}



    public static Double down = 0.1;
    public static Double up = 0.95;



    private boolean isUp = false;

    public ServoEx lift;
    final boolean reverse;
    final String name;

    public Command toggle() {
        if (isUp) {
            isUp = false;
            return up();
        }
        else {
            isUp = true;
            return down();
        }
    }

    public Command up(){


            return new SetPosition(lift, up);


    }

    public Command down(){



        return new SetPosition(lift, down);
    }

    public Command shootCycle() {
            return new SequentialGroup(
                    up(),
                    new Delay(0.8),
                    down()
            );
    }

    @Override
    public void initialize() {
        lift = new ServoEx(name);
        if(reverse){
            lift.getServo().setDirection(Servo.Direction.REVERSE);
        }
        else{
            lift.getServo().setDirection(Servo.Direction.FORWARD);
        }
        lift.getServo().setPosition(0.02);
    }


    @Override
    public void periodic() {
    }
}
