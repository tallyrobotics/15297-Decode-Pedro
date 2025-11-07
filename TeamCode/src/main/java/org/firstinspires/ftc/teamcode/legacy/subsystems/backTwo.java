package org.firstinspires.ftc.teamcode.legacy.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.positionable.SetPosition;

public class backTwo implements Subsystem {

public static final backTwo INSTANCE = new backTwo();
private backTwo() {}

    public static final Double down = 0.0;
    public static final Double up = 1.0;

    private boolean isUp = false;

    public ServoEx lift;
    public String name = "backTwo";

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

    public Command up() {
        return new SetPosition(lift, up);
    }

    public Command down() {
        return new SetPosition(lift, down);
    }

    public Command shootCycle() {
        return new SequentialGroup(
                up(),
                new Delay(0.5),
                down()
        );
    }

    @Override
    public void initialize() {
        lift = new ServoEx(name);
        lift.getServo().setDirection(Servo.Direction.FORWARD);
    }
}
