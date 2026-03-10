package org.firstinspires.ftc.teamcode.legacy.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class flyRightShooter extends flyShooter {
    public static final flyRightShooter INSTANCE = new flyRightShooter("flyRight", true);

    public flyRightShooter(String motorName, boolean reverseMotor) {
        super(motorName, reverseMotor, 0);
    }
}
