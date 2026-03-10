package org.firstinspires.ftc.teamcode.legacy.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class flyLeftShooter extends flyShooter {// implements Subsystem {
    public static final flyLeftShooter INSTANCE = new flyLeftShooter("flyLeft", true);

    public flyLeftShooter(String motorName, boolean reverseMotor) {
        super(motorName, reverseMotor, 0);
    }
}
