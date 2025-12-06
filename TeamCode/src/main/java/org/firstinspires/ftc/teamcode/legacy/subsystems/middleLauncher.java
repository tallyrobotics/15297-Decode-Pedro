package org.firstinspires.ftc.teamcode.legacy.subsystems;

public class middleLauncher extends launcher {

    public static final frontLauncher INSTANCE = new frontLauncher("middle",true);

    public middleLauncher(String launchName, Boolean isReversed) {
        super(launchName, isReversed);
    }


}
