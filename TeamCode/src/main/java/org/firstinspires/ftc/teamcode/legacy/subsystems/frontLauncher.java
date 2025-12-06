package org.firstinspires.ftc.teamcode.legacy.subsystems;

public class frontLauncher extends launcher {

    public static final frontLauncher INSTANCE = new frontLauncher("front",false);

    public frontLauncher(String launchName, Boolean isReversed) {
        super(launchName, isReversed);
    }


}
