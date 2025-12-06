package org.firstinspires.ftc.teamcode.legacy.subsystems;

public class backLauncher extends launcher {

    public static final frontLauncher INSTANCE = new frontLauncher("back",false);

    public backLauncher(String launchName, Boolean isReversed) {
        super(launchName, isReversed);
    }


}
