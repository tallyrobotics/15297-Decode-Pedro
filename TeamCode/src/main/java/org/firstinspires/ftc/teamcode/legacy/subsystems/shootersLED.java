package org.firstinspires.ftc.teamcode.legacy.subsystems;

import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.ftc.ActiveOpMode;

public class shootersLED extends SubsystemGroup {
    public static final shootersLED INSTANCE = new shootersLED();

    private shootersLED() {
        super(
                flyLeftShooter.INSTANCE,
                flyRightShooter.INSTANCE,
                rpmLED.INSTANCE
        );
    }

    public void periodic() {
        super.periodic();
        double leftRPM = flyLeftShooter.INSTANCE.flyMotor.getVelocity();
        double rightRPM = flyRightShooter.INSTANCE.flyMotor.getVelocity();
        boolean leftGood = false;
        boolean rightGood = false;

        if(leftRPM>=flyLeftShooter.INSTANCE.getTargetRPM()-50&&
                leftRPM<=flyLeftShooter.INSTANCE.getTargetRPM()+50){
            leftGood = true;
        }
        if(rightRPM>=flyRightShooter.INSTANCE.getTargetRPM()-50&&
                rightRPM<=flyRightShooter.INSTANCE.getTargetRPM()+50){
            rightGood = true;
        }

        if(rightGood&&leftGood){
            rpmLED.INSTANCE.Blue().schedule();
        }
        else if(leftGood){
            rpmLED.INSTANCE.Yellow().schedule();
        }
        else if(rightGood){
            rpmLED.INSTANCE.Red().schedule();
        }
        else{
            rpmLED.INSTANCE.Off().schedule();
        }
    }
}
