package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;

@Autonomous(name = "AutoCloseNoGateFallBackBlue", group = "Competition")
public class AutoCloseNoGateFallBackBlue extends AutoCloseNoGateFallBack {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
