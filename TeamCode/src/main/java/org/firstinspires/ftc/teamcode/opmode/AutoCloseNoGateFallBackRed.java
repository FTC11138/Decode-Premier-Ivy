package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;

@Autonomous(name = "AutoCloseNoGateFallBackRed", group = "Competition")
public class AutoCloseNoGateFallBackRed extends AutoCloseNoGateFallBack {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
