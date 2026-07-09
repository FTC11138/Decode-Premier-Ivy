package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;

@Autonomous(name = "GateIntakeWithFarBlue", group = "Competition")
public class GateIntakeWithFarBlue extends GateIntakeWithFarAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
