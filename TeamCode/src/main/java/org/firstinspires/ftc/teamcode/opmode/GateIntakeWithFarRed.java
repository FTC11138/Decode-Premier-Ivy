package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;

@Autonomous(name = "GateIntakeWithFarRed", group = "Competition")
public class GateIntakeWithFarRed extends GateIntakeWithFarAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
