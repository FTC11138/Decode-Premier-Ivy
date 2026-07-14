package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;

@Autonomous(name = "FarZoneRed", group = "Competition")
public class FarZoneRed extends FarZoneAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
