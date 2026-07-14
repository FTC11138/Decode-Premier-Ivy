package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Alliance;

@Autonomous(name = "FarZoneBlue", group = "Competition")
public class FarZoneBlue extends FarZoneAuto {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
