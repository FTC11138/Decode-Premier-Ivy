package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {

    /* -------------------------------------------- DRIVE CONSTANTS -------------------------------------------- */
    public static double pathEndXTolerance = 1;
    public static double pathEndYTolerance = 1;
    public static double pathEndHeadingTolerance = Math.toRadians(2);
    public static boolean robotCentric = false;
    public static double odoY = 4.091074485478439;
    public static double odoX = -6.032020989365465;
    public static double fieldWidth = 141.5;

    public static double driveForwardMultiplier = 1.4;
    public static double driveStrafeMultiplier = 1.4;
    public static double driveTurnMultiplier = 1.0;
    public static double driveTurnDeadband = 0.1;
    public static double driveFieldCentricYMultiplier = 1.1;

    public static double driveHeadingKp = 1.75;
    public static double driveHeadingKi = 0;
    public static double driveHeadingKd = 0.09;
    public static double driveHeadingKf = 0;
    public static double gateOpenHeadingDegrees = 36.5;
    public static double teleOpStartX = 72;
    public static double teleOpStartY = 72;
    public static double teleOpStartHeadingDegrees = 0;
    public static double redGoalX = 138;
    public static double redGoalY = 138;
    public static double blueGoalX = 3.5;
    public static double blueGoalY = 138;

    /* -------------------------------------------- CAMERA CONSTANTS -------------------------------------------- */
    //Pipeline: 0
    //Res: 1280X960 40FPS
    //Exposure: 252
    // Black Level Offset: 0
    // Sensor Gain: 15
    // Marker Size 101.6
    // Detector Downscale: 4
    // Quality Threshold: 2
    // Sort Mode: Largest
    public static double cameraHeightMeters = 0.39267;
    public static double cameraYawDegrees = 0;

    /* -------------------------------------------- INTAKE CONSTANTS -------------------------------------------- */
    public static double intakeFastPower = -1;
    public static double intakeSlowPower = -1;
    public static double intakeSlowPowerClose = -0.6;
    public static double intakeSlowPowerYThreshold = 48;
    public static double intakeOffPower = 0;
    public static double intakeReversePower = 1;
    public static double intakeShortReverseTimeMs = 150;

    public static double ballDetectThreshold = 0.3;
    public static int ballDetectWait = 170;
    public static int ballDetectWaitAuto = 160;
    public static int intakeUnstuckDelay = 100;
    public static int unstuckWait = 300;

    /* -------------------------------------------- SHOOT CONSTANTS -------------------------------------------- */
    public static double shooterKp = 0.01;
    public static double shooterKi = 0;
    public static double shooterKd = 0;
    public static double shooterKs = 0.065;
    public static double shooterKv = 0.000365;
    // Voltage compensation: scales flywheel output by (nominal / measured) so the
    // motor receives constant voltage as the battery sags. When comp is off or the
    // battery is at nominal, the scale is 1.0 and behavior is unchanged.
    public static boolean shooterVoltageComp = true;     // master switch
    public static double shooterNominalVoltage = 12.0;   // voltage kV was tuned at
    // Integral-zone: only accumulate the flywheel integral when |error| is within
    // this window, preventing windup during spin-up. shooterKi above is the
    // integral's own switch (0 = disabled).
    public static double shooterIntegralZoneTps = 150;
    public static int shooterVelocityTolerance = 25;
    public static boolean shooterOverride = false;
    public static double shooterOverrideTarget = 1000;
    public static double shooterPowerSign = -1.0;
    public static boolean shooterTopReversed = false;
    public static boolean shooterBottomReversed = false;
    public static boolean shooterTopEnabled = true;
    public static boolean shooterBottomEnabled = true;

    public static double frontShotVelocity = 1000;
    public static double farShotVelocity = 1600;

    public static double shootPower = -0.66;
    public static double readyPower = -1.0;
    public static double reverseStopPower = 1;
    public static double lowerShootPower = 0.71;
    public static double shootMultiplier = 1.0;
    public static double kF = 32767 / 2800.0;
    public static int shootBetweenWait = 300;
    public static int shootWait = 1800;
    public static double adjHoodMax = 0.02;
    public static double adjHoodMin = 0.48;
    public static double shootVelFar = -1850;
//    public static double shootVelClose = -1450;
//    public static double shootVelTol = 25;
    public static double shootHoodFar = 0.38;
//    public static double shootHoodClose = 0.38;
//    public static double autoHood = 0.4;
//    public static double autoVel = -1520;

    public static double shooterHoodTolerance = 0.015;

    public static double spindexerTicksPerRevolution = 537.7;
    public static double spindexerDeadbandDegrees = 2.0;
    public static double spindexerMovePower = 1.0;
    public static double spindexerShootPower = 0.7;
    public static double spindexerHoldPower = 0.25;
    public static boolean autoSpindex = true;
    public static long sensorWait = 325;
    public static long spindexerAutoLoadDelayMs = 100;
    public static long shootSensorWait = 500;
    public static long shootSingleSensorWait = 350;
    public static double stuckCurrent = 8.0;
    public static long spindexerUnstuckWaitMs = 750;
//    public static double spindexer_kP = 0.005;
//    public static double spindexer_kI = 0.000001;
//    public static double spindexer_kD = 0.0001;
//    public static double spindexerRotatePower = 0.5;

    /* -------------------------------------------- TURRET CONSTANTS -------------------------------------------- */
    public static double turretEncoderTicksPerRevolution = 8192.0;
    public static double turretEncoderRevolutionsPerTurretRevolution = 102.0 / 45.0;
    public static double turretTicksPerRevolution = turretEncoderTicksPerRevolution * turretEncoderRevolutionsPerTurretRevolution;
    public static double turretIncrementDegrees = 2.5;
    public static double turretHomedAngleDegrees = 0;
    public static double turretMinAngleDegrees = -90;
    public static double turretMaxAngleDegrees = 90;
//    public static double turretMinServoPosition = 0.0;
//    public static double turretMaxServoPosition = 1.0;
//    public static double turretDisabledServoPosition = 0.0;
    public static double turretForwardOffsetInches = 0;
    public static double turretLeftOffsetInches = -2.19;
//    public static double turretMinimumAutoAimAngleDegrees = -90;
    public static boolean turretEncoderReversed = true;
    public static boolean turretServoReversed = true;

    public static double deadbandDeg = 1.0;
    public static double errAlpha = 0.35;
//    public static double CENTER_KP = 0.008;
//    public static double CENTER_KD = 0.0001;
    public static double maxIntegral = 30.0;
    public static double maxDeriv = 320.0;
    public static double maxPower = 0.6;
    public static double kS = 0.0;
    public static double kP_v = 0.017;
    public static double kI_v = 0;
    public static double kD_v = 0.001;
//    public static double kP_velo = 0.75;
//    public static double kI_velo = 0.0;
//    public static double kD_velo = 0.2;
//    public static double kF_velo = 14.0;

}
