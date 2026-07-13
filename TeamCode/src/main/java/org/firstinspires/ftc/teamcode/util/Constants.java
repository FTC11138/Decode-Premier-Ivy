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
    // Full field span (inches). Single source of truth for mirroring BLUE-authored
    // poses to RED (auto paths + teleop relocalization) and for PoseMirror's aim
    // mirror, so those never silently diverge. final = a fixed physical dimension,
    // not a dashboard-tunable value.
    public static final double fieldWidthInches = 141.5;

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
    public static double limelightMaxResetDistanceInches = 18.0;
    public static double limelightMaxResetHeadingDegrees = 25.0;
    public static int limelightMinimumTagCount = 1;
    public static boolean limelightAllowLargePoseReset = false;
    // --- Multi-frame snapshot reset (MT2 position-only, auto-triggered) ---------
    // When true, the camera correction is a discrete, multi-frame snapshot: it
    // collects several agreeing, quality-gated frames and then snaps X/Y to their
    // median (keeping the IMU heading, since MegaTag2 can't measure heading). When
    // false, LLPoseResetter falls back to the legacy continuous soft-fusion blend.
    // This fits a rear camera that only sees a tag a few times per match.
    public static boolean limelightUseSnapshotReset = true;
    // How many distinct, gated camera frames must agree before a reset fires.
    public static int limelightResetSampleCount = 5;
    // All samples must fall within this rolling window (ms) or the oldest is dropped.
    // Keep >= sampleCount / cameraFps; at 15 Hz, 5 frames span ~270 ms.
    public static double limelightSampleWindowMs = 500;
    // Reject any frame older than this (ms). Guards against reusing a stale packet.
    public static double limelightMaxStalenessMs = 100;
    // Camera-to-tag distance gate (inches). Too close = steep angle / clipped tag;
    // too far = PnP noise grows. Skipped automatically if per-tag 3D pose is absent.
    public static double limelightMinTagDistanceInches = 12.0;
    public static double limelightMaxTagDistanceInches = 120.0;
    // Reject frames while the robot is rotating faster than this (deg/s); a rear
    // camera smears badly while spinning.
    public static double limelightMaxAngularVelocityDps = 60.0;
    // Buffered samples must agree within this X/Y spread (inches) — the key defense
    // against a single bad frame or a PnP "flipped" solution slipping through.
    public static double limelightMaxSampleSpreadInches = 2.0;
    // Wait this long (ms) after a successful reset before another can fire.
    public static double limelightResetCooldownMs = 750;
    // Goal-tag allow-list (trust filter). DECODE localizes from the GOAL tags only;
    // the OBELISK motif tags (21/22/23) are randomized and sit off the field, so a
    // reset must be sourced from a goal tag. When true, a frame is rejected unless it
    // contains one of the goal IDs below. Backup for a correct goal-only field map —
    // the .fmap config is the true guarantee; this is defense-in-depth. Verify the
    // IDs against your loaded field map / game manual before trusting them.
    public static boolean limelightRequireGoalTag = true;
    public static int limelightBlueGoalTagId = 20;
    public static int limelightRedGoalTagId = 24;
    // Poll rate (Hz) the SDK pulls results at; higher = fresher frames so the
    // staleness gate passes more often and consensus builds faster in the brief
    // window the rear camera faces a tag.
    public static int limelightPollRateHz = 100;
    // Opt-in pipeline lock. When true, start() switches the Limelight to
    // limelightPipelineIndex so it always runs the AprilTag/MegaTag pipeline
    // (guards against booting on the wrong pipeline). Off by default = no change;
    // set the index to YOUR AprilTag pipeline slot before enabling.
    public static boolean limelightForcePipeline = false;
    public static int limelightPipelineIndex = 0;
    // Absolute field-bounds sanity for a vision reset (inches). A valid field pose
    // must land within these bounds (a small margin outside 0..144 allows for the
    // robot origin sitting near a wall). Catches an off-field "wrong solution" that
    // frame-agreement alone cannot.
    public static double limelightFieldMinInches = -6.0;
    public static double limelightFieldMaxInches = 150.0;
    // Max distance (inches) a snapshot reset may move the pose from odometry. More
    // generous than limelightMaxResetDistanceInches because the multi-frame
    // agreement + goal-tag + field-bounds gates already validate the fix, so a large
    // but trusted correction (exactly what you turn around to get after drift) is
    // allowed through. Set limelightAllowLargePoseReset=true to bypass entirely.
    public static double limelightSnapshotMaxJumpInches = 60.0;

    /* -------------------------------------------- INTAKE CONSTANTS -------------------------------------------- */
    public static double intakeFastPower = -1;
    public static double intakeSlowPower = -1;
    public static double intakeSlowPowerClose = -0.6;
    public static double intakeSlowPowerYThreshold = 48;
    public static double intakeOffPower = 0;
    public static double intakeReversePower = 1;
    public static double intakeShortReverseTimeMs = 30;
    // Reverse power for jam recovery. Strong enough to actually kick a wedged ball
    // back out quickly (the old 0.4 was too gentle to clear jams in a reasonable
    // time). If it starts flinging GOOD (loaded) balls back out of the robot, dial
    // this down toward 0.5. The full-strength reverse is still used for manual/eject.
    public static double intakeJamReversePower = 0.6;
    // How long the direct jam reverse (Spindexer periodic -> Intake.requestJamReverse)
    // runs per event. This is the recovery that clears a stuck ball during AUTO,
    // where a scheduled reverse would be blocked. Longer = clears heavier jams but
    // risks backing a good ball out; paired with spindexerUnstuckWaitMs cooldown.
    // THIS IS THE FIRST KNOB TO RAISE if the robot still wedges (e.g. at the gate).
    public static long intakeJamReverseDurationMs = 200;
    // After the spindexer finishes a counterclockwise turn, the feed servo keeps
    // running with it this much longer (unless the intake is running, the
    // spindexer turns clockwise, or it is shooting - those supersede this coast).
    public static long intakeServoCcwCoastMs = 300;
    // When the spindexer fills to 3: wait, then reverse to spit the extra ball,
    // then the intake shuts off until toggled or the count drops below 3.
    public static long intakeFullEjectDelayMs = 400;
    public static long intakeFullEjectReverseMs = 500;
    public static double intakeStuckCurrentMilliamps = 4000;
    // Hysteresis release: once the stuck timer starts, it keeps counting until
    // current drops below this, so noise near the trigger doesn't reset it.
    public static double intakeStuckReleaseMilliamps = 3000;
    // How long high current must sustain (with NO ball being handled) before a
    // normal jam fires. Lowered from 250 for a snappier reaction. This path only
    // runs when the spindexer is idle and no ball is on the sensor, so a normal
    // load can't trip it - raise it only if idle current noise causes false reverses.
    public static long intakeStuckDetectionTimeMs = 160;
    // A "hard jam" - current stays high this long even while the auto-index path
    // looks busy (ball at sensor / spindexer stuck mid-move) - forces a reverse
    // and index anyway, since the normal path clearly isn't clearing the ball.
    // This is the dominant reaction time when a ball is wedged ON the sensor (the
    // common case), so it was cut from 900 to react roughly twice as fast. If a
    // normal load ever pins current long enough to trip it (spurious reverse +
    // ball miscount mid-load), raise it back toward 700.
    public static long intakeHardJamTimeMs = 500;

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
    // integral's own switch (0 = disabled), so this is currently inert until kI>0.
    public static double shooterIntegralZoneTps = 150;
    // TEMPORARY: widened from 20 so atTarget() (and the ready LED) trips easily and
    // the auto reliably fires. Tighten back down once shooting is confirmed working.
    public static int shooterVelocityTolerance = 100;
    // TeleOp firing discipline (OFF by default = no behavior change). When enabled,
    // the right-TRIGGER shot is blocked while the robot is driving faster than
    // shooterMaxFireVelocityIps, so a moving robot does not throw shots off-aim.
    // The right-BUMPER stays ungated as a manual override. Enable + tune on hardware.
    public static boolean shooterRequireSlowToFire = false;
    public static double shooterMaxFireVelocityIps = 12.0;
    public static boolean shooterOverride = false;
    public static double shooterOverrideTarget = 1000;
    public static double shooterPowerSign = -1.0;
    public static boolean shooterTopReversed = false;
    public static boolean shooterBottomReversed = true;
    public static boolean shooterTopEnabled = true;
    public static boolean shooterBottomEnabled = true;

    public static double shooterTestSpeed = 600;
    public static double shooterTestHood = 0.81;

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
    public static double adjHoodMax = 0.0;
    public static double adjHoodMin = 0.48;
    public static double adjHoodServoMax = 0.81;
    public static double shootVelFar = -500;
//    public static double shootVelClose = -1450;
//    public static double shootVelTol = 25;
    public static double shootHoodFar = 0.55;
//    public static double shootHoodClose = 0.38;
//    public static double autoHood = 0.4;
//    public static double autoVel = -1520;

    public static double shooterHoodTolerance = 0.015;
    // Estimated hood-servo travel speed in servo-position units per second. The
    // hood servo has no feedback (getPosition() only echoes the last command), so
    // Shooter models the hood's physical position by integrating the commanded
    // target at this rate. atTarget() waits for that model to arrive, so a shot
    // never fires while the hood is still traveling to a new angle (e.g. right
    // after a close<->far zone switch). Tune DOWN if shots still leave before the
    // hood settles; UP if it needlessly delays ready shots.
    public static double shooterHoodSlewRatePerSecond = 1.0;
    // Master switch for the hood-settle model. When false, atTarget() falls back to
    // master behavior (compares the servo's echoed position to the target, i.e. no
    // settle gating), so the feature can be disabled instantly from the dashboard.
    public static boolean shooterHoodModelEnabled = true;
    // Hard cap (ms) on how long the hood-settle model may hold atTarget() false after
    // the hood goal changes. Bounds the worst-case delay a wrong slew-rate estimate
    // could add, protecting the un-timed TeleOp trigger (Auto also has its own 2s
    // race). Keep comfortably above the real hood travel time so it rarely triggers.
    public static double shooterHoodMaxSettleMs = 400;

    public static double spindexerTicksPerRevolution = 8192.0;
    public static double spindexerDeadbandDegrees = 2.0;
    public static double spindexerHoldCorrectionDegrees = 2.0;
    public static double spindexerPositionKp = 0.00026;
    public static double spindexerPositionKi = 0.0000005;
    public static double spindexerPositionKd = 0.000013;
    // Static feedforward: constant push (in the direction of travel) added on top
    // of the PID while moving, to break past the mechanical detent/resistance on
    // every turn. Applied only while |error| > deadband, so it never fights the
    // hold at rest. Tune up if it still stalls short, down if it overshoots/jitters.
    public static double spindexerFeedforward = 0.071;
    public static double spindexerMaxIntegral = 1500.0;
    public static long spindexerMoveTimeoutMs = 3000;
    public static boolean spindexerEncoderReversed = false;
    public static boolean spindexerMotorReversed = false;
    public static boolean autoSpindex = true;
    public static long sensorWait = 200;
    public static long spindexerAutoLoadDelayMs = 0;
    // Minimum time between spindexer rotations: a manual index is rejected if a
    // rotation happened within this window, and the auto-index also waits this
    // long after any rotation before spinning again.
    public static long spindexerRotationLockoutMs = 500;
    public static long postShootSensorWaitMs = 300;
    public static long shootSingleSensorWait = 350;
    // Cooldown after a jam recovery fires before another can. Lowered from 750 so a
    // stubborn jam gets re-pulsed sooner instead of grinding forward for ~600ms
    // between attempts. CAUTION: an intake-motor jam recovery also advances the
    // spindexer one slot and increments the ball count, so if a single physical jam
    // starts inflating the ball count (robot thinks it's full early), raise this
    // back up. Bounded anyway - the slot/count path stops once ballCount hits 3.
    public static long spindexerUnstuckWaitMs = 500;
    // Spindexer strained mid-CCW-turn: if the spindexer motor draws more than this
    // while turning counterclockwise, a ball is wedging it, so reverse the intake
    // to relieve it. The sustain time avoids firing on the normal acceleration
    // current spike (raise it if normal turns trip it, lower for faster reaction).
    public static double spindexerStuckCurrentMilliamps = 3000;
    public static long spindexerStuckDetectionTimeMs = 200;
    // Safety net: if a ball stays detected this long with nothing queued and it
    // is safe to act, force a load so a ball never sits unhandled.
    public static long spindexerBallRecoveryMs = 400;

    /* -------------------------------------------- LED CONSTANTS -------------------------------------------- */
    // RGB indicator lights driven as PWM servos. Positions are the FTC 0-1
    // values from the Base10 3118-0808-0002 color chart.
    public static double ledOff = 0.0;     // 500us  - off
    public static double ledRed = 0.28;    // ~1104us - true red (chart red is 1100us; higher drifts orange)
    public static double ledOrange = 0.333;// 1200us - orange
    public static double ledGreen = 0.500; // 1500us - green
    public static double ledBlue = 0.611;  // 1700us - blue
    public static double ledPurple = 0.722;// 1900us - violet/purple (3 balls + ready)
    // 2 balls + ready. White (>1900us) - the strip zone right next to violet, so
    // it reads as "almost purple" yet is unmistakably different. Tune toward
    // indigo (~0.666) if you'd rather a purple-family shade.
    public static double ledReadyTwoBalls = 0.9; // ~2220us - white
//    public static double spindexer_kP = 0.005;
//    public static double spindexer_kI = 0.000001;
//    public static double spindexer_kD = 0.0001;
//    public static double spindexerRotatePower = 0.5;

    /* -------------------------------------------- TURRET CONSTANTS -------------------------------------------- */
    public static double turretEncoderTicksPerRevolution = 8192.0;
    public static double turretEncoderRevolutionsPerTurretRevolution = 102.0 / 45.0;
    public static double turretTicksPerRevolution = turretEncoderTicksPerRevolution * turretEncoderRevolutionsPerTurretRevolution;
    public static double turretIncrementDegrees = 2.5;
    public static double turretAimOffsetDegrees = 0;
    public static double turretAimOffsetStepDegrees = 1.0;
    public static double turretJoystickOffsetRateDegreesPerSecond = 45.0;
    // Small deadband while adjusting so tiny stick moves still register.
    public static double turretJoystickDeadband = 0.02;
    // Expo curve on the stick: >1 makes small deflections nudge slowly and large
    // deflections turn fast (2 = squared response). 1.0 would be linear.
    public static double turretJoystickExponent = 1.5;
    // Large enough to shift auto-aim across the whole turret range (~265 deg span),
    // so the manual offset can point the turret anywhere it can physically reach.
    // The final target is still clipped to [turretMin, turretMax] in TractorBeam.
    public static double turretMaximumAimOffsetDegrees = 270.0;
    public static double turretPoseXCorrectionInches = 0.0;
    public static double turretHomedAngleDegrees = 0;
    public static double turretMinAngleDegrees = -175;
    public static double turretMaxAngleDegrees = 90;
//    public static double turretMinServoPosition = 0.0;
//    public static double turretMaxServoPosition = 1.0;
//    public static double turretDisabledServoPosition = 0.0;
    public static double turretForwardOffsetInches = -0.150234276094488;
    public static double turretLeftOffsetInches = -2.17392353822834661;
//    public static double turretMinimumAutoAimAngleDegrees = -90;
    public static boolean turretEncoderReversed = true;
    public static boolean turretServoReversed = true;

    public static double deadbandDeg = 1.0;
    // Firing gate: the turret counts as "aimed" once |target - angle| is within
    // this. MUST be > deadbandDeg (the controller stops correcting inside the
    // deadband, so a tighter tolerance could never trip and would block firing).
    // Tighten toward deadbandDeg for more accuracy once verified on hardware.
    public static double turretAimedToleranceDegrees = 2.0;
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
