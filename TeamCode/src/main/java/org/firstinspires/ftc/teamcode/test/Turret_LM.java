//NOTICE READ THIS FIRST
//for anyone going over this code, I let AI go through the code and add comments to what most lines do
//it should be able to answer some questions or simplify the complicated parts



package org.firstinspires.ftc.teamcode.test;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp
public class Turret_LM extends LinearOpMode {

    // =========================================================================
    // PIDF CONTROLLER (inner class)
    // =========================================================================
    // PIDF stands for Proportional, Integral, Derivative, and Feedforward.
    // It's a control algorithm that calculates how much power to send to a motor
    // based on how far away the motor is from its target position (the "error").
    //
    // Think of it like cruise control in a car:
    //   - P (Proportional): the further you are from the target, the harder you push
    //   - I (Integral):     if you've been off-target for a while, push a little extra
    //   - D (Derivative):   if you're approaching the target fast, ease off early
    //   - F (Feedforward):  a constant "hint" push in the right direction to help overcome friction
    //
    // We have TWO instances of this class below: one for large errors (far from target)
    // and one for small errors (close to target), with different tuning values for each.
    // =========================================================================
    private static class PIDF {
        private double kP, kI, kD, kF;          // The four gain coefficients (tuning knobs)
        private double lastError = 0, integral = 0; // Remembered state between loop cycles
        private double feedForward = 0;             // The direction hint passed in each loop

        public PIDF(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }

        // Called every loop to feed in the current error value.
        // The integral accumulates over time (sum of all past errors).
        public void updateError(double error) {
            integral += error;
            lastError = error;
        }

        // Called every loop to tell the PIDF which direction to push.
        // Typically +1.0 or -1.0 (or a scaled version) based on which way the turret needs to move.
        public void updateFeedForwardInput(double ff) {
            feedForward = ff;
        }

        // Calculates and returns the final motor power output.
        // Note: kP * lastError appears twice below (P and D share lastError here since
        // we're not storing the previous error separately — kD effectively acts as
        // a second proportional gain in this simplified implementation).
        public double run() {
            return kP * lastError + kI * integral + kD * lastError + kF * feedForward;
        }

        // Wipes all remembered state. Called when switching between tracking modes
        // (vision <-> odometry) so stale values from one mode don't bleed into the other
        // and cause unexpected jerks or power spikes.
        public void reset() {
            lastError = 0;
            integral = 0;
            feedForward = 0;
        }
    }

    // =========================================================================
    // TUNING CONSTANTS
    // =========================================================================
    // rpt = "radians per tick" — how many radians the turret physically rotates
    // for every one encoder tick on the turret motor. Used to convert between
    // encoder counts (what the motor reports) and angles (what the math works in).
    public static double rpt        = 0.00268785;

    // pidfSwitch = the error threshold (in encoder ticks) that decides which PIDF
    // set to use. If the turret is MORE than 60 ticks away from target, use the
    // aggressive "far" PIDF (p). If it's LESS than 60 ticks away, use the gentle
    // "near" PIDF (s) for a smooth, accurate final approach.
    public static double pidfSwitch = 60;

    // "Far" PIDF gains — used when the turret has a large error (needs to move quickly)
    public static double kp = 1.5, kf = 0.05, kd = 0.005;

    // "Near" PIDF gains — used when the turret is close to target (needs fine control)
    // Also used exclusively during vision tracking (Limelight).
    public static double sp = 0.5, sf = 0.03, sd = 0.001;

    // VISION_SIGN flips the direction of the entire vision tracking correction.
    // Confirmed via test footage: with +1.0, tx climbed away from target instead of
    // converging toward it, meaning positive PIDF output was turning the turret
    // the wrong way. Set to -1.0 to invert and correct this.
    public static double VISION_SIGN = -1.0;

    // Maximum motor power allowed during vision (fine) tracking.
    // Keeps the turret from slamming at full speed if the Limelight suddenly
    // re-acquires the tag after losing it, or if there's a big initial tx jump.
    public static double VISION_MAX_POWER = 0.5;

    // =========================================================================
    // GOAL POSITION
    // =========================================================================
    // The field coordinates (in inches) of the target the turret should always
    // point at. Pedro Pathing uses (0,0) as one corner of the field; 144 inches
    // is the full field length, so GOAL_Y = 144 places the goal at the far wall.
    public static double GOAL_X = 0;
    public static double GOAL_Y = 144;

    // =========================================================================
    // HARDWARE + STATE VARIABLES
    // =========================================================================
    private DcMotorEx turretMotor;
    private PIDF p, s;   // p = far PIDF, s = near/vision PIDF (see gains above)

    private double t     = 0;     // The calculated TARGET position in encoder ticks
    private double error = 0;     // How far the turret currently is from that target (in ticks)

    private boolean tracking      = false; // Whether the turret is actively trying to track at all
    private boolean wasUsingVision = false; // Remembers which mode was active last loop,
    // so we can detect a mode switch and reset PIDF state

    private Follower follower;

    // The robot's starting position on the field. Pedro Pathing needs this to know
    // where the robot is before it starts moving. X/Y are in inches, heading in radians.
    // 72, 72 = center of the field; Math.toRadians(90) = facing "up" on the field.
    // *** UPDATE THIS to match where you actually place the robot before each match! ***
    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        // =====================================================================
        // INITIALIZATION
        // =====================================================================
        // Set up the Pedro Pathing odometry follower so we always know
        // where the robot is on the field, even without vision.
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize the Limelight camera. We set it to poll at 80Hz (80 reads
        // per second) and use pipeline 0 (AprilTag detection).
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(80);
        limelight.pipelineSwitch(0);
        limelight.start();

        // Set up the turret motor. We reset the encoder to 0 here so the
        // turret's starting position is always treated as "tick 0."
        // RUN_WITHOUT_ENCODER means we manually send power values (-1 to 1)
        // rather than using the built-in motor controller velocity mode.
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setPower(0);

        // Create both PIDF instances with their respective gain values.
        p = new PIDF(kp, 0, kd, kf); // far PIDF  — aggressive, for large errors
        s = new PIDF(sp, 0, sd, sf); // near PIDF — gentle, for small errors and vision

        // Pre-calculate where the turret should be pointing based on startPose,
        // so there's no sudden jerk on the very first loop cycle when tracking begins.
        seedTargetFromPose(startPose);
        tracking = true;

        telemetry.addData("Status", "Initialized & Tracking Active");
        telemetry.update();

        waitForStart();

        // =====================================================================
        // MAIN LOOP — runs continuously while the OpMode is active
        // =====================================================================
        while (opModeIsActive()) {

            // Update Pedro Pathing's odometry so currentPose reflects the robot's
            // actual position on the field this loop cycle.
            follower.update();
            Pose currentPose = follower.getPose();

            // ---------------------------------------------------------------
            // LIMELIGHT — check for AprilTags this loop cycle
            // ---------------------------------------------------------------
            LLResult result = limelight.getLatestResult();
            boolean usingVision       = false; // Will be set to true if we find a valid tag
            double targetOffsetDegrees = 0;    // The deliberate aim offset for each tag
            int lockedTagId            = -1;   // Which tag we're locked onto (-1 = none)

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult tag : fiducialResults) {
                    int id = tag.getFiducialId();
                    if (id == 20) {
                        // Tag 20: aim 5° to the LEFT of center
                        // (negative = left in Limelight's tx convention)
                        targetOffsetDegrees = -5.0;
                        usingVision = true;
                        lockedTagId = 20;
                        break;
                    } else if (id == 24) {
                        // Tag 24: aim 5° to the RIGHT of center
                        targetOffsetDegrees = 5.0;
                        usingVision = true;
                        lockedTagId = 24;
                        break;
                    }
                }
            }

            // ---------------------------------------------------------------
            // TURRET CONTROL — only runs if tracking is enabled
            // ---------------------------------------------------------------
            if (tracking) {
                double currentPosition = turretMotor.getCurrentPosition(); // encoder ticks
                double errorInRadians  = 0;

                if (usingVision && result != null) {
                    // =============================================================
                    // MODE 1: FINE VISION TRACKING (Limelight is seeing a tag)
                    // =============================================================
                    // tx is the horizontal angle (in degrees) from the Limelight's
                    // crosshair to the detected tag. Negative = tag is left of center,
                    // positive = tag is right of center.
                    //
                    // We want tx to settle at targetOffsetDegrees (e.g. -5° for tag 20),
                    // NOT at 0°. So the error is how far tx currently is from that target.
                    //
                    // VISION_SIGN = -1.0 corrects for the fact that a positive PIDF
                    // output turns this turret in the direction that *increases* tx,
                    // which is backwards from what the math expects.

                    // Wipe stale PIDF state on the first cycle of vision mode,
                    // so values from odometry mode don't cause a jerk on entry.
                    if (!wasUsingVision) {
                        s.reset();
                    }

                    double tx = result.getTx();
                    double visionErrorDegrees = VISION_SIGN * (tx - targetOffsetDegrees);

                    // Deadband: if the error is tiny (less than 0.5°), treat it as zero.
                    // This prevents sensor noise from constantly triggering small
                    // corrections that cause a slow, jittery drift near the target.
                    double VISION_DEADBAND_DEG = 0.5;
                    if (Math.abs(visionErrorDegrees) < VISION_DEADBAND_DEG) {
                        visionErrorDegrees = 0;
                    }

                    errorInRadians = Math.toRadians(visionErrorDegrees);

                    // Convert the angular error to ticks for display in telemetry,
                    // and update t so telemetry shows the current vision target position.
                    error = errorInRadians / rpt;
                    t = currentPosition + error;

                    s.updateError(errorInRadians);

                    // Scale feedforward down as error approaches zero, instead of
                    // always applying the full ±sf kick at any nonzero error.
                    // At 5° error -> ffScale = 1.0 (full feedforward)
                    // At 2.5° error -> ffScale = 0.5 (half feedforward)
                    // At 0° error -> ffScale = 0.0 (no feedforward, just hold)
                    // This is what prevents the slow creep-and-jerk behavior.
                    double ffScale = Math.min(1.0, Math.abs(visionErrorDegrees) / 5.0);
                    s.updateFeedForwardInput(Math.signum(errorInRadians) * ffScale);

                    // Clamp output so the turret can't slam at full power during
                    // vision tracking (e.g. if the tag suddenly re-appears far off-center).
                    double visionPower = s.run();
                    visionPower = Math.max(-VISION_MAX_POWER, Math.min(VISION_MAX_POWER, visionPower));
                    turretMotor.setPower(visionPower);

                } else {
                    // =============================================================
                    // MODE 2: COARSE ODOMETRY TRACKING (no tag visible — fallback)
                    // =============================================================
                    // Uses the robot's known field position (from Pedro Pathing) to
                    // calculate the angle from the robot to the goal, then figures out
                    // how much the turret needs to rotate relative to the robot's heading.

                    // Wipe stale PIDF state on the first cycle back in odometry mode,
                    // same reason as above — avoid jerks from leftover vision state.
                    if (wasUsingVision) {
                        s.reset();
                        p.reset();
                    }

                    // atan2 gives us the absolute field angle from the robot to the goal.
                    double angleToGoal = Math.atan2(GOAL_Y - currentPose.getY(), GOAL_X - currentPose.getX());

                    // Subtract the robot's heading to get the angle *relative to the robot*.
                    // normalizeAngle keeps the result in the range (-π, π] to avoid wrap-around issues.
                    double robotAngleDiff = normalizeAngle(angleToGoal - currentPose.getHeading());

                    // Clamp to safe physical limits so the turret doesn't try to
                    // rotate past its mechanical stops.
                    // Min: -90° (π/2 radians left), Max: +135° (toRadians(135) right)
                    robotAngleDiff = Math.max(-Math.PI / 2, Math.min(Math.toRadians(135), robotAngleDiff));

                    // Convert the angle (radians) to a target position in encoder ticks.
                    t     = robotAngleDiff / rpt;
                    error = t - currentPosition;
                    errorInRadians = error * rpt;

                    // Switch between far and near PIDF based on how far the turret
                    // still needs to travel. Far = fast approach, near = smooth settle.
                    if (Math.abs(error) > pidfSwitch) {
                        p.updateError(errorInRadians);
                        p.updateFeedForwardInput(Math.signum(errorInRadians));
                        turretMotor.setPower(p.run());
                    } else {
                        s.updateError(errorInRadians);
                        s.updateFeedForwardInput(Math.signum(errorInRadians));
                        turretMotor.setPower(s.run());
                    }
                }
            } else {
                // Tracking was disabled (driver pressed Back/Share) — stop the turret.
                turretMotor.setPower(0);
            }

            // Remember which mode we were in this cycle, so next cycle we can
            // detect a mode switch and reset PIDF state if needed.
            wasUsingVision = usingVision;

            // Driver can disable tracking by pressing Back (Xbox) or Share (PS4).
            if (gamepad1.back || gamepad1.share) {
                tracking = false;
            }

            // ---------------------------------------------------------------
            // TELEMETRY — displayed on the Driver Station screen each loop
            // ---------------------------------------------------------------
            telemetry.addData("Tracking Mode", usingVision ? "VISION (Limelight)" : "ODOMETRY (Pedro)");
            if (usingVision && result != null) {
                telemetry.addData("Locked Tag ID", lockedTagId);
                telemetry.addData("tx (raw, deg)", "%.2f", result.getTx());   // Raw Limelight angle
                telemetry.addData("Target Offset (deg)", targetOffsetDegrees); // Where we want tx to settle
                telemetry.addData("VISION_SIGN", VISION_SIGN);                 // Flip check (-1.0 = correct)
            }
            telemetry.addData("Robot Pose", "X: %.2f, Y: %.2f, Heading: %.2f°",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Turret Target (t)", t);                         // Where turret is trying to go (ticks)
            telemetry.addData("Turret Current Pos", turretMotor.getCurrentPosition()); // Where it actually is (ticks)
            telemetry.addData("Turret Error (Ticks)", error);                  // Difference (should approach 0)
            telemetry.update();
        }

        limelight.stop();
    }

    // =========================================================================
    // seedTargetFromPose()
    // =========================================================================
    // Pre-calculates where the turret SHOULD be pointing based on a given pose,
    // and stores that as the initial target (t). Called once during init so the
    // turret doesn't start from t=0 and lurch to the correct angle on the first
    // loop cycle — it already knows where it should be before the match starts.
    private void seedTargetFromPose(Pose pose) {
        double angleToGoal    = Math.atan2(GOAL_Y - pose.getY(), GOAL_X - pose.getX());
        double robotAngleDiff = normalizeAngle(angleToGoal - pose.getHeading());
        robotAngleDiff = Math.max(-Math.PI / 2, Math.min(Math.toRadians(135), robotAngleDiff));
        t = robotAngleDiff / rpt;
    }

    // =========================================================================
    // normalizeAngle()
    // =========================================================================
    // Wraps any angle (in radians) into the range (-π, π].
    // This prevents issues like a 359° angle being treated as "far from" a 1°
    // angle when they're actually only 2° apart. Without this, the turret could
    // try to spin the long way around instead of the short way.
    private static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        if (angle > Math.PI)   angle -= Math.PI * 2D;
        return angle;
    }
}