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

    // -------------------------------------------------------------------------
    // PIDF Inner Class
    // -------------------------------------------------------------------------
    private static class PIDF {
        private double kP, kI, kD, kF;
        private double lastError = 0, integral = 0;
        private double feedForward = 0;

        public PIDF(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }

        public void updateError(double error) {
            integral += error;
            lastError = error;
        }

        public void updateFeedForwardInput(double ff) {
            feedForward = ff;
        }

        public double run() {
            return kP * lastError + kI * integral + kD * lastError + kF * feedForward;
        }
    }

    // -------------------------------------------------------------------------
    // Tunable constants & Hardware
    // -------------------------------------------------------------------------
    public static double rpt        = 0.00268785;
    public static double pidfSwitch = 60;
    public static double kp = 1.5, kf = 0.05, kd = 0.005;
    public static double sp = 0.5, sf = 0.03, sd = 0.001;

    // Goal position on the field (inches)
    public static double GOAL_X = 0;
    public static double GOAL_Y = 144;

    private DcMotorEx turretMotor;
    private PIDF p, s;
    private double t = 0;
    private double error = 0;
    private boolean tracking = false;

    // Pedro Pathing Follower (Used strictly for localizing via odometry pods)
    private Follower follower;
    private final Pose startPose = new Pose(72, 72, 90);

    // Limelight Hardware Instance
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // 1. Initialize Pedro Pathing for tracking location only
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // 2. Initialize Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(80);
        limelight.pipelineSwitch(1);   // Switch to pipeline 1
        limelight.start();

        // 3. Initialize Turret Hardware
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setPower(0);

        // 4. Initialize PIDFs
        p = new PIDF(kp, 0, kd, kf);
        s = new PIDF(sp, 0, sd, sf);

        // Pre-aim target based on start pose
        seedTargetFromPose(startPose);

        // Set tracking to true immediately during initialization phase
        tracking = true;

        telemetry.addData("Status", "Initialized & Tracking Active");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update Pedro Pathing Odometry to get current coordinates
            follower.update();
            Pose currentPose = follower.getPose();

            // Read the latest Limelight data frame
            LLResult result = limelight.getLatestResult();
            boolean usingVision = false;
            double targetOffsetDegrees = 0;

            // Check if Limelight data is fresh and contains target data
            // Check if Limelight data is fresh and contains target data
            if (result != null && result.isValid()) {
                // Correct method: getFiducialResults() instead of getBarcodeResults()
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                for (LLResultTypes.FiducialResult tag : fiducialResults) {
                    // Correct method: getFiducialId() returns a standard int directly! No need to parse a String.
                    int id = tag.getFiducialId();

                    if (id == 20) {
                        targetOffsetDegrees = -5.0; // 5 degrees to the left
                        usingVision = true;
                        break;
                    } else if (id == 24) {
                        targetOffsetDegrees = 5.0;  // 5 degrees to the right
                        usingVision = true;
                        break;
                    }
                }
            }

            // Execute Turret Tracking Logic
            if (tracking) {
                double currentPosition = turretMotor.getCurrentPosition();
                double errorInRadians = 0;

                if (usingVision && result != null) {
                    // ---- FINE VISION TRACKING ----
                    double tx = result.getTx(); // Raw offset angle from camera lens center

                    // Physical alignment error: target an offset relative to center
                    double visionErrorDegrees = tx - targetOffsetDegrees;

                    // Convert degrees to radians to align safely with your PIDF processing bounds
                    errorInRadians = Math.toRadians(visionErrorDegrees);

                    // Back-calculate what the pseudo target tick (t) would be for telemetry clarity
                    error = errorInRadians / rpt;
                    t = currentPosition + error;

                } else {
                    // ---- COARSE ODOMETRY TRACKING (FALLBACK) ----
                    double angleToGoal    = Math.atan2(GOAL_Y - currentPose.getY(), GOAL_X - currentPose.getX());
                    double robotAngleDiff = normalizeAngle(angleToGoal - currentPose.getHeading());
                    robotAngleDiff = Math.max(-Math.PI / 2, Math.min(Math.toRadians(135), robotAngleDiff));

                    t     = robotAngleDiff / rpt;
                    error = t - currentPosition;
                    errorInRadians = error * rpt;
                }

                // Push error to selected profile loops
                if (Math.abs(error) > pidfSwitch) {
                    p.updateError(errorInRadians);
                    p.updateFeedForwardInput(Math.signum(errorInRadians));
                    turretMotor.setPower(p.run());
                } else {
                    s.updateError(errorInRadians);
                    s.updateFeedForwardInput(Math.signum(errorInRadians));
                    turretMotor.setPower(s.run());
                }
            } else {
                turretMotor.setPower(0);
            }

            // Optional: Back/Share button acts as a safety kill-switch for the turret power
            if (gamepad1.back || gamepad1.share) {
                tracking = false;
            }

            // Telemetry Outputs
            telemetry.addData("Tracking Mode", usingVision ? "VISION (Limelight)" : "ODOMETRY (Pedro)");
            telemetry.addData("Robot Pose", "X: %.2f, Y: %.2f, Heading: %.2f°",
                    currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Turret Target (t)", t);
            telemetry.addData("Turret Current Pos", turretMotor.getCurrentPosition());
            telemetry.addData("Turret Error", error);
            telemetry.update();
        }

        // Clean up limelight stream on completion
        limelight.stop();
    }

    private void seedTargetFromPose(Pose pose) {
        double angleToGoal    = Math.atan2(GOAL_Y - pose.getY(), GOAL_X - pose.getX());
        double robotAngleDiff = normalizeAngle(angleToGoal - pose.getHeading());
        robotAngleDiff = Math.max(-Math.PI / 2, Math.min(Math.toRadians(135), robotAngleDiff));
        t = robotAngleDiff / rpt;
    }

    private static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        if (angle > Math.PI)   angle -= Math.PI * 2D;
        return angle;
    }
}