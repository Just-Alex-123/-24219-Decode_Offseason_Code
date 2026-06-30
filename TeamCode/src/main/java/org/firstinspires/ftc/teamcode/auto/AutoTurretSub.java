package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class AutoTurretSub {

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
    // Tunable constants
    // -------------------------------------------------------------------------
    public static double rpt        = 0.00268785;
    public static double pidfSwitch = 60;

    public static double kp = 1.5, kf = 0.05, kd = 0.005; // far PIDF
    public static double sp = 0.5, sf = 0.03, sd = 0.001;  // near PIDF

    // Goal position on the field (inches)
    public static double GOAL_X = 0;
    public static double GOAL_Y = 144;

    // -------------------------------------------------------------------------
    // Hardware + state
    // -------------------------------------------------------------------------
    public final DcMotorEx m;
    private final PIDF p, s;

    private double t               = 0;
    private double error           = 0;
    private double currentPosition = 0;
    private boolean tracking       = false;

    // The robot's starting pose — used to seed the turret heading on init
    private final Pose startPose;

    // -------------------------------------------------------------------------
    // Constructor — pass in your auto's startPose so the turret knows where
    // the robot begins on the field instead of assuming 0, 0, 0.
    // -------------------------------------------------------------------------
    public AutoTurretSub(HardwareMap hardwareMap, Pose startPose) {
        this.startPose = startPose;

        m = hardwareMap.get(DcMotorEx.class, "turret");
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m.setPower(0);

        p = new PIDF(kp, 0, kd, kf);
        s = new PIDF(sp, 0, sd, sf);

        // Pre-aim the turret target based on start pose so there's no
        // sudden jump when tracking begins on the first loop() cycle
        seedTargetFromPose(startPose);
    }

    // -------------------------------------------------------------------------
    // Call startTracking() once in init() — turret will continuously face
    // the goal using the robot's current Pedro pose every loop cycle.
    // -------------------------------------------------------------------------
    public void startTracking() { tracking = true;  }
    public void stopTracking()  { tracking = false; }

    // -------------------------------------------------------------------------
    // Call every loop() after follower.update()
    // -------------------------------------------------------------------------
    public void periodic(Pose robotPose) {
        if (!tracking) {
            m.setPower(0);
            return;
        }

        currentPosition = m.getCurrentPosition();

        double angleToGoal    = Math.atan2(GOAL_Y - robotPose.getY(), GOAL_X - robotPose.getX());
        double robotAngleDiff = normalizeAngle(angleToGoal - robotPose.getHeading());
        robotAngleDiff = Math.max(-Math.PI / 2, Math.min(Math.toRadians(135), robotAngleDiff));

        t     = robotAngleDiff / rpt;
        error = t - currentPosition;

        double errorInRadians = error * rpt;

        if (Math.abs(error) > pidfSwitch) {
            p.updateError(errorInRadians);
            p.updateFeedForwardInput(Math.signum(errorInRadians));
            m.setPower(p.run());
        } else {
            s.updateError(errorInRadians);
            s.updateFeedForwardInput(Math.signum(errorInRadians));
            m.setPower(s.run());
        }
    }

    // -------------------------------------------------------------------------
    // Getters
    // -------------------------------------------------------------------------
    public double  getError()   { return error; }
    public boolean isReady()    { return Math.abs(error) < 20; }
    public boolean isTracking() { return tracking; }

    // -------------------------------------------------------------------------
    // Computes and stores the initial turret target from a given pose,
    // so the PIDF doesn't start from a stale t = 0 on the first cycle.
    // -------------------------------------------------------------------------
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