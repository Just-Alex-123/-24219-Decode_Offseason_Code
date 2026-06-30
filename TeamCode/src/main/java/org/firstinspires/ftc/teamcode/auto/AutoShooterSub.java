package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * AutoShooterSub
 *
 * Autonomous-only flywheel + counter-roller + gate subsystem.
 * Gate defaults to CLOSED always — call openGate() to shoot, closeGate() when done.
 *
 * Usage in SamplePath.java:
 *
 *   private AutoShooterSub shooterSub;
 *
 *   // in init():
 *   shooterSub = new AutoShooterSub(hardwareMap);
 *
 *   // to shoot:
 *   shooterSub.spinUp(1470);          // spin flywheels to target velocity
 *   // ... wait for isReady() via state machine ...
 *   shooterSub.openGate();            // open gate to let ring through
 *   shooterSub.feed(true);            // run CR roller to push ring
 *   // ... wait via feedTimer ...
 *   shooterSub.feed(false);
 *   shooterSub.closeGate();           // close gate (also called automatically by stop())
 *   shooterSub.stop();                // stop flywheels, close gate, stop CR
 */
public class AutoShooterSub {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
    private final DcMotorEx fwL, fwR;
    private final CRServo   cr;
    private final Servo     gate;

    // -------------------------------------------------------------------------
    // PIDF Coefficients — match your teleop tuned values
    // -------------------------------------------------------------------------
    private static final double PIDF_P = 20.0;
    private static final double PIDF_I = 1.5;
    private static final double PIDF_D = 0.0;
    private static final double PIDF_F = 13.5;

    // -------------------------------------------------------------------------
    // Velocity tolerance for isReady() (ticks/sec)
    // -------------------------------------------------------------------------
    private static final double VELOCITY_TOLERANCE = 50.0;

    // -------------------------------------------------------------------------
    // CR servo power values
    // -------------------------------------------------------------------------
    private static final double CR_FORWARD = 1.0;
    private static final double CR_STOP    = 0.0;

    // -------------------------------------------------------------------------
    // Gate servo positions
    // -------------------------------------------------------------------------
    private static final double GATE_OPEN   = 0.45;
    private static final double GATE_CLOSED = 0.49;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    private double targetVelocity = 0.0;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------
    public AutoShooterSub(HardwareMap hardwareMap) {
        fwL = hardwareMap.get(DcMotorEx.class, "fwL");
        fwR = hardwareMap.get(DcMotorEx.class, "fwR");
        cr  = hardwareMap.get(CRServo.class,   "cr");
        gate = hardwareMap.get(Servo.class,    "gate");

        fwL.setDirection(DcMotorSimple.Direction.FORWARD);
        fwR.setDirection(DcMotorSimple.Direction.REVERSE);

        fwL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fwR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fwL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fwR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fwL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fwR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        applyPIDF();

        gate.setPosition(GATE_CLOSED); // always closed on init
        cr.setPower(CR_STOP);
    }

    // -------------------------------------------------------------------------
    // Gate controls — call openGate() to shoot, closeGate() when done.
    // Gate defaults to CLOSED on init and after stop().
    // -------------------------------------------------------------------------
    public void openGate()  { gate.setPosition(GATE_OPEN);   }
    public void closeGate() { gate.setPosition(GATE_CLOSED); }

    // -------------------------------------------------------------------------
    // Spin the flywheels up to a target velocity (ticks/sec).
    // Call this when you enter a shooting state.
    // -------------------------------------------------------------------------
    public void spinUp(double velocityTicksPerSec) {
        targetVelocity = velocityTicksPerSec;
        fwL.setVelocity(targetVelocity);
        fwR.setVelocity(targetVelocity);
    }

    // -------------------------------------------------------------------------
    // Stop both flywheels and the CR roller.
    // Call this after a shot is complete.
    // -------------------------------------------------------------------------
    public void stop() {
        targetVelocity = 0.0;
        fwL.setVelocity(0);
        fwR.setVelocity(0);
        cr.setPower(CR_STOP);
        gate.setPosition(GATE_CLOSED);
    }

    // -------------------------------------------------------------------------
    // Run or stop the counter roller (CR servo).
    //   feed(true)  — roller on, feeds ring into flywheels
    //   feed(false) — roller off
    // -------------------------------------------------------------------------
    public void feed(boolean on) {
        cr.setPower(on ? CR_FORWARD : CR_STOP);
    }

    // -------------------------------------------------------------------------
    // Returns true when both flywheels are within tolerance of target velocity.
    // Use this to gate your feed command so you don't shoot early.
    // -------------------------------------------------------------------------
    public boolean isReady() {
        if (targetVelocity == 0) return false;
        return Math.abs(fwL.getVelocity() - targetVelocity) < VELOCITY_TOLERANCE
                && Math.abs(fwR.getVelocity() - targetVelocity) < VELOCITY_TOLERANCE;
    }

    // -------------------------------------------------------------------------
    // Getters — useful for telemetry while debugging auto
    // -------------------------------------------------------------------------
    public double getLeftVelocity()   { return fwL.getVelocity(); }
    public double getRightVelocity()  { return fwR.getVelocity(); }
    public double getTargetVelocity() { return targetVelocity; }

    // -------------------------------------------------------------------------
    // PIDF — applied once at init
    // -------------------------------------------------------------------------
    private void applyPIDF() {
        PIDFCoefficients pidf = new PIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F);
        fwL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        fwR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
}