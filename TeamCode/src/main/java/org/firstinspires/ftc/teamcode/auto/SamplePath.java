package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class SamplePath extends OpMode {
    private Follower       follower;
    private AutoTurretSub  turretSub;
    private AutoShooterSub shooterSub;

    private final Pose startPose        = new Pose(15, 107.5, Math.toRadians(180));
    private final Pose shootingPose1Pos = new Pose(47, 87,    Math.toRadians(160));
    private final Pose row2GrabPos      = new Pose(9, 49,     Math.toRadians(0));
    private final Pose shoot1Pos        = new Pose(54, 79,    Math.toRadians(210));
    private final Pose gateCyclePos     = new Pose(12, 55,    Math.toRadians(0));
    private final Pose shoot2Pos        = new Pose(54, 79,    Math.toRadians(210));
    private final Pose row1Pos          = new Pose(15, 82.5,  Math.toRadians(180));
    private final Pose shoot4Pos        = new Pose(46, 82.5,  Math.toRadians(180));

    private enum State {
        STARTING_POSE,

        SHOOTING_POSE_1,        // driving to first shoot position
        SHOOTING_POSE_1_SHOOT,  // gate open 1.5s

        ROW_2_GRAB,             // driving to grab position
        SHOOT_1_DRIVE,          // driving to shoot_1 position
        SHOOT_1_SHOOT,          // gate open 1.5s

        GATE_CYCLE_1_DRIVE,     // driving gate cycle path
        GATE_CYCLE_1_WAIT,      // 3s wait at gate
        SHOOT_2_DRIVE,          // driving to shoot2 position
        SHOOT_2_SHOOT,          // gate open 1.5s

        GATE_CYCLE_2_DRIVE,     // driving gate cycle path
        GATE_CYCLE_2_WAIT,      // 3s wait at gate
        SHOOT_3_DRIVE,          // driving to shoot position (reuses shoot2 path)
        SHOOT_3_SHOOT,          // gate open 1.5s

        ROW_1_DRIVE,            // driving to row 1
        SHOOT_4_DRIVE,          // driving to shoot4 position
        SHOOT_4_SHOOT,          // gate open 1.5s

        IDLE
    }

    private State currentState  = State.STARTING_POSE;
    private long  waitStartTime = 0;

    private static final long SHOOT_WAIT_MS = 1500; // gate open duration (ms)
    private static final long GATE_WAIT_MS  = 3000; // gate cycle pause (ms)

    private AutoIntakeSub intakeSub;
    private PathChain shootingPose1, row2Grab, shoot_1, gate_cycle, shoot2, row_1, shoot_4;

    @Override
    public void init() {
        follower   = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        intakeSub  = new AutoIntakeSub(hardwareMap);
        turretSub  = new AutoTurretSub(hardwareMap, startPose);
        shooterSub = new AutoShooterSub(hardwareMap);

        turretSub.startTracking();
    }

    public void buildPaths() {
        shootingPose1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(startPose.getX(), startPose.getY()),
                        new Pose(shootingPose1Pos.getX(), shootingPose1Pos.getY())
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose1Pos.getHeading())
                .build();

        row2Grab = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(shootingPose1Pos.getX(), shootingPose1Pos.getY()),
                        new Pose(47, 71),
                        new Pose(38, 50),
                        new Pose(row2GrabPos.getX(), row2GrabPos.getY())
                ))
                .setTangentHeadingInterpolation()
                .build();

        shoot_1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(row2GrabPos.getX(), row2GrabPos.getY()),
                        new Pose(shoot1Pos.getX(), shoot1Pos.getY())
                ))
                .setConstantHeadingInterpolation(Math.toRadians(210))
                .build();

        gate_cycle = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(shoot1Pos.getX(), shoot1Pos.getY()),
                        new Pose(52.5, 60.5),
                        new Pose(40, 46),
                        new Pose(gateCyclePos.getX(), gateCyclePos.getY())
                ))
                .setTangentHeadingInterpolation()
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(gateCyclePos.getX(), gateCyclePos.getY()),
                        new Pose(shoot2Pos.getX(), shoot2Pos.getY())
                ))
                .setLinearHeadingInterpolation(160, shoot2Pos.getHeading())
                .build();

        row_1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(shoot2Pos.getX(), shoot2Pos.getY()),
                        new Pose(row1Pos.getX(), row1Pos.getY())
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shoot_4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(row1Pos.getX(), row1Pos.getY()),
                        new Pose(shoot4Pos.getX(), shoot4Pos.getY())
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    @Override
    public void loop() {
        follower.update();
        turretSub.periodic(follower.getPose());
        intakeSub.intake(1);

        switch (currentState) {

            // ── Start: drive to first shoot position, spin up flywheels ──────
            case STARTING_POSE:
                follower.followPath(shootingPose1, true);
                shooterSub.spinUp(1500);
                currentState = State.SHOOTING_POSE_1;
                break;

            case SHOOTING_POSE_1:
                if (!follower.isBusy()) {
                    shooterSub.openGate();
                    waitStartTime = System.currentTimeMillis();
                    currentState = State.SHOOTING_POSE_1_SHOOT;
                }
                break;
    
            case SHOOTING_POSE_1_SHOOT:
                if (System.currentTimeMillis() - waitStartTime >= SHOOT_WAIT_MS) {
                    shooterSub.closeGate();
                    follower.followPath(row2Grab, true);
                    currentState = State.ROW_2_GRAB;
                }
                break;

            // ── Grab row 2, then drive to shoot_1 ────────────────────────────
            case ROW_2_GRAB:
                if (!follower.isBusy()) {
                    follower.followPath(shoot_1, true);
                    currentState = State.SHOOT_1_DRIVE;
                }
                break;

            case SHOOT_1_DRIVE:
                if (!follower.isBusy()) {
                    shooterSub.openGate();
                    waitStartTime = System.currentTimeMillis();
                    currentState = State.SHOOT_1_SHOOT;
                }
                break;

            case SHOOT_1_SHOOT:
                if (System.currentTimeMillis() - waitStartTime >= SHOOT_WAIT_MS) {
                    shooterSub.closeGate();
                    follower.followPath(gate_cycle, true);
                    currentState = State.GATE_CYCLE_1_DRIVE;
                }
                break;

            // ── Gate cycle 1: drive → 3s wait → drive to shoot2 ──────────────
            case GATE_CYCLE_1_DRIVE:
                if (!follower.isBusy()) {
                    waitStartTime = System.currentTimeMillis();
                    currentState = State.GATE_CYCLE_1_WAIT;
                }
                break;

            case GATE_CYCLE_1_WAIT:
                if (System.currentTimeMillis() - waitStartTime >= GATE_WAIT_MS) {
                    follower.followPath(shoot2, true);
                    currentState = State.SHOOT_2_DRIVE;
                }
                break;

            case SHOOT_2_DRIVE:
                if (!follower.isBusy()) {
                    shooterSub.openGate();
                    waitStartTime = System.currentTimeMillis();
                    currentState = State.SHOOT_2_SHOOT;
                }
                break;

            case SHOOT_2_SHOOT:
                if (System.currentTimeMillis() - waitStartTime >= SHOOT_WAIT_MS) {
                    shooterSub.closeGate();
                    follower.followPath(gate_cycle, true);
                    currentState = State.GATE_CYCLE_2_DRIVE;
                }
                break;

            // ── Gate cycle 2: drive → 3s wait → drive to shoot3 ──────────────
            case GATE_CYCLE_2_DRIVE:
                if (!follower.isBusy()) {
                    waitStartTime = System.currentTimeMillis();
                    currentState = State.GATE_CYCLE_2_WAIT;
                }
                break;

            case GATE_CYCLE_2_WAIT:
                if (System.currentTimeMillis() - waitStartTime >= GATE_WAIT_MS) {
                    follower.followPath(shoot2, true);
                    currentState = State.SHOOT_3_DRIVE;
                }
                break;

            case SHOOT_3_DRIVE:
                if (!follower.isBusy()) {
                    shooterSub.openGate();
                    waitStartTime = System.currentTimeMillis();
                    currentState = State.SHOOT_3_SHOOT;
                }
                break;

            case SHOOT_3_SHOOT:
                if (System.currentTimeMillis() - waitStartTime >= SHOOT_WAIT_MS) {
                    shooterSub.closeGate();
                    follower.followPath(row_1, true);
                    currentState = State.ROW_1_DRIVE;
                }
                break;

            // ── Row 1 → shoot4 ────────────────────────────────────────────────
            case ROW_1_DRIVE:
                if (!follower.isBusy()) {
                    follower.followPath(shoot_4, true);
                    currentState = State.SHOOT_4_DRIVE;
                }
                break;

            case SHOOT_4_DRIVE:
                if (!follower.isBusy()) {
                    shooterSub.openGate();
                    waitStartTime = System.currentTimeMillis();
                    currentState = State.SHOOT_4_SHOOT;
                }
                break;

            case SHOOT_4_SHOOT:
                if (System.currentTimeMillis() - waitStartTime >= SHOOT_WAIT_MS) {
                    shooterSub.closeGate();
                    shooterSub.stop();
                    currentState = State.IDLE;
                }
                break;

            case IDLE:
                break;
        }
    }
}