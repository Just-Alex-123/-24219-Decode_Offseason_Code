package org.firstinspires.ftc.teamcode.pedroPathing;

import androidx.core.graphics.drawable.IconCompat;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.CompoundPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.SimplePath;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.curves.Line;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class LineTest extends OpMode {
    public static double DISTANCE = 24;
    public double loops = 0, lastLoop = 0, loopTime = 0;
    private Path line1, line2;
    private boolean forward;
    private Follower follower;
    private MultipleTelemetry multipleTelemetry;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.pose(new Pose(72, 72, 0));

        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        Curve forwards = new Line(new Pose(72,72, 0), new Pose(DISTANCE + 72,72, 0));
        Curve backwards = new Line(new Pose(DISTANCE + 72,72, 0), new Pose(72,72, 0));
        line1 = new SimplePath(forwards, Interpolator.constant(0));
        line2 = new SimplePath(backwards, Interpolator.constant(0));
        follower.follow(line1);
    }

    @Override
    public void loop() {
        loops++;

        if (loops > 10) {
            double now = System.currentTimeMillis();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        follower.update();

        if (!follower.isBusy()) {
            if (forward) {
                follower.follow(line2);
            } else {
                follower.follow(line1);
            }
            forward = !forward;
        }

        multipleTelemetry.addData("Loop Time Hz", 1000/loopTime);
        multipleTelemetry.addData("Is Busy?", follower.isBusy());
        multipleTelemetry.addData("Pose", follower.pose());
        DrivePowers hold = DrivePowers.zero();
        if (follower.state() != null) {
            if (follower.state().pathTracker() != null) {
                multipleTelemetry.addData("Empty", follower.state().pathTracker().empty());
                multipleTelemetry.addData("Is Following", follower.state().pathTracker().isFollowing());
                multipleTelemetry.addData("Size", follower.state().pathTracker().size());
                multipleTelemetry.addData("End", follower.state().pathTracker().end());
                multipleTelemetry.addLine();
//                hold = follower.algorithm().hold(follower.state().pathTracker().end(), follower.state());
            }
        }
//        multipleTelemetry.addData("Forward", hold.forward());
//        multipleTelemetry.addData("Strafe", hold.strafe());
//        multipleTelemetry.addData("Turn", hold.turn());
        multipleTelemetry.update();
    }
}
