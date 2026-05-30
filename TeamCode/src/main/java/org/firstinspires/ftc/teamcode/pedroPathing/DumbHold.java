/**
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.FollowState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Utils;

public class DumbHold implements Algorithm {
    private final ForesightConfig config;

    public DumbHold(ForesightConfig config) {
        this.config = config;
    }

    @Override
    public DrivePowers calculate(FollowState state) {
        double t = state.pathTracker()
                .current()
                .closestT(state.motionState().pose().toVector2D());
        double targetHeading = state.pathTracker().current().heading(t);

        if (t >= (1 - config.parametricTConstraint.get())) { // End Constraint
            if (state.pathTracker().size() > 1) { // advance if constraints met
                state.pathTracker().advance();
                return calculate(state);
            }

            state.pathTracker().isFollowing(false);
            return hold(state.pathTracker().current().endPoint().toPose(targetHeading), state);
        }

        return hold(state.pathTracker().current().endPoint().toPose(targetHeading), state);
    }

    @Override
    public DrivePowers hold(Pose target, FollowState state) {
        Vector2D translationalError = target.minus(state.motionState().pose()).toVector2D();

        Vector2D linearVel = state.motionState().velocity().toLinear();

        Vector2D brakingDisplacement = Vector2D.cartesian(
                getBrakingDisplacement(linearVel.x(), config.quadraticBrakeCoefficients.get().get(0,0), config.linearBrakeCoefficients.get().get(0,0)),
                getBrakingDisplacement(linearVel.y(), config.quadraticBrakeCoefficients.get().get(0,0), config.linearBrakeCoefficients.get().get(0,0)));

        Vector2D adjustedError = translationalError.minus(brakingDisplacement);

        // make sure there is no D term
        Vector2D translational = Vector2D.cartesian(
                config.translationalController.get().calculate(0, adjustedError.x()),
                config.translationalController.get().calculate(0, adjustedError.y())
        );
        double headingPower = headingPower(state, target.heading());
        return getDrivePowers(translational, state, headingPower);
    }

    /**
     * Compute heading correction power for the given state and target heading.
     */
    public double headingPower(FollowState state, double targetHeading) {
        double current = state.motionState().pose().heading();
        double error = headingError(current, targetHeading);
        return config.headingController.get().calculate(targetHeading, error);
    }

    public DrivePowers getDrivePowers(Vector2D fieldRelativeDrivePower, FollowState state, double headingPower) {
        Vector2D robotFrameDrivePower = fieldRelativeDrivePower.rotate(-state.motionState().pose().heading());
        double forward = Utils.Control.clampBrakingPower(robotFrameDrivePower.x(), state.motionState().twist().vx(), config.maxBrakingPower.get());
        double strafe = Utils.Control.clampBrakingPower(robotFrameDrivePower.y(), state.motionState().twist().vy(), config.maxBrakingPower.get());
        return new DrivePowers(forward, strafe, headingPower);
    }

    public double headingError(double current, double target) {
        return Utils.Angle.smallestDifference(current, target) * Utils.Angle.turnDirection(current, target);
    }

    public double getBrakingDisplacement(double velocity, double quad, double linear) {
        return velocity * linear + velocity * Math.abs(velocity) * quad;
    }
}