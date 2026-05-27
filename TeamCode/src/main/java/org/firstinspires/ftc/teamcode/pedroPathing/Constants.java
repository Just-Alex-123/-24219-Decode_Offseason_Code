package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Pinpoint;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    static MecanumConfig mecanumConfig = new MecanumConfig(
            c -> {
                c.leftFrontName.set("leftFront");
                c.leftRearName.set("leftRear");
                c.rightFrontName.set("rightFront");
                c.rightRearName.set("rightRear");

                c.leftFrontDirection.set(DcMotorSimple.Direction.FORWARD);
                c.leftRearDirection.set(DcMotorSimple.Direction.FORWARD);
                c.rightFrontDirection.set(DcMotorSimple.Direction.REVERSE);
                c.rightRearDirection.set(DcMotorSimple.Direction.REVERSE);

                c.manualBrakeMode.set(true);
            }
    );

    static PinpointConfig pinpointConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");

                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);

                c.xPodOffset.set(0.0);
                c.yPodOffset.set(0.0);

                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            }
    );

    static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                c.linearBrakeCoefficients.set(Matrix.diag(0.139333365, 0.139333365));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.000210842, 0.000210842));
                c.maxAchievableForwardVelocity.set(88.036);
                c.maxAchievableStrafeVelocity.set(71.881);
                c.maxAchievableForwardDeceleration.set(30.3333);
                c.maxAchievableStrafeDeceleration.set(62.58098);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(new Pinpoint(h, pinpointConfig), new Mecanum(h, mecanumConfig), new Foresight(foresightConfig));
    }
}
