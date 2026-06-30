package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoIntakeSub {

    private DcMotor intake;

    public AutoIntakeSub(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intake(double power) {
        intake.setPower(power);
    }

    public void outtake(double power) {
        intake.setPower(-power);
    }

    public void stop() {
        intake.setPower(0);
    }
}