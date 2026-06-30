package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {

    private DcMotor intake;

    private boolean IntakeOn = false;
    private boolean lastBumper = false;
    private boolean currentBumper = false;

    public intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(boolean bumperPressed) {
        currentBumper = bumperPressed;

        if (currentBumper && !lastBumper) {
            IntakeOn = !IntakeOn;
        }

        lastBumper = currentBumper;
        intake.setPower(IntakeOn ? 1.0 : 0.0);
    }

    public boolean isIntakeOn() {
        return IntakeOn;
    }
}