package org.firstinspires.ftc.teamcode.main;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.test.Turret_LM;

public class teleop extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private Follower follower;
    private intake intake;
//    private shooter shooter;
//    private Turret_LM turret;
    private Servo rgb, hood;

    @Override
    public void runOpMode() {
        intake = new intake(hardwareMap);



        waitForStart();
        while (opModeIsActive()){

            intake.update(gamepad1.left_trigger > 0.5);

        }
    }
}
