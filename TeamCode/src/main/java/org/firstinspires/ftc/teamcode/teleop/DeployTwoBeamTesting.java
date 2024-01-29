package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "DeployTwoBeamTesting")
public class DeployTwoBeamTesting extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Servo servoRotationBeamLeft;
    private Servo servoRotationBeamRight;
    private Servo servoRotationBox;
    private Servo servoHoldLower;
    public FtcDashboard dashboard;
    public static double SERVO_POS = 0.88;
    public static double SERVO_BOX_POS = 0.32;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {
            servoRotationBeamLeft.setPosition(SERVO_POS);
            servoRotationBeamRight.setPosition(SERVO_POS);
            servoRotationBox.setPosition(SERVO_BOX_POS);
            if (gamepad1.a) {
                servoHoldLower.setPosition(0);
            } else {
                servoHoldLower.setPosition(0.5);
            }

            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        servoRotationBox = hardwareMap.get(Servo.class, "servo_rotation_box");

        servoRotationBeamLeft = hardwareMap.get(Servo.class, "servo_rotation_beam_left");
        servoRotationBeamRight = hardwareMap.get(Servo.class, "servo_rotation_beam_right");
        servoRotationBeamRight.setDirection(Servo.Direction.REVERSE);

        servoHoldLower = hardwareMap.get(Servo.class, "servo_hold_lower");

        telemetry.update();
    }
}
