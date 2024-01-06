package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Lift;

@Config
@TeleOp(name = "DeployTwoBeamTesting")
public class DeployTwoBeamTesting extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Servo servoRotationBeamLeft;
    private Servo servoRotationBeamRight;
    private Servo servoRotationBox;
    private Lift lift;
    public FtcDashboard dashboard;
    public static double SERVO_POS = 0.5;
    public static double SERVO_POS_BOX = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {
            lift.opControl();
            servoRotationBeamLeft.setPosition(SERVO_POS - 0.043);
            servoRotationBeamRight.setPosition(SERVO_POS);
            servoRotationBox.setPosition(SERVO_POS_BOX);

            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        servoRotationBeamLeft = hardwareMap.get(Servo.class, "servo_rotation_beam_left");
        servoRotationBeamRight = hardwareMap.get(Servo.class, "servo_rotation_beam_right");
        servoRotationBox = hardwareMap.get(Servo.class, "servo_rotation_box");

        servoRotationBeamLeft.setDirection(Servo.Direction.REVERSE);

        lift = new Lift(this, dashboard);

        telemetry.update();
    }
}
