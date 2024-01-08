package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.modules.Lift;

@Config
@TeleOp(name = "HolderTesting")
public class HolderTesting extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Servo servoHoldLower;
    private Servo servoHoldUpper;
    public FtcDashboard dashboard;
    public static double SERVO_POS_LOWER = 0.5;
    public static double SERVO_POS_UPPER = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {
            servoHoldLower.setPosition(SERVO_POS_LOWER);
            servoHoldUpper.setPosition(SERVO_POS_UPPER);

            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        servoHoldUpper = hardwareMap.get(Servo.class, "servo_hold_upper");
        servoHoldLower = hardwareMap.get(Servo.class, "servo_hold_lower");

        servoHoldUpper.setDirection(Servo.Direction.REVERSE);

        telemetry.update();
    }
}
