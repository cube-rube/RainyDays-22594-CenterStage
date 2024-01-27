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
    public FtcDashboard dashboard;
    public static double SERVO_POS = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {
            servoRotationBeamLeft.setPosition(SERVO_POS);
            servoRotationBeamLeft.getConnectionInfo();

            telemetry.addData("Runtime", runtime.toString());
            telemetry.addData("", servoRotationBeamLeft.getConnectionInfo());
            telemetry.addData("", servoRotationBeamLeft.getDeviceName());
            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        servoRotationBeamLeft = hardwareMap.get(Servo.class, "servo_finger");
        telemetry.update();
    }
}
