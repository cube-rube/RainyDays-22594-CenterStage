package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Shooter;

@Config
@TeleOp(group = "test")
public class ShooterTesting extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Shooter shooter;
    public FtcDashboard dashboard;
    public static double SERVO_POS = 0.68;
    public static double SERVO_BOX_POS = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {
            shooter.teleOneButton();

            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        shooter = new Shooter(this);

        telemetry.update();
    }
}
