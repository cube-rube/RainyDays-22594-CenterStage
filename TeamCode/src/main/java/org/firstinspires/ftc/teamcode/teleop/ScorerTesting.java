package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.Scorer;

@Config
@TeleOp(name = "ScorerTesting")
public class ScorerTesting extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Lift lift;
    private Scorer scorer;
    public FtcDashboard dashboard;
    public static double SERVO_POS = 0.5;
    public static double SERVO_POS_BOX = 0.5;
    public static double diff = 0.043;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {
            lift.opControl();
            scorer.opControl();

            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        lift = new Lift(this, dashboard);
        scorer = new Scorer(this, lift);

        telemetry.update();
    }
}
