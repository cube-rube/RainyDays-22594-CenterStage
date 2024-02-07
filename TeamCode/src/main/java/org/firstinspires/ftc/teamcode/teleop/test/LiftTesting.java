package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.Scorer;

@TeleOp(group = "test")
public class LiftTesting extends LinearOpMode {
    public Lift lift;
    public Scorer scorer;
    public Intake intake;
    private final ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();
        lift.timer.reset();

        while (opModeIsActive()) {
            lift.opControlPos();
            scorer.opControl();
            intake.opControlSensor();


            telemetry.addData("Runtime", runtime.toString());
            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        lift = new Lift(this, dashboard);
        scorer = new Scorer(this, lift);
        intake = new Intake(this);

        telemetry.update();
    }
}
