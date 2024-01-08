package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.BasicDrive;
import org.firstinspires.ftc.teamcode.modules.PullUp;
import org.firstinspires.ftc.teamcode.modules.Scorer;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Shooter;
import org.firstinspires.ftc.teamcode.modules.Lift;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {
    public BasicDrive basicDrive;
    public Lift lift;
    public Intake intake;
    public Scorer scorer;
    public PullUp pullUp;
    public Shooter shooter;
    private final ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();
        lift.timer.reset();
        pullUp.timer.reset();
        basicDrive.runtimeReset();


        while (opModeIsActive()) {
            basicDrive.driveFieldCentric();
            basicDrive.telemetry();
            // lift.opControlPos();
            intake.opControlSensor();
            scorer.opControl();
            pullUp.opControl();
            // shooter.tele();

            telemetry.addData("Runtime", runtime.toString());

            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        basicDrive = new BasicDrive(this, dashboard);
        lift = new Lift(this, dashboard);
        intake = new Intake(this);
        scorer = new Scorer(this);
        pullUp = new PullUp(this, dashboard);
        // shooter = new Shooter(this);


        telemetry.update();
    }
}
