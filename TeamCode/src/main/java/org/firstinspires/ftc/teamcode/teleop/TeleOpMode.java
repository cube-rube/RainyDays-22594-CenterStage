package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.BasicDrive;
import org.firstinspires.ftc.teamcode.modules.PullUp;
import org.firstinspires.ftc.teamcode.modules.Deploy;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Launch;
import org.firstinspires.ftc.teamcode.modules.Lift;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {
    private BasicDrive basicDrive;
    private Lift lift;
    private Intake intake;
    private Deploy deploy;
    private PullUp pullUp;
    private Launch launch;
    private ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();
         lift.runtimeReset();
        basicDrive.runtimeReset();


        while (opModeIsActive()) {
             basicDrive.testing();
            //basicDrive.forwardWithIMU();
            //basicDrive.driveRobotCentricEncoder();
            //basicDrive.driveFieldCentric();
            basicDrive.driveFieldCentricEncoder();
            //basicDrive.tele();
            intake.tele();
             lift.telePID();
            //deploy.easyTele();
            pullUp.tele();
             //launch.tele();

            telemetry.addData("Runtime", runtime.toString());

            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        basicDrive = new BasicDrive(this, dashboard);
        lift = new Lift(this, dashboard);
        intake = new Intake(this);
         //deploy = new Deploy(this);
        pullUp = new PullUp(this);
         //launch = new Launch(this);


        telemetry.update();
    }
}
