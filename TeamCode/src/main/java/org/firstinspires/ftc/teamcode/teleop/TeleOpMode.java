package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.BasicDrive;
import org.firstinspires.ftc.teamcode.drive.BasicDriveNoEncoders;
import org.firstinspires.ftc.teamcode.modules.Claw;
import org.firstinspires.ftc.teamcode.modules.Deploy;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Launch;
import org.firstinspires.ftc.teamcode.modules.Lift;

@TeleOp(name = "TeleOpMode")
public class TeleOpMode extends LinearOpMode {
    private BasicDriveNoEncoders basicDrive;
    private Lift lift;
    private Intake intake;
    private Deploy deploy;
    private Claw claw;
    private Launch launch;
    private ElapsedTime runtime = new ElapsedTime();
    public FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        runtime.reset();
        waitForStart();
        lift.runtimeReset();


        while (opModeIsActive()) {
            basicDrive.tele();
            intake.tele();
            lift.telePID();
            deploy.easyTele();
            claw.tele();

            telemetry.addData("Runtime", runtime.toString());

            telemetry.update();
        }
    }

    private void initRobot() {
        dashboard = FtcDashboard.getInstance();

        basicDrive = new BasicDriveNoEncoders(this);
        lift = new Lift(this, dashboard);
        intake = new Intake(this);
        deploy = new Deploy(this);
        claw = new Claw(this);
        launch = new Launch(this);

        telemetry.update();
    }
}
