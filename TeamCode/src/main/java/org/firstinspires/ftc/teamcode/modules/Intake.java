package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


// TODO: Переделать
public class Intake {
    private final LinearOpMode linearOpMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final DcMotor motor;

    public Intake(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        //gamepad = linearOpMode.gamepad2;
        gamepad = linearOpMode.gamepad2;

        motor = hardwareMap.get(DcMotor.class, "motor_intake");
        motor.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Intake: ", "Initialized");
    }

    public void tele() {
        if (gamepad.x) {
            motor.setPower(1);
        } else {
            motor.setPower(0);
        }
        /*double axial = -gamepad.right_stick_y;
        motor.setPower(axial*0.5);*/
    }

    public void testing() {
        telemetry.addData("Intake_motor", motor.getCurrentPosition());
    }
}
