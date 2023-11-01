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

public class Intake {
    private LinearOpMode linearOpMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private CRServo servo;
    private DcMotor motor;

    public Intake(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;

        servo = hardwareMap.get(CRServo.class, "intake");
        servo.setDirection(CRServo.Direction.FORWARD);
        motor = hardwareMap.get(DcMotor.class, "motor_intake");
        motor.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Intake: ", "Initialized");
    }

    public void tele() {
        if (gamepad.x) {
            servo.setPower(1);
            motor.setPower(1);
        }
        /* else if (gamepad.y) {
            servo.setPower(-1);
            motor.setPower(-1);
        } */
        else {
            servo.setPower(0);
            motor.setPower(0);
        }
    }

    public void testing() {
        telemetry.addData("Intake_motor", motor.getCurrentPosition());
    }
}
