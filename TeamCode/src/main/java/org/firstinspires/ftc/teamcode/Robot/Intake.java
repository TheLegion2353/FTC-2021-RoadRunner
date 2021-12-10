package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends RobotPart {

	private HardwareControllerEx motorController;

	public Intake(Gamepad gp, DcMotorEx motor, Telemetry tel) {
		super(gp);
		motorController = new HardwareControllerEx(tel, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, null, motor);
	}

	@Override
	public void driverUpdate() {
		if (gamepad.x) {
			motorController.setSpeed(1.0);
		} else if (gamepad.y) {
			motorController.setSpeed(-.25);
		} else {
			motorController.setSpeed(0.0);
		}
	}

	void setSpeed(double s) {
		motorController.setSpeed(s);
	}
}