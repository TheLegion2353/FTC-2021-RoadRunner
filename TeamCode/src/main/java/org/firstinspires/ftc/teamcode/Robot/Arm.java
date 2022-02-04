package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.index.qual.LTEqLengthOf;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm extends RobotPart {
	protected double kP = 1.2;
	protected double kI = 0.3;
	protected double kD = 0.05;
	protected double position = 0.0;
	protected static int positionDetent = 0;
	protected Telemetry telemetry = null;

	protected HardwareControllerEx motorController;
	protected PID pid;
	protected double beginBound, endBound;

	protected static boolean rightBumper, leftBumper;
	protected static boolean manualOverride;
	protected boolean kill = false;

	public Arm(Gamepad gp, DcMotorEx motor, Telemetry tel) {
		super(gp);
		telemetry = tel;
		position = 0.0;
		pid = new PID(kP, kI, kD, position);
		pid.setErrorOverTimeMax(10);
		motorController = new HardwareControllerEx(tel, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER, null, motor);
		motorController.setDirection(DcMotorSimple.Direction.REVERSE);
		beginBound = 0.32;
		endBound = 1.65;
		leftBumper = false;
		rightBumper = false;
		manualOverride = true;
	}

	@Override
	public void driverUpdate() {
		if (gamepad != null) {
			if (gamepad.right_bumper) {
				if (!rightBumper) {
					incrimentDetent();
					rightBumper = true;
				}
			} else {
				rightBumper = false;
			}

			if (gamepad.left_bumper) {
				if (!leftBumper) {
					decrementDetent();
					leftBumper = true;
				}
			} else {
				leftBumper = false;
			}

			if (gamepad.start) {
				manualOverride = false;
			}

			if (gamepad.back) {
				manualOverride = true;
			}

			updatePositions();
			pid.setSetPoint(position);
			double voltage = pid.PIDLoop(motorController.getVoltage());
			motorController.setSpeed(voltage);
			telemetry.addData("Arm Setpoint: ", position);
			telemetry.addData("Arm Position: ", motorController.getVoltage());
			telemetry.addData("Arm Voltage: ", voltage);
			telemetry.addData("Arm Detent Position: ", positionDetent);
		}
	}

	@Override
	protected void autonomousUpdate() {
		if (position > endBound) {
			position = endBound;
		}
		if (position < beginBound) {
			position = beginBound;
		}
		pid.setSetPoint(position);
		double voltage = pid.PIDLoop(motorController.getVoltage());
		if (!kill) {
			motorController.setSpeed(voltage);
		}
		telemetry.addData("Setpoint: ", position);
		telemetry.addData("Position: ", motorController.getVoltage());
	}

	protected void updatePositions() {
		if (manualOverride) {
			pid.updateConst(2.0, .75, .75);
			position += (gamepad.right_trigger - gamepad.left_trigger) * 1.5 * pid.getElapsedTime();
			if (position > endBound) {
				position = endBound;
			}
			if (position < beginBound) {
				position = beginBound;
			}
		} else {
			switch (positionDetent) {
				case 0: {  // back
					position = 1.55;
					pid.updateConst(1.2, 0.3, .05);
				}
				break;

				case 1: {  // down
					position = 0.32;
					pid.updateConst(4, 1.0, .1);
				}
				break;

				case 2: {  // lvl 1
					position = 0.46;
					pid.updateConst(4, 1.0, .1);
				}
				break;

				case 3: {  // lvl 2
					position = 0.61;
					pid.updateConst(4, 1.0, .1);
				}
				break;

				case 4: {  // lvl 3
					position = 0.89;
					pid.updateConst(4, 1.0, .1);
				}
				break;

				default: {
					positionDetent = 0;
				}
			}
		}
	}

	protected void incrimentDetent() {
		pid.resetErrorOverTime();
		positionDetent++;
		if (positionDetent > 4) {
			positionDetent = 0;
		}
	}

	protected void decrementDetent() {
		pid.resetErrorOverTime();
		positionDetent--;
		if (positionDetent < 0) {
			positionDetent = 4;
		}
	}

	public void kill() {
		kill = true;
	}

	public void setPosition(double p) {
		position = p;
	}

	public void setPotentiometer(AnalogInput pot) {
		motorController.addAnalogInput(pot);
		position = pot.getVoltage();
	}

	public void setBounds(double begin, double end) {
		beginBound = begin;
		endBound = end;
	}

	public PID getPID() {
		return pid;
	}
}