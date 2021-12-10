package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

public class Robot {
	public enum AutonomousPath {
		BLUE_CLOSE_CAROUSEL_PARK_1_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_PARK_2_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY,
		BLUE_FAR_1_PARK,
		BLUE_FAR_2_PARK,
		RED_CLOSE_CAROUSEL_PARK_TRAJECTORY,
		RED_FAR_PARK
	}
	private ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	private MecanumDrivetrain drivetrain;
	private Carousel carousel;
	private Arm arm;
	private LinearSlide slide;
	private Intake intake;
	private Gamepad gamepad = null;
	private Telemetry telemetry = null;
	private TrajectorySequence BLUE_CLOSE_CAROUSEL_PARK_1_TRAJECTORY = null;
	private TrajectorySequence BLUE_CLOSE_CAROUSEL_PARK_2_TRAJECTORY = null;
	private TrajectorySequence BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY = null;
	private TrajectorySequence BLUE_CLOSE_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY = null;
	private TrajectorySequence BLUE_CLOSE_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY = null;
	private TrajectorySequence BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY = null;
	private TrajectorySequence BLUE_CLOSE_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY = null;
	private TrajectorySequence BLUE_CLOSE_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY = null;
	private TrajectorySequence BLUE_FAR_1_PARK = null;
	private TrajectorySequence BLUE_FAR_2_PARK = null;
	private TrajectorySequence RED_CLOSE_CAROUSEL_PARK_TRAJECTORY = null;
	private TrajectorySequence RED_FAR_PARK = null;

	public Robot(Gamepad gp, Telemetry t, HardwareMap hwMap) {
		telemetry = t;
		gamepad = gp;
		drivetrain = new MecanumDrivetrain(gamepad, telemetry, hwMap);
		BLUE_CLOSE_CAROUSEL_PARK_1_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.lineToConstantHeading(new Vector2d(-62.5, 55.0))
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
					carousel.setSpeed(1.0);
				})
				.waitSeconds(3.5)
				.addTemporalMarker(() -> {
					carousel.setSpeed(0.0);
				})
				.lineToSplineHeading(new Pose2d(7.5, 63.5, 0.0))
				.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
				.lineToConstantHeading(new Vector2d(37, 64.0))
				.resetVelConstraint()
				.build();

		BLUE_CLOSE_CAROUSEL_PARK_2_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.lineToConstantHeading(new Vector2d(-62.5, 55.0))
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
					carousel.setSpeed(1.0);
				})
				.waitSeconds(3.5)
				.addTemporalMarker(() -> {
					carousel.setSpeed(0.0);
				})
				.lineToSplineHeading(new Pose2d(7.5, 63.5, 0.0))
				.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
				.lineToConstantHeading(new Vector2d(37, 64.0))
				.resetVelConstraint()
				.lineToConstantHeading(new Vector2d(37, 40.0))
				.build();

		BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.lineToConstantHeading(new Vector2d(-14.0, 40.0))  // go in front of the shipping hub
				.addTemporalMarker(() -> {
					arm.setPosition(1.715);
				})
				.waitSeconds(1.5)
				.addTemporalMarker(() -> {
					intake.setSpeed(-0.25);
				})
				.waitSeconds(1.0)
				.addTemporalMarker(() -> {
					intake.setSpeed(0.0);
					arm.setPosition(1.35);
				})
				.lineToConstantHeading(new Vector2d(-62.5, 55.0))
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {  // go to the carousel thing
					carousel.setSpeed(1.0);
				})
				.waitSeconds(3.5)
				.addTemporalMarker(() -> {
					carousel.setSpeed(0.0);
				})
				.lineToSplineHeading(new Pose2d(7.5, 63.5, 0.0))  // set up for going to the parking
				.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
 				.lineToConstantHeading(new Vector2d(37, 64.0))  // go through the obstacle
				.resetVelConstraint()
				.build();

		BLUE_CLOSE_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.lineToConstantHeading(new Vector2d())  // go in front of the shipping hub
				.addTemporalMarker(() -> {
					arm.setPosition(1.715);
				})
				.waitSeconds(1.5)
				.addTemporalMarker(() -> {
					slide.setPosition(-300.0);
				})
				.waitSeconds(2.0)
				.addTemporalMarker(() -> {
					intake.setSpeed(-0.25);
				})
				.waitSeconds(1.0)
				.addTemporalMarker(() -> {
					intake.setSpeed(0.0);
					arm.setPosition(1.35);
					slide.setPosition(0.0);
				})
				.lineToConstantHeading(new Vector2d(-62.5, 55.0))
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {  // go to the carousel thing
					carousel.setSpeed(1.0);
				})
				.waitSeconds(3.5)
				.addTemporalMarker(() -> {
					carousel.setSpeed(0.0);
				})
				.lineToSplineHeading(new Pose2d(7.5, 63.5, 0.0))  // set up for going to the parking
				.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
				.lineToConstantHeading(new Vector2d(37, 64.0))  // go through the obstacle
				.resetVelConstraint()
				.build();

		BLUE_CLOSE_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.lineToConstantHeading(new Vector2d())  // go in front of the shipping hub
				.addTemporalMarker(() -> {
					arm.setPosition(1.715);
				})
				.waitSeconds(1.5)
				.addTemporalMarker(() -> {
					slide.setPosition(-1600.0);
				})
				.waitSeconds(4.0)
				.addTemporalMarker(() -> {
					intake.setSpeed(-0.25);
				})
				.waitSeconds(1.0)
				.addTemporalMarker(() -> {
					intake.setSpeed(0.0);
					arm.setPosition(1.35);
					slide.setPosition(0.0);
				})
				.lineToConstantHeading(new Vector2d(-62.5, 55.0))
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {  // go to the carousel thing
					carousel.setSpeed(1.0);
				})
				.waitSeconds(3.5)
				.addTemporalMarker(() -> {
					carousel.setSpeed(0.0);
				})
				.lineToSplineHeading(new Pose2d(7.5, 63.5, 0.0))  // set up for going to the parking
				.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
				.lineToConstantHeading(new Vector2d(37, 64.0))  // go through the obstacle
				.resetVelConstraint()
				.build();

/*
		BLUE_FAR_PARK = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
			.build();

		RED_CLOSE_CAROUSEL_PARK_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
			.build();

		RED_FAR_PARK = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
			.build();
*/

	}

	public void runAuto(AutonomousPath path) throws InterruptedException {
		switch (path) {
			case BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().followTrajectorySequence(BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY);
			}
		}
	}

	public void autoUpdate() {
		if (arm != null) {
			arm.update();
		} else {
			telemetry.addLine("Intake null!");
		}
		telemetry.update();
		clock.reset();
	}

	public void update() {
		if (drivetrain != null) {
			drivetrain.update();
		} else {
			telemetry.addLine("Drivetrain null!");
		}

		if (carousel != null) {
			carousel.update();
		} else {
			telemetry.addLine("Carousel null!");
		}

		if (arm != null) {
			arm.update();
		} else {
			telemetry.addLine("Arm null!");
		}

		if (slide != null) {
			slide.update();
		} else {
			telemetry.addLine("Linear slide null!");
		}

		if (intake != null) {
			intake.update();
		} else {
			telemetry.addLine("Intake null!");
		}

		telemetry.update();
		clock.reset();
	}

	public void setCarouselMotor(DcMotorEx motor) {
		carousel = new Carousel(gamepad, motor, telemetry);
	}

	public void setArm(DcMotorEx motor, AnalogInput pot) {
		arm = new Arm(gamepad, motor, telemetry);
		arm.setPotentiometer(pot);
	}

	public void setLinearSlide(DcMotorEx motor) {
		slide = new LinearSlide(gamepad, motor, telemetry);
	}

	public void setIntake(DcMotorEx motor) {
		intake = new Intake(gamepad, motor, telemetry);
	}
}