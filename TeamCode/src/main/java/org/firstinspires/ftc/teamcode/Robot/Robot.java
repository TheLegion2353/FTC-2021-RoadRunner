package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
		BLUE_CLOSE_PARK_1_TRAJECTORY,
		BLUE_CLOSE_PARK_2_TRAJECTORY,
		BLUE_FAR_PARK_1_TRAJECTORY,
		BLUE_FAR_PARK_2_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY,
		BLUE_CLOSE_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY,
		BLUE_FAR_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY,
		BLUE_FAR_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY,
		BLUE_FAR_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY,
		BLUE_FAR_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY,
		BLUE_FAR_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY,
		BLUE_FAR_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY,

		RED_CLOSE_PARK_1_TRAJECTORY,
		RED_CLOSE_PARK_2_TRAJECTORY,
		RED_FAR_PARK_1_TRAJECTORY,
		RED_FAR_PARK_2_TRAJECTORY,
		RED_CLOSE_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY,
		RED_CLOSE_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY,
		RED_CLOSE_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY,
		RED_CLOSE_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY,
		RED_CLOSE_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY,
		RED_CLOSE_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY,
		RED_FAR_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY,
		RED_FAR_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY,
		RED_FAR_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY,
		RED_FAR_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY,
		RED_FAR_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY,
		RED_FAR_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY,
		TEST_TRAJECTORY
	}

	private ElapsedTime clock = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
	private MecanumDrivetrain drivetrain = null;
	private Carousel carousel;
	private Arm arm;
	private LinearSlide slide;
	private Intake intake;
	private Gamepad gamepad = null;
	private Telemetry telemetry = null;

	private TrajectorySequence BLUE_PARK_INNER = null;
	private TrajectorySequence BLUE_CAROUSEL_LEVEL_1_TRAJECTORY = null;
	private TrajectorySequence BLUE_CAROUSEL_LEVEL_2_TRAJECTORY = null;
	private TrajectorySequence BLUE_CAROUSEL_LEVEL_3_TRAJECTORY = null;
	private TrajectorySequence BLUE_PARK_TRAJECTORY = null;

	private TrajectorySequence RED_PARK_INNER = null;
	private TrajectorySequence RED_CAROUSEL_LEVEL_1_TRAJECTORY = null;
	private TrajectorySequence RED_CAROUSEL_LEVEL_2_TRAJECTORY = null;
	private TrajectorySequence RED_CAROUSEL_LEVEL_3_TRAJECTORY = null;
	private TrajectorySequence RED_PARK_TRAJECTORY = null;

	private TrajectorySequence FINAL_TRAJECTORY = null;

	public Robot(Gamepad gp, Telemetry t, HardwareMap hwMap) {
		telemetry = t;
		gamepad = gp;
		drivetrain = new MecanumDrivetrain(gamepad, telemetry, hwMap);
	}

	public Robot(Gamepad gp, Telemetry t, HardwareMap hwMap, AutonomousPath path) {
		telemetry = t;
		gamepad = gp;
		drivetrain = new MecanumDrivetrain(gamepad, telemetry, hwMap);

		constructPaths(path);

		/*
		BLUE_CAROUSEL_LEVEL_1_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.lineToConstantHeading(new Vector2d(-14.0, 40.0))  // go in front of the shipping hub
				.addTemporalMarker(() -> {
					arm.setPosition(1.715);
				})
				.waitSeconds(1.5)
				.addTemporalMarker(() -> {
					intake.setSpeed(-0.4);
				})
				.waitSeconds(1.0)
				.addTemporalMarker(() -> {
					intake.setSpeed(0.0);
					arm.setPosition(1.35);
				})
				.lineToConstantHeading(new Vector2d(-62.5, 56.5))  // go to the carousel thing
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
					carousel.setSpeed(1.0);
				})
				.waitSeconds(3.5)
				.addTemporalMarker(() -> {
					carousel.setSpeed(0.0);
				})
				.lineToSplineHeading(new Pose2d(7.5, 64, 0.0))  // set up for going to the parking
				.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
				.lineToConstantHeading(new Vector2d(37, 64.5))  // go through the obstacle
				.resetVelConstraint()
				.build();



		BLUE_CAROUSEL_LEVEL_2_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.lineToConstantHeading(new Vector2d())  // go in front of the shipping hub
				.addTemporalMarker(() -> {
					arm.setPosition(1.715);
					slide.setPosition(-300.0);
				})
				.waitSeconds(1.5)
				.addTemporalMarker(() -> {
					intake.setSpeed(-0.4);
				})
				.waitSeconds(1.0)
				.addTemporalMarker(() -> {
					intake.setSpeed(0.0);
					slide.setPosition(0.0);
				})
				.lineToConstantHeading(new Vector2d(-60.0, 55.0))
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {  // go to the carousel thing
					carousel.setSpeed(1.0);
					arm.setPosition(1.35);
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

		BLUE_CAROUSEL_LEVEL_3_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.lineToConstantHeading(new Vector2d())  // go in front of the shipping hub
				.addTemporalMarker(() -> {
					arm.setPosition(1.715);
					slide.setPosition(-1600.0);

				})
				.waitSeconds(4.0)
				.addTemporalMarker(() -> {
					intake.setSpeed(-0.4);

				})
				.waitSeconds(1.0)
				.addTemporalMarker(() -> {
					intake.setSpeed(0.0);
					slide.setPosition(-300.0);
				})
				.lineToConstantHeading(new Vector2d(-62.5, 55.0))
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {  // go to the carousel thing
					carousel.setSpeed(1.0);
					arm.setPosition(1.35);

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

		BLUE_PARK_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
				.lineToConstantHeading(new Vector2d(39.0, 62.0))
				.resetVelConstraint()
				.build();

		RED_CAROUSEL_LEVEL_1_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.lineToConstantHeading(new Vector2d(-14.0, -40.0))  // go in front of the shipping hub
				.addTemporalMarker(() -> {
					arm.setPosition(1.715);
				})
				.waitSeconds(1.5)
				.addTemporalMarker(() -> {
					intake.setSpeed(-0.4);
				})
				.waitSeconds(1.0)
				.addTemporalMarker(() -> {
					intake.setSpeed(0.0);
					arm.setPosition(1.35);
				})
				.lineToConstantHeading(new Vector2d(-62.5, -56.5))  // go to the carousel thing
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
					carousel.setSpeed(1.0);
				})
				.waitSeconds(3.5)
				.addTemporalMarker(() -> {
					carousel.setSpeed(0.0);
				})
				.lineToSplineHeading(new Pose2d(7.5, -64, 0.0))  // set up for going to the parking
				.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
				.lineToConstantHeading(new Vector2d(37, -64.5))  // go through the obstacle
				.resetVelConstraint()
				.build();

		RED_CAROUSEL_LEVEL_2_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.lineToConstantHeading(new Vector2d())  // go in front of the shipping hub
				.addTemporalMarker(() -> {
					arm.setPosition(1.715);
					slide.setPosition(-300.0);
				})
				.waitSeconds(1.5)
				.addTemporalMarker(() -> {
					intake.setSpeed(-0.4);
				})
				.waitSeconds(1.0)
				.addTemporalMarker(() -> {
					intake.setSpeed(0.0);
					slide.setPosition(0.0);
				})
				.lineToConstantHeading(new Vector2d(-60.0, 55.0))
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {  // go to the carousel thing
					carousel.setSpeed(1.0);
					arm.setPosition(1.35);
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

		RED_CAROUSEL_LEVEL_3_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.lineToConstantHeading(new Vector2d())  // go in front of the shipping hub
				.addTemporalMarker(() -> {
					arm.setPosition(1.715);
					slide.setPosition(-1600.0);

				})
				.waitSeconds(4.0)
				.addTemporalMarker(() -> {
					intake.setSpeed(-0.4);
				})
				.waitSeconds(1.0)
				.addTemporalMarker(() -> {
					intake.setSpeed(0.0);
					slide.setPosition(-300.0);
				})
				.lineToConstantHeading(new Vector2d(-62.5, 55.0))
				.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {  // go to the carousel thing
					carousel.setSpeed(1.0);
					arm.setPosition(1.35);

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

		RED_PARK_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
				.lineToConstantHeading(new Vector2d(39.0, -62.0))
				.resetVelConstraint()
				.build();

		BLUE_PARK_INNER = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.strafeRight(23.0)
				.build();

		RED_PARK_INNER = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
				.strafeLeft(23.0)
				.build();
		 */
	}

	public void constructPaths(AutonomousPath path) {
		switch (path) {
			// BLUE LEVEL 1
			case BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY: {  // test this first then basically copy for rest of levels
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, 40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, 56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, 64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, 64.5))  // go through the obstacle
						.resetVelConstraint()
						.build();
			} break;

			case BLUE_FAR_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY: {  // test this first then basically copy for rest of levels
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, -Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, 40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, 56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, 64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, 64.5))  // go through the obstacle
						.resetVelConstraint()
						.build();
			} break;

			case BLUE_CLOSE_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, 40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, 56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, 64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, 64.5))  // go through the obstacle
						.resetVelConstraint()
						.strafeRight(23.0)
						.build();
			} break;

			case BLUE_FAR_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, -Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, 40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, 56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, 64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, 64.5))  // go through the obstacle
						.resetVelConstraint()
						.strafeRight(23.0)
						.build();
			} break;

			// BLUE LEVEL 2
			case BLUE_CLOSE_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
			} break;

			case BLUE_FAR_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, -Math.PI / 2.0));
			} break;

			case BLUE_CLOSE_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
			} break;

			case BLUE_FAR_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, -Math.PI / 2.0));
			} break;

			// BLUE LEVEL 3
			case BLUE_CLOSE_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
			} break;

			case BLUE_FAR_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, -Math.PI / 2.0));
			} break;

			case BLUE_CLOSE_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, -Math.PI / 2.0));
			} break;

			case BLUE_FAR_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, -Math.PI / 2.0));
			} break;

			// BLUE PARK ONLY
			case BLUE_CLOSE_PARK_1_TRAJECTORY: {  // deprecated
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, 62.0))
						.resetVelConstraint()
						.build();
			} break;

			case BLUE_FAR_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, 62.0))
						.resetVelConstraint()
						.build();
			} break;

			case BLUE_CLOSE_PARK_2_TRAJECTORY: {  // deprecated
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, 62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, 62.0))
						.resetVelConstraint()
						.strafeRight(23.0)
						.build();
			} break;

			case BLUE_FAR_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, 62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, 62.0))
						.resetVelConstraint()
						.strafeRight(23.0)
						.build();
			} break;


			// RED LEVEL 1
			case RED_CLOSE_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY: {  // test this first then basically copy for rest of levels
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, -40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, -56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, -64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, -64.5))  // go through the obstacle
						.resetVelConstraint()
						.build();
			} break;

			case RED_FAR_CAROUSEL_LEVEL_1_PARK_1_TRAJECTORY: {  // test this first then basically copy for rest of levels
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, -62.0, Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, -40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, -56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, -64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, -64.5))  // go through the obstacle
						.resetVelConstraint()
						.build();
			} break;

			case RED_CLOSE_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, -40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, -56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, -64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, -64.5))  // go through the obstacle
						.resetVelConstraint()
						.strafeLeft(23.0)
						.build();
			} break;

			case RED_FAR_CAROUSEL_LEVEL_1_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, -62.0, Math.PI / 2.0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.lineToConstantHeading(new Vector2d(-14.0, -40.0))  // go in front of the shipping hub
						.addTemporalMarker(() -> {
							arm.setPosition(1.715);
						})
						.waitSeconds(1.5)
						.addTemporalMarker(() -> {
							intake.setSpeed(-0.4);
						})
						.waitSeconds(1.0)
						.addTemporalMarker(() -> {
							intake.setSpeed(0.0);
							arm.setPosition(1.35);
						})
						.lineToConstantHeading(new Vector2d(-62.5, -56.5))  // go to the carousel thing
						.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
							carousel.setSpeed(1.0);
						})
						.waitSeconds(3.5)
						.addTemporalMarker(() -> {
							carousel.setSpeed(0.0);
						})
						.lineToSplineHeading(new Pose2d(7.5, -64, 0.0))  // set up for going to the parking
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(37, -64.5))  // go through the obstacle
						.resetVelConstraint()
						.strafeLeft(23.0)
						.build();
			} break;

			// RED LEVEL 2
			case RED_CLOSE_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
			} break;

			case RED_FAR_CAROUSEL_LEVEL_2_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0 + 46.0, -62.0, Math.PI / 2.0));
			} break;

			case RED_CLOSE_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
			} break;

			case RED_FAR_CAROUSEL_LEVEL_2_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0 + 46.0, -62.0, Math.PI / 2.0));
			} break;

			// RED LEVEL 3
			case RED_CLOSE_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
			} break;

			case RED_FAR_CAROUSEL_LEVEL_3_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0 + 46.0, -62.0, Math.PI / 2.0));
			} break;

			case RED_CLOSE_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, Math.PI / 2.0));
			} break;

			case RED_FAR_CAROUSEL_LEVEL_3_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0 + 46.0, -62.0, Math.PI / 2.0));
			} break;

			// RED PARK ONLY
			case RED_CLOSE_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, -62.0))
						.resetVelConstraint()
						.build();
			} break;

			case RED_FAR_PARK_1_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, -62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, -62.0))
						.resetVelConstraint()
						.build();
			} break;

			case RED_CLOSE_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(-36.0, -62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, -62.0))
						.resetVelConstraint()
						.strafeLeft(23.0)
						.build();
			} break;

			case RED_FAR_PARK_2_TRAJECTORY: {
				drivetrain.getDrivetrain().setPoseEstimate(new Pose2d(10.0, -62.0, 0));
				FINAL_TRAJECTORY = drivetrain.getDrivetrain().trajectorySequenceBuilder(drivetrain.getDrivetrain().getPoseEstimate())
						.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
						.lineToConstantHeading(new Vector2d(39.0, -62.0))
						.resetVelConstraint()
						.strafeLeft(23.0)
						.build();
			} break;
		}
	}

	public void runAuto(OpMode opMode) throws InterruptedException {
		drivetrain.getDrivetrain().followTrajectorySequence(FINAL_TRAJECTORY);
		opMode.requestOpModeStop();
	}

	public void runAuto(AutonomousPath path, OpMode opMode) throws InterruptedException {
		drivetrain.getDrivetrain().followTrajectorySequence(FINAL_TRAJECTORY);
		opMode.requestOpModeStop();
	}

	public void autoUpdate() {
		if (arm != null) {
			arm.update();
		} else {
			telemetry.addLine("Intake null!");
		}

		if (slide != null) {
			slide.update();
		} else {
			telemetry.addLine("Slide null!");
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