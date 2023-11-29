package frc.robot.subsystems.swerve.orchestra;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveConstants;

import java.util.ArrayList;

public class SwerveOrchestra extends SubsystemBase {

	private static SwerveOrchestra instance = null;

	public static SwerveOrchestra getInstance() {
		if (instance == null) {
			instance = new SwerveOrchestra();
		}
		return instance;
	}

	Orchestra orchestra;

	ArrayList<TalonFX> controllers;

	/*
	 * An array of songs that are available to be played, can you guess the
	 * song/artists?
	 */
	String[] songs = new String[] {};

	/* track which song is selected for play */
	int songSelection = 0;

	/* overlapped actions */
	int timeToPlayLoops = 0;

	/* Boolean to tell if should play */
	boolean play = false;

	public SwerveOrchestra(ArrayList<TalonFX> controllers) {
		this.controllers = controllers;
		createInstruments();
		loadSelectedSong(0);
	}

	private SwerveOrchestra() {
		// Add controllers from swerve constants -- The singing motors
		controllers = new ArrayList<TalonFX>();
		controllers.add(SwerveConstants.FRONT_LEFT_DRIVE_MOTOR);
		controllers.add(SwerveConstants.FRONT_LEFT_ANGLE_MOTOR);
		controllers.add(SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR);
		controllers.add(SwerveConstants.FRONT_RIGHT_ANGLE_MOTOR);
		controllers.add(SwerveConstants.BACK_LEFT_DRIVE_MOTOR);
		controllers.add(SwerveConstants.BACK_LEFT_ANGLE_MOTOR);
		controllers.add(SwerveConstants.BACK_RIGHT_DRIVE_MOTOR);
		controllers.add(SwerveConstants.BACK_RIGHT_ANGLE_MOTOR);

		createInstruments();
	}

	public void shouldPlay(boolean play) {
		this.play = play;
	}

	public boolean isPlaying() {
		return orchestra.isPlaying();
	}

	private void createInstruments() {
		orchestra = new Orchestra(controllers);
	}

	public void loadSelectedSong(int offset) {
		/* increment song selection */
		songSelection += offset;
		songSelection %= songs.length;

		/* load the chirp file */
		orchestra.loadMusic(songs[songSelection]);

		/*
		 * schedule a play request, after a delay.
		 * This gives the Orchestra service time to parse chirp file.
		 * If play() is called immedietely after, you may get an invalid action error
		 * code.
		 */
		timeToPlayLoops = 10;
	}

	@Override
	public void periodic() {
		/* if song selection changed, auto-play it */
		if (timeToPlayLoops > 0) {
			--timeToPlayLoops;
			if (timeToPlayLoops == 0 && play) {
				/* scheduled play request */
				orchestra.play();
			}
		}
	}
}
