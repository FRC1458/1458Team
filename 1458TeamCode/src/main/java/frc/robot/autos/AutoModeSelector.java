package frc.robot.autos;

import frc.robot.autos.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class AutoModeSelector {
	public enum DesiredMode {
		DO_NOTHING
	}

	public enum TargetNote {
		N1,
		N2,
		N3,
		N4,
		N5
	}

	public enum TargetSpike {
		LEFT,
		CENTER,
		RIGHT
	}

	private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;
	private TargetNote mCachedFirstNote = TargetNote.N1;
	private TargetNote mCachedSecondNote = TargetNote.N2;
	private TargetSpike mCachedSpike = TargetSpike.LEFT;

	private Optional<AutoModeBase> mAutoMode = Optional.empty();

	private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();
	private static SendableChooser<TargetNote> mFirstNoteChooser = new SendableChooser<>();
	private static SendableChooser<TargetNote> mSecondNoteChooser = new SendableChooser<>();
	private static SendableChooser<TargetSpike> mSpikeChooser = new SendableChooser<>();

	public AutoModeSelector() {
		mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
		mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
	}

	public void updateModeCreator(boolean force_regen) {
		DesiredMode desiredMode = mModeChooser.getSelected();
		TargetNote firstNote = mFirstNoteChooser.getSelected();
		TargetNote secondNote = mSecondNoteChooser.getSelected();
		TargetSpike desiredSpike = mSpikeChooser.getSelected();

		if (desiredMode == null) {
			desiredMode = DesiredMode.DO_NOTHING;
		}
		if (mCachedDesiredMode != desiredMode
				|| mCachedFirstNote != firstNote
				|| mCachedSecondNote != secondNote
				|| mCachedSpike != desiredSpike
				|| force_regen) {
			System.out.println("Auto selection changed, updating creator: desiredMode-> " + desiredMode.name() + "//"
					+ firstNote.name() + "//" + secondNote.name() + "//" + desiredMode.name() + " Spike");
			mAutoMode = getAutoModeForParams(desiredMode, firstNote, secondNote, desiredSpike);
		}
		mCachedDesiredMode = desiredMode;
		mCachedFirstNote = firstNote;
		mCachedSecondNote = secondNote;
		mCachedSpike = desiredSpike;
	}

	private Optional<AutoModeBase> getAutoModeForParams(
			DesiredMode mode, TargetNote n_0, TargetNote n_1, TargetSpike s_0) {
		switch (mode) {
			case DO_NOTHING:
				return Optional.of(new DoNothingMode());

			default:
				System.out.println("ERROR: unexpected auto mode: " + mode);
				break;
		}

		System.err.println("No valid auto mode found for  " + mode);
		return Optional.empty();
	}

	public static SendableChooser<DesiredMode> getModeChooser() {
		return mModeChooser;
	}

	public DesiredMode getDesiredAutomode() {
		return mCachedDesiredMode;
	}

	public void reset() {
		mAutoMode = Optional.empty();
		mCachedDesiredMode = null;
	}

	public void outputToSmartDashboard() {
		SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
	}

	public Optional<AutoModeBase> getAutoMode() {
		if (!mAutoMode.isPresent()) {
			return Optional.empty();
		}
		return mAutoMode;
	}

	public boolean isDriveByCamera() {
		return mCachedDesiredMode == DesiredMode.DO_NOTHING;
	}
}
