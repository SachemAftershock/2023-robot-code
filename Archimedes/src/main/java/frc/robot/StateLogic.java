package frc.robot;

import frc.robot.Constants.ButtonState;
import frc.robot.Constants.DriveLocationLUT;
import frc.robot.Constants.ElevatorStateLUT;

public class StateLogic {

    private ButtonState mConeOrCube;
    private ButtonState mHeightAndExtension;
    private ButtonState mIntakeOrEject;
    private DriveLocationLUT mDriveLocation;
    private ElevatorStateLUT mState;

    private static StateLogic mInstance;

    public StateLogic() {
        mConeOrCube = ButtonState.eNone;
        mHeightAndExtension = ButtonState.ePark;
        mIntakeOrEject = ButtonState.eNone;
        mDriveLocation = DriveLocationLUT.eNone;
        mState = ElevatorStateLUT.ePark;
    }

    public void setConeOrCube(ButtonState coneOrCube) {
        mConeOrCube = coneOrCube;
    }

    public void setHeightAndExtension(ButtonState heightAndExtension) {
        mHeightAndExtension = heightAndExtension;
    }

    public void setIntakeOrEject(ButtonState intakeOrEject) {
        mIntakeOrEject = intakeOrEject;
    }

    public void setDriveLocation(DriveLocationLUT driveLocation) {
        mDriveLocation = driveLocation;
    }

    public void periodic() {

        if(mConeOrCube == ButtonState.eCone) {
            if(mHeightAndExtension == ButtonState.eFloor)
                mState = ElevatorStateLUT.eFloorCone;
            if(mHeightAndExtension == ButtonState.eHumanStation) 
                mState = ElevatorStateLUT.eHumanStationCone;
            if(mHeightAndExtension == ButtonState.eMid) 
                mState = ElevatorStateLUT.eMidCone;
            if(mHeightAndExtension == ButtonState.eHigh) 
                mState = ElevatorStateLUT.eHighCone;
        }

        if(mConeOrCube == ButtonState.eCube) {
            if(mHeightAndExtension == ButtonState.eFloor)
                mState = ElevatorStateLUT.eFloorCube;
            if(mHeightAndExtension == ButtonState.eHumanStation) 
                mState = ElevatorStateLUT.eHumanStationCube;
            if(mHeightAndExtension == ButtonState.eMid) 
                mState = ElevatorStateLUT.eMidCube;
            if(mHeightAndExtension == ButtonState.eHigh) 
                mState = ElevatorStateLUT.eHighCube;
        }

    }

    /**
     * 
     * @return returns the state of the elevator and arm as an enum
     * 
     */
    public ElevatorStateLUT getState() {
        return mState;
    }

    /**
     * 
     * @return returns a robot state object
     * 
     */
    public RobotState getRobotState() {
        return new RobotState(mState, mDriveLocation);

    }

    public synchronized static StateLogic getInstance() {
		if (mInstance == null) {
			mInstance = new StateLogic();
		}
		return mInstance;
	}

    public class RobotState {

        ElevatorStateLUT mState;
        DriveLocationLUT mLocation;

        public RobotState(ElevatorStateLUT state, DriveLocationLUT location) {
            mState = state;
            mLocation = location;
        }

        public ElevatorStateLUT getElevatorState() {
            return mState;
        }

        public DriveLocationLUT getDriveLocation() {
            return mLocation;
        }

    }
    
}
