package frc.robot;

import static frc.robot.Constants.ButtonBoxConstants.kErrorDelaySeconds;

public class ErrorTracker {

    private final boolean[] mErrors = new boolean[5];
    private int mErrorIndex = 0;
    private long mErrorTime = 0;

    private static ErrorTracker mInstance;

    private ErrorTracker() {
        for (int i = 0; i < mErrors.length; i++) {
            mErrors[i] = false;
        }
    }

    public enum ErrorType {
        eNavXZero, eElevatorLidarInfinity, eIntakeLidarInfinity, eArmLidarInfinity, eElevatorMoveFailure;

        public int get() {
            return ordinal();
        }
    }

    public void reportErrors() {
        if (mErrorIndex >= 4) mErrorIndex = 0;

        // for (int i = 0; i < mErrors.length; i++) {
        // System.out.print(mErrors[i] + " ");
        // }
        // System.out.println();

        if (!isErrorPresent(mErrors)) {
            ButtonBoxPublisher.clearError();
            System.out.println("No errors");
            return;
        }

        if (System.currentTimeMillis() - mErrorTime < kErrorDelaySeconds * 1000) {
            System.out.println("waiting");
            return;
        }

        for (int i = mErrorIndex; i < mErrors.length; i++) {
            System.out.println(i + " " + mErrors[i]);
            if (mErrors[i]) {
                ButtonBoxPublisher.sendError(ErrorType.values()[i]);
                System.out.println("Sending error " + ErrorType.values()[i]);
                mErrorIndex = i + 1;
                mErrorTime = System.currentTimeMillis();
                break;
            }
        }
    }

    /**
     * Enable or disable an error to be sent to the button box lcd.
     * 
     * @param errorType the error type to set
     * @param error     true to enable the error, false to disable
     */
    public void setError(ErrorType errorType, boolean error) {
        mErrors[errorType.get()] = error;
    }

    public void enableError(ErrorType errorType) {
        setError(errorType, true);
    }

    public void disableError(ErrorType errorType) {
        setError(errorType, false);
    }

    public boolean isErrorEnabled(ErrorType errorType) {
        return mErrors[errorType.get()];
    }

    private boolean isErrorPresent(boolean[] arr) {
        for (boolean b : arr) if (b) return true;
        return false;
    }

    public static ErrorTracker getInstance() {
        if (mInstance == null) mInstance = new ErrorTracker();
        return mInstance;
    }

}
