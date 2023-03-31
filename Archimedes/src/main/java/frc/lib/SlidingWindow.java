package frc.lib;

import java.security.InvalidParameterException;
import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Class representing a sliding window, allowing us to get the delta
 * 
 * @author Ty McKeon
 */
public class SlidingWindow {

    private final Deque<Number> deque;
    private final int maxLength;

    private Number delta;

    /**
     * Creates a window with a specified length and no delta
     * 
     * @param length
     */
    public SlidingWindow(int length) {
        deque = new ArrayDeque<>();
        this.maxLength = length;
    }

    /**
     * Creates a window with a preset length of 10 and no delta
     */
    public SlidingWindow() {
        this(10);
    }

    /**
     * Creates a window with a specified length and delta
     * 
     * @param length
     * @param delta
     */
    public SlidingWindow(int length, Number delta) {
        this(length);
        this.delta = delta;
    }

    /**
     * Add a number to the window. If max length is reached, removes the earliest
     * added value
     * 
     * @param value
     */
    public void add(Number value) {
        if (deque.size() >= maxLength) deque.removeFirst();
        deque.addLast(value);
    }


    /**
     * Check if the window is full
     * 
     * @return if window size is equal to max length
     */
    public boolean windowReady() {
        return deque.size() >= maxLength;
    }

    /**
     * Checks if the difference between the newest and oldest numbers in the window is less than the delta
     * 
     * @return if (first - last) < delta
     */
    public boolean checkDeltaLess() {
        return checkDelta(false);
    }

    /**
     * Checks if the difference between the newest and oldest numbers in the window is greater than the delta
     * 
     * @return if (first - last) > delta
     */
    public boolean checkDeltaGreater() {
        return checkDelta(true);
    }

    private boolean checkDelta(boolean isGreater) {
        if (delta == null) throw new InvalidParameterException("No delta sepcified. A delta must be sepcified on construction");

        double first = deque.getFirst().doubleValue();
        double last = deque.getLast().doubleValue();

        if (isGreater) return first - last > delta.doubleValue();
        return first - last < delta.doubleValue();
    }

    /**
     * Gets the delta between the newest and oldest numbers in the window
     * 
     * @return first - last
     */
    public Number getDelta() {
        double first = deque.getFirst().doubleValue();
        double last = deque.getLast().doubleValue(); 

        return (Number) (first - last);
    }

}
