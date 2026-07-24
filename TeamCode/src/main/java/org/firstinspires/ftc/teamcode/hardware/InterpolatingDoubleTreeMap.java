package org.firstinspires.ftc.teamcode.hardware; // Update to match your project's package structure

import java.util.Map;
import java.util.TreeMap;

/**
 * FTC-compatible implementation of WPILib's InterpolatingDoubleTreeMap.
 *
 * Maps double keys to double values and performs linear interpolation for keys
 * that fall between defined data points. Clamps to boundary values when queried
 * outside the map's bounds.
 */
public class InterpolatingDoubleTreeMap {

    private final TreeMap<Double, Double> map = new TreeMap<>();

    /**
     * Constructs an empty InterpolatingDoubleTreeMap.
     */
    public InterpolatingDoubleTreeMap() {}

    /**
     * Inserts a key-value pair into the map.
     *
     * @param key   The key (e.g., target distance in inches or meters)
     * @param value The value (e.g., motor velocity or arm angle)
     */
    public void put(double key, double value) {
        map.put(key, value);
    }

    /**
     * Gets the interpolated value associated with the given key.
     *
     * @param key The key to look up.
     * @return The exact value if key exists, the linearly interpolated value if
     *         between two keys, the clamped boundary value if outside range,
     *         or {@code Double.NaN} if the map is empty.
     */
    public double get(double key) {
        if (map.isEmpty()) {
            return Double.NaN;
        }

        // Return exact match if present
        Double exactMatch = map.get(key);
        if (exactMatch != null) {
            return exactMatch;
        }

        Map.Entry<Double, Double> floor = map.floorEntry(key);
        Map.Entry<Double, Double> ceiling = map.ceilingEntry(key);

        // Boundary handling (Clamping)
        if (floor == null) {
            return ceiling.getValue(); // Below minimum key
        }
        if (ceiling == null) {
            return floor.getValue(); // Above maximum key
        }

        // Linear Interpolation
        double x0 = floor.getKey();
        double y0 = floor.getValue();
        double x1 = ceiling.getKey();
        double y1 = ceiling.getValue();

        double t = (key - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    /**
     * Removes an entry from the lookup table.
     *
     * @param key The key to remove.
     */
    public void remove(double key) {
        map.remove(key);
    }

    /**
     * Clears all entries from the map.
     */
    public void clear() {
        map.clear();
    }

    /**
     * Returns the number of entries in the lookup table.
     */
    public int size() {
        return map.size();
    }

    /**
     * Checks if the map contains no entries.
     */
    public boolean isEmpty() {
        return map.isEmpty();
    }

    /**
     * Gets the lowest key currently stored.
     *
     * @return Minimum key, or {@code Double.NaN} if empty.
     */
    public double getMinKey() {
        return map.isEmpty() ? Double.NaN : map.firstKey();
    }

    /**
     * Gets the highest key currently stored.
     *
     * @return Maximum key, or {@code Double.NaN} if empty.
     */
    public double getMaxKey() {
        return map.isEmpty() ? Double.NaN : map.lastKey();
    }
}