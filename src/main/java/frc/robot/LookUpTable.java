// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.TreeMap;

/**
 * @author Michael Jansen
 */
public class LookUpTable {
    private final TreeMap<InterpolatingDouble, InterpolatingDouble> treeMap;

    public LookUpTable() {
        this.treeMap = new TreeMap<>();
    }

    /**
     * Inserts a distance value pair into the lookup table
     *
     * @param distance   input distance for a value
     * @param value value returned for an input (hood angle/ shooter speed)
     */
    public void put(double distance, double value) {
        this.treeMap.put(new InterpolatingDouble(distance), new InterpolatingDouble(value));
    }

    /**
     * Lookup a value in the table
     *
     * @param distance distance value to lookup
     * @return Value retrieved from the table
     */
    public double lookup(double distance) {
        //convert incoming key(distance) to an InterpotlatingDouble object
        InterpolatingDouble interpDistance = new InterpolatingDouble(distance);

        //Check if the requested key(distance) is in the treemap
        InterpolatingDouble actualValue = treeMap.get(interpDistance);

        if (actualValue == null) {
            //find the next higher value given the key (distance)
            InterpolatingDouble topDistance = treeMap.ceilingKey(interpDistance);

            //find the next lower key(key) given the key (distance)
            InterpolatingDouble bottomDistance = treeMap.floorKey(interpDistance);

            //check if the key(distance) is beyond the table, by checking if one of the nearest keys is null
            if (topDistance == null) {
                //if the key(distance) is beyond the table, return the lower value(which is the top of the table)
                return treeMap.get(bottomDistance).value;
            } else if (bottomDistance == null) {
                //if the key(distance) is below the table, return the higher value(which is the bottom of the table)
                return treeMap.get(topDistance).value;
            }

            //pull the value for the key(distance) above and below the input key(distance)
            InterpolatingDouble topValue = treeMap.get(topDistance);
            InterpolatingDouble bottomValue = treeMap.get(bottomDistance);
            return bottomValue.lerp(topValue, bottomDistance.inverseLerp(topDistance, interpDistance)).value;
        }
        //if the requested key(distance) is in the treemap, return that value
        return actualValue.value;
    }

    private static class InterpolatingDouble implements Comparable<InterpolatingDouble> {
        private final double value;

        private InterpolatingDouble(double value) {
            this.value = value;
        }

        private InterpolatingDouble lerp(InterpolatingDouble other, double x) {
            double dydx = other.value - value;
            return new InterpolatingDouble(dydx * x + value);
        }

        private double inverseLerp(InterpolatingDouble upper, InterpolatingDouble query) {
            double upperToLower = upper.value - value;
            if (upperToLower <= 0) {
                return 0;
            }

            double queryToLower = query.value - value;
            if (queryToLower <= 0) {
                return 0;
            }

            return queryToLower / upperToLower;
        }

        public int compareTo(InterpolatingDouble other) {
            return Double.compare(value, other.value);
        }
    }
}