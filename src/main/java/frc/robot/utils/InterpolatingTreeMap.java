package frc.robot.utils;

import java.util.TreeMap;

public class InterpolatingTreeMap extends TreeMap<Double, Double> {
    public Double getInterpolated(Double key) {
        Double exactValue = get(key);
        if (exactValue == null) {
            Double topBound = ceilingKey(key);
            Double bottomBound = floorKey(key);

            if (topBound == null && bottomBound == null) {
                return null;
            } else if (topBound == null) {
                return get(bottomBound);
            } else if (bottomBound == null) {
                return get(topBound);
            }

            Double topElem = get(topBound);
            Double bottomElem = get(bottomBound);
            return bottomElem + (key - bottomBound) / (topBound - bottomBound) * (topElem - bottomElem);
        }
        return exactValue;
    }
}