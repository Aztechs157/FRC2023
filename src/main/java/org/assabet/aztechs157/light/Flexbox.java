package org.assabet.aztechs157.light;

import java.util.ArrayList;

public class Flexbox {
    private ArrayList<Pattern> patterns = new ArrayList<>();
    private ArrayList<Integer> units = new ArrayList<>();

    public Flexbox add(final int unit, final Pattern pattern) {
        patterns.add(pattern);
        units.add(unit);
        return this;
    }

    private Pattern intoPattern(final boolean flag) {
        final var breakpoints = new ArrayList<Integer>(units.size());

        var totalUnits = 0;
        for (final var unit : units) {
            totalUnits += unit;
        }

        var runningTotalUnits = 0;
        for (final var unit : units) {
            runningTotalUnits += unit / totalUnits;
            breakpoints.add(runningTotalUnits);
        }

        return (data) -> {
            final var current = flag
                    ? data.position() / data.maxPosition()
                    : data.time() / data.maxTime();

            var highest = 0;
            for (int i = 0; i < breakpoints.size(); i++) {
                if (current >= breakpoints.get(i)) {
                    highest = i;
                } else {
                    break;
                }
            }

            // TODO: Send `data` with proper maxPosition/maxTime
            return patterns.get(highest).getColor(data);
        };
    }

    public Pattern buildUsingPosition() {
        return intoPattern(true);
    }

    public Pattern buildUsingTime() {
        return intoPattern(false);
    }

}
