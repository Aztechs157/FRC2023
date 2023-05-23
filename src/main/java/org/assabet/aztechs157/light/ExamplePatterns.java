package org.assabet.aztechs157.light;

import edu.wpi.first.wpilibj.util.Color;

public class ExamplePatterns {
    public static Pattern silly() {
        final var blue = Pattern.solid(new Color(91, 206, 250));
        final var pink = Pattern.solid(new Color(245, 169, 184));
        final var white = Pattern.solid(Color.kWhite);

        return new Flexbox()
                .add(1, blue)
                .add(1, pink)
                .add(1, white)
                .add(1, pink)
                .add(1, blue)
                .buildUsingPosition();
    }

    public static Pattern climb() {
        final var red = Pattern.solid(Color.kFirstRed);
        final var blue = Pattern.solid(Color.kFirstBlue);

        return new Flexbox()
                .add(1, red)
                .add(3, blue)
                .add(1, red)
                .add(3, blue)
                .buildUsingPosition()
                .shiftPosition((data) -> -data.time());
    }

    public static Pattern strobe() {
        return new Flexbox()
                .add(1, Pattern.gradientUsingTime(Color.kGold, Color.kBlue))
                .add(1, Pattern.gradientUsingTime(Color.kBlue, Color.kGold))
                .buildUsingTime();
    }
}
