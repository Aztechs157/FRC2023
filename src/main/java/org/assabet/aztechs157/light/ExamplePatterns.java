package org.assabet.aztechs157.light;

import edu.wpi.first.wpilibj.util.Color;

public class ExamplePatterns {
    public static LightPattern silly() {
        final var blue = LightPattern.solid(new Color(91, 206, 250));
        final var pink = LightPattern.solid(new Color(245, 169, 184));
        final var white = LightPattern.solid(Color.kWhite);

        return new Flexbox()
                .add(1, blue)
                .add(1, pink)
                .add(1, white)
                .add(1, pink)
                .add(1, blue)
                .buildUsingPosition();
    }

    public static LightPattern climb() {
        final var red = LightPattern.solid(Color.kFirstRed);
        final var blue = LightPattern.solid(Color.kFirstBlue);

        return new Flexbox()
                .add(1, red)
                .add(3, blue)
                .add(1, red)
                .add(3, blue)
                .buildUsingPosition()
                .shiftPosition((data) -> -data.time());
    }
}
