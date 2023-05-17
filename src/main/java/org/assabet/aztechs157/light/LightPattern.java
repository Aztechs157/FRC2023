package org.assabet.aztechs157.light;

import org.assabet.aztechs157.light.LightSystem.PixelData;

import edu.wpi.first.wpilibj.util.Color;

@FunctionalInterface
public interface LightPattern {
    public Color getColor(final PixelData data);

    public default LightPattern speedBy(final double speed) {
        return (data) -> {
            final var t = (int) Math.floor(data.t() * speed);
            return getColor(new PixelData(data.x(), t, data.length()));
        };
    }

    public default LightPattern shiftByTime() {
        return (data) -> {
            final var x = (data.x() - data.t()) % data.length();
            return getColor(new PixelData(x, data.t(), data.length()));
        };
    }

    public static LightPattern solid(final Color color) {
        return (data) -> color;
    }

    public static LightPattern stripe(final Color firstColor, final Color secondColor, final int firstLength,
            final int secondLength) {

        return (data) -> {
            final var period = data.x() % (firstLength + secondLength);
            return period > firstLength ? firstColor : secondColor;
        };
    }

    public static LightPattern strobe(final Color firstColor, final Color secondColor, final int firstLength,
            final int secondLength) {
        return (data) -> {
            final var period = data.t() % (firstLength + secondLength);
            return period > firstLength ? firstColor : secondColor;
        };
    }
}
