package org.assabet.aztechs157.light;

import org.assabet.aztechs157.light.LightSystem.PixelData;

import edu.wpi.first.wpilibj.util.Color;

@FunctionalInterface
public interface LightPattern {
    public Color getColor(final PixelData data);

    public default LightPattern speedBy(final double speed) {
        return (data) -> {
            final var time = (int) Math.floor(data.time() * speed);
            return getColor(data.withTime(time));
        };
    }

    public default LightPattern shiftBy(final int offset) {
        return (data) -> {
            final var position = (data.position() - offset) % data.length();
            return getColor(data.withPosition(position));
        };
    }

    public default LightPattern shiftByTime() {
        return (data) -> {
            final var position = (data.position() - data.time()) % data.length();
            return getColor(data.withPosition(position));
        };
    }

    public static LightPattern solid(final Color color) {
        return (data) -> color;
    }

    public static LightPattern gradient(final Color startColor, final Color endColor) {
        final var diffRed = endColor.red - startColor.red;
        final var diffGreen = endColor.green - startColor.green;
        final var diffBlue = endColor.blue - startColor.blue;

        return (data) -> {
            final var percentFade = data.position() / data.length();
            final var red = (diffRed * percentFade) + startColor.red;
            final var green = (diffGreen * percentFade) + startColor.green;
            final var blue = (diffBlue * percentFade) + startColor.blue;
            return new Color(red, green, blue);
        };
    }

    /**
     * Note that `firstPattern` and `secondPattern` will have their `data.length`
     * set to `firstLength` and `secondLength` respectively. This is so things such
     * as gradients display properly.
     *
     * @param firstLength
     * @param secondLength
     * @param firstPattern
     * @param secondPattern
     * @return
     */
    public static LightPattern alternate(
            final int firstLength,
            final int secondLength,
            final LightPattern firstPattern,
            final LightPattern secondPattern) {

        return (data) -> {
            final var period = data.position() % (firstLength + secondLength);
            return period > firstLength
                    ? firstPattern.getColor(data.withLength(firstLength))
                    : secondPattern.getColor(data.withLength(secondLength));
        };
    }

    public static LightPattern strobe(
            final int firstLength,
            final int secondLength,
            final LightPattern firstPattern,
            final LightPattern secondPattern) {
        return (data) -> {
            final var period = data.time() % (firstLength + secondLength);
            return period > firstLength
                    ? firstPattern.getColor(data)
                    : secondPattern.getColor(data);
        };
    }
}
