package org.assabet.aztechs157.light;

import java.util.function.ToIntFunction;

import org.assabet.aztechs157.light.LightSystem.PixelData;

import edu.wpi.first.wpilibj.util.Color;

@FunctionalInterface
public interface Pattern {
    public Color getColor(final PixelData data);

    public default Pattern scaleTime(final int speed) {
        return scaleTime((data) -> speed);
    }

    public default Pattern scaleTime(final ToIntFunction<PixelData> speedFunc) {
        return (data) -> {
            final var time = data.time() * speedFunc.applyAsInt(data);
            return getColor(data.withTime(time));
        };
    }

    public default Pattern scalePosition(final int scalar) {
        return scalePosition((data) -> scalar);
    }

    public default Pattern scalePosition(final ToIntFunction<PixelData> scalarFunc) {
        return (data) -> {
            final var position = data.position() * scalarFunc.applyAsInt(data);
            return getColor(data.withPosition(position));
        };
    }

    public default Pattern shiftPosition(final int offset) {
        return shiftPosition((data) -> offset);
    }

    public default Pattern shiftPosition(final ToIntFunction<PixelData> offsetFunc) {
        return (data) -> {
            final var position = (data.position() - offsetFunc.applyAsInt(data)) % data.maxPosition();
            return getColor(data.withPosition(position));
        };
    }

    public static Pattern solid(final Color color) {
        return (data) -> color;
    }

    public static Pattern gradientUsingPosition(final Color startColor, final Color endColor) {
        final var diffRed = endColor.red - startColor.red;
        final var diffGreen = endColor.green - startColor.green;
        final var diffBlue = endColor.blue - startColor.blue;

        return (data) -> {
            final var percentFade = data.position() / data.maxPosition();
            final var red = (diffRed * percentFade) + startColor.red;
            final var green = (diffGreen * percentFade) + startColor.green;
            final var blue = (diffBlue * percentFade) + startColor.blue;
            return new Color(red, green, blue);
        };
    }

    public static Pattern gradientUsingTime(final Color startColor, final Color endColor) {
        final var diffRed = endColor.red - startColor.red;
        final var diffGreen = endColor.green - startColor.green;
        final var diffBlue = endColor.blue - startColor.blue;

        return (data) -> {
            final var percentFade = data.time() / data.maxTime();
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
    public static Pattern alternate(
            final int firstLength,
            final int secondLength,
            final Pattern firstPattern,
            final Pattern secondPattern) {

        return (data) -> {
            final var period = data.position() % (firstLength + secondLength);
            return period > firstLength
                    ? firstPattern.getColor(data.withMaxPosition(firstLength))
                    : secondPattern.getColor(data.withMaxPosition(secondLength));
        };
    }

    public static Pattern strobe(
            final int firstLength,
            final int secondLength,
            final Pattern firstPattern,
            final Pattern secondPattern) {
        return (data) -> {
            final var period = data.time() % (firstLength + secondLength);
            return period > firstLength
                    ? firstPattern.getColor(data)
                    : secondPattern.getColor(data);
        };
    }
}
