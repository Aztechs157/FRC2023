package org.assabet.aztechs157.light;

import org.assabet.aztechs157.light.LightSystem.PixelData;

import edu.wpi.first.wpilibj.util.Color;

@FunctionalInterface
public interface LightPattern {
    public Color getColor(final PixelData data);

    public default LightPattern speedBy(final double speed) {
        return (data) -> {
            final var time = (int) Math.floor(data.time() * speed);
            return getColor(new PixelData(data.pos(), time, data.length()));
        };
    }

    public default LightPattern shiftBy(final int offset) {
        return (data) -> {
            final var pos = (data.pos() - offset) % data.length();
            return getColor(new PixelData(pos, data.time(), data.length()));
        };
    }

    public default LightPattern shiftByTime() {
        return (data) -> {
            final var pos = (data.pos() - data.time()) % data.length();
            return getColor(new PixelData(pos, data.time(), data.length()));
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
            final var percentFade = data.pos() / data.length();
            final var red = (diffRed * percentFade) + startColor.red;
            final var green = (diffGreen * percentFade) + startColor.green;
            final var blue = (diffBlue * percentFade) + startColor.blue;
            return new Color(red, green, blue);
        };
    }

    public static LightPattern alternate(
            final int firstLength,
            final int secondLength,
            final LightPattern firstPattern,
            final LightPattern secondPattern) {

        return (data) -> {
            final var period = data.pos() % (firstLength + secondLength);
            return period > firstLength
                    ? firstPattern.getColor(data)
                    : secondPattern.getColor(data);
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

    public static LightPattern climb(int color1Length, int color2Length, Color color1, Color color2, double speed) {
        return alternate(color1Length, color2Length, solid(color1), solid(color2)).shiftByTime().speedBy(speed);
    }
}
