package org.assabet.aztechs157;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSystem {
    private class InternalLightSubsystem extends SubsystemBase {
    }

    private InternalLightSubsystem internalSubsystem = new InternalLightSubsystem();

    private final AddressableLED lights;
    private final AddressableLEDBuffer buffer;
    private final int length;

    public LightSystem(final int port, final int length) {
        this(new AddressableLED(port), length);
    }

    public LightSystem(final AddressableLED lights, final int length) {
        this.lights = lights;
        this.buffer = new AddressableLEDBuffer(length);
        this.length = length;

        lights.setLength(length);
        lights.setData(buffer);
        lights.start();
    }

    public record PixelData(int x, int t, int length) {
    }

    @FunctionalInterface
    public interface Pattern {
        public Color getColor(final PixelData data);

        public default Pattern speedBy(final double speed) {
            return (data) -> {
                final var t = (int) Math.floor(data.t * speed);
                return getColor(new PixelData(data.x, t, data.length));
            };
        }

        public default Pattern shiftByTime() {
            return (data) -> {
                final var x = (data.x - data.t) % data.length;
                return getColor(new PixelData(x, data.t, data.length));
            };
        }

        public static Pattern solid(final Color color) {
            return (data) -> color;
        }

        public static Pattern stripe(final Color firstColor, final Color secondColor, final int firstLength,
                final int secondLength) {

            return (data) -> {
                final var period = data.x % (firstLength + secondLength);
                return period > firstLength ? firstColor : secondColor;
            };
        }

        public static Pattern strobe(final Color firstColor, final Color secondColor, final int firstLength,
                final int secondLength) {
            return (data) -> {
                final var period = data.t % (firstLength + secondLength);
                return period > firstLength ? firstColor : secondColor;
            };
        }
    }

    public class RenderCommand extends CommandBase {

        private final Pattern pattern;

        private RenderCommand(final Pattern pattern) {
            this.pattern = pattern;
            addRequirements(internalSubsystem);
        }

        private int t = 0;

        @Override
        public void initialize() {
            t = 0;
        }

        @Override
        public void execute() {
            for (int x = 0; x < length; x++) {
                final var data = new PixelData(x, t, length);
                buffer.setLED(x, pattern.getColor(data));
            }
            t++;
            lights.setData(buffer);
        }

    }

    public RenderCommand addPattern(final Pattern pattern) {
        return new RenderCommand(pattern);
    }

}
