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

    public interface Pattern {

        public Color getColor(final PixelData data);
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
                final var color = pattern.getColor(new PixelData(x, t, length));
                if (color != null) {
                    buffer.setLED(x, color);
                }
            }
            t++;
            lights.setData(buffer);
        }

    }

    public RenderCommand addPattern(final Pattern pattern) {
        return new RenderCommand(pattern);
    }
}
