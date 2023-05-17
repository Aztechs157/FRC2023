package org.assabet.aztechs157.light;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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

    public class RenderCommand extends CommandBase {

        private final LightPattern pattern;

        private RenderCommand(final LightPattern pattern) {
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

    public RenderCommand addPattern(final LightPattern pattern) {
        return new RenderCommand(pattern);
    }
}
