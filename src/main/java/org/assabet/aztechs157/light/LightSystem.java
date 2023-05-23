package org.assabet.aztechs157.light;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightSystem extends SubsystemBase {

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

    private int defaultTimeCycle = (int) Math.floor(1 / TimedRobot.kDefaultPeriod);

    public LightSystem withDefaultTimeCycle(final int timeCycle) {
        this.defaultTimeCycle = timeCycle;
        return this;
    }

    public record PixelData(int position, int time, int maxPosition, int maxTime) {
        public PixelData withPosition(final int position) {
            return new PixelData(position, time, maxPosition, maxTime);
        }

        public PixelData withTime(final int time) {
            return new PixelData(position, time, maxPosition, maxTime);
        }

        public PixelData withMaxPosition(final int maxPosition) {
            return new PixelData(position, time, maxPosition, maxTime);
        }

        public PixelData withMaxTime(final int maxTime) {
            return new PixelData(position, time, maxPosition, maxTime);
        }
    }

    public class RenderCommand extends CommandBase {

        private final LightPattern pattern;

        private RenderCommand(final LightPattern pattern) {
            this.pattern = pattern;
            addRequirements(LightSystem.this);
        }

        private int time = 0;

        @Override
        public void initialize() {
            time = 0;
        }

        @Override
        public void execute() {
            for (int position = 0; position < length; position++) {
                final var data = new PixelData(position, time, length, defaultTimeCycle);
                buffer.setLED(position, pattern.getColor(data));
            }
            time++;
            lights.setData(buffer);
        }
    }

    public RenderCommand addPattern(final LightPattern pattern) {
        return new RenderCommand(pattern);
    }
}
