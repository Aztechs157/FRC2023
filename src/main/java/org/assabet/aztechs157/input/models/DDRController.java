// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.assabet.aztechs157.input.models;

import org.assabet.aztechs157.input.Model;
import org.assabet.aztechs157.input.values.Button;

/** Add your docs here. */
public class DDRController extends Model {

    public DDRController(final int joystickId) {
        super(joystickId);
    }

    public final Button up = button("Up", 0);
    public final Button down = button("Down", 0);
    public final Button right = button("Right", 0);
    public final Button left = button("Left", 0);
    public final Button cross = button("Cross", 0);
    public final Button circle = button("Circle", 0);
    public final Button triangle = button("Triangle", 0);
    public final Button square = button("Square", 0);
    public final Button start = button("Start", 0);
    public final Button select = button("Select", 0);
}
