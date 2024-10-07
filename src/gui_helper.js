function roundToNearest(value, base) {
    return Math.round(value / base) * base;
}

export function updateGUI(physicsState, solver, entity_manager, handle_FPS, constants_values) {
    updateDisplayedDebugs(solver, handle_FPS, physicsState);
    updateSliderValues(physicsState, solver, entity_manager, constants_values);
}

function updateDisplayedDebugs(solver, handle_FPS, physicsState) {
    if (!solver.simulating)
        solver.physics_frame_time = -1;

    document.getElementById("fps").innerHTML = roundToNearest(handle_FPS.fps, 60);
    document.getElementById("simulation_rate").innerHTML = roundToNearest(solver.sim_steps * Math.round(handle_FPS.fps), 10);
    document.getElementById("steps").innerHTML = solver.sim_steps.toFixed(0);

    document.getElementById("physics_frame_time").innerHTML = solver.physics_frame_time.toFixed(1);
    document.getElementById("system dt").innerHTML = (solver.dt * 10 ** 3).toFixed(3);
    document.getElementById("render_frame_time").innerHTML = solver.render_frame_time.toFixed(1);
    document.getElementById("CFS_ms").innerHTML = physicsState.averaging.averaged_cfsdt.toFixed(1);
    document.getElementById("CFS_accumulated_error").innerHTML = physicsState.CFS_accumulated_error.toFixed(2);
    document.getElementById("system_energy").innerHTML = physicsState.system_energy.toFixed(3);
    document.getElementById("C_value").innerHTML = physicsState.C_value.toFixed(5);
    document.getElementById("C_dot_value").innerHTML = physicsState.C_dot_value.toFixed(5);

    document.getElementById("q length").innerHTML = (3 * physicsState.getDynamicObjectsLength()).toFixed(0);
    document.getElementById("lambda length").innerHTML = (physicsState.getConstraintLength()).toFixed(0);

}

function updateSliderValues(physicsState, solver, entity_manager, constants_values) {
    const options = document.getElementById("option");
    const slider = document.getElementById("rangeSlider");

    const selected = options.value;
    if (selected == "gravity") {
        physicsState.setGravity(slider.value);
        constants_values.gravity = slider.value;
    } else if (selected == "linear damping") {
        physicsState.setLinearDampingMU(slider.value);
        constants_values.linear_damping = slider.value;
    } else if (selected == "mouse spring") {
        physicsState.setMouseSpringStiffness(slider.value);
        constants_values.mouse_spring = slider.value;
    } else if (selected == "spring joint") {
        physicsState.setSpringJointStiffness(slider.value);
        constants_values.spring_joint = slider.value;
    } else if (selected == "omega constraint") {
        physicsState.setOmegaConstraintValue(slider.value * solver.dt);
        entity_manager.angular_motor_vel = slider.value * solver.dt;
        constants_values.omega_constraint = slider.value;
    } else if (selected == "lagrange limit") {
        physicsState.setLagrangeLimit(slider.value);
        constants_values.lagrange_limit = slider.value;
    }
}