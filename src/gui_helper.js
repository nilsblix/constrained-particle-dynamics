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

    document.getElementById("system-simulating").innerHTML = solver.simulating ? "TRUE" : "FALSE";
    document.getElementById("system-fr").innerHTML = roundToNearest(handle_FPS.fps, 60);
    document.getElementById("system-sr").innerHTML = roundToNearest(solver.sim_steps * Math.round(handle_FPS.fps), 10);
    document.getElementById("system-steps").innerHTML = solver.sim_steps.toFixed(0);

    document.getElementById("physics-mass-objects").innerHTML = physicsState.getDynamicObjectsLength();
    document.getElementById("physics-force-generators").innerHTML = physicsState.getForceGeneratorsLength();
    document.getElementById("physics-constraints").innerHTML = physicsState.getConstraintLength();

    document.getElementById("system-dt").innerHTML = solver.dt.toFixed(3);
    document.getElementById("system-pdt").innerHTML = solver.physics_frame_time.toFixed(1);
    document.getElementById("system-rdt").innerHTML = solver.render_frame_time.toFixed(1);
    document.getElementById("system-cfsdt").innerHTML = physicsState.averaging.averaged_cfsdt.toFixed(1);
    document.getElementById("system-cfs-err").innerHTML = physicsState.CFS_accumulated_error.toFixed(2);
    document.getElementById("system-energy").innerHTML = physicsState.system_energy.toFixed(3);
    document.getElementById("system-c-eval").innerHTML = physicsState.C_value.toFixed(5);
    document.getElementById("system-c-dot-eval").innerHTML = physicsState.C_dot_value.toFixed(5);

}

function updateSliderValues(physicsState, solver, entity_manager, constants_values) {

    const gravity_slider = document.getElementById("settings-gravity-slider");
    const linear_damping_slider = document.getElementById("settings-linear-damping-slider");
    const mouse_spring_slider = document.getElementById("settings-mouse-spring-slider");
    const spring_joint_slider = document.getElementById("settings-spring-joint-slider");
    const omega_constraint_slider = document.getElementById("settings-omega-constraint-slider");
    const lagrange_limit_slider = document.getElementById("settings-lagrange-limit-slider");

    physicsState.setGravity(gravity_slider.value);
    constants_values.gravity = gravity_slider.value;

    physicsState.setLinearDampingMU(linear_damping_slider.value);
    constants_values.linear_damping = linear_damping_slider.value;

    physicsState.setMouseSpringStiffness(mouse_spring_slider.value);
    constants_values.mouse_spring = mouse_spring_slider.value;

    physicsState.setSpringJointStiffness(spring_joint_slider.value);
    constants_values.spring_joint = spring_joint_slider.value;

    physicsState.setOmegaConstraintValue(omega_constraint_slider.value * solver.dt);
    entity_manager.angular_motor_vel = omega_constraint_slider.value * solver.dt;
    constants_values.omega_constraint = omega_constraint_slider.value;

    physicsState.setLagrangeLimit(lagrange_limit_slider.value);
    constants_values.lagrange_limit = lagrange_limit_slider.value;

}
