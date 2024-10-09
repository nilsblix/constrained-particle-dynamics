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

export function handleSavedStates(physicsState, saves) {
    const on_color = "#3d85e1"
    const off_color = "#21324e";

    function handleButton(button, key) {
        button.onclick = () => {
            if (saves[key] === -1) {
                // saves[key] = _.cloneDeep(physicsState);
                // saves[key] = Object.assign(Object.create(Object.getPrototypeOf(physicsState)), physicsState);
                // saves[key] = _.cloneDeep(physicsState.deepClone());
                // saves[key] = JSON.stringify(physicsState);
                saves[key] = physicsState.JSONstringify();
                button.style.backgroundColor = on_color;
                console.log("saved state: " + key);
                console.log(key + ": " + saves[key])
                // console.log("some vec2: " + saves[key].getObjectPositionById(0).toString());
            } else {
                // physicsState = _.cloneDeep(saves[key]);
                // physicsState = Object.assign(Object.create(Object.getPrototypeOf(saves[key])), saves[key]);
                // physicsState.loadDeepClone(_.cloneDeep(saves[key]));
                physicsState = JSON.parse(saves[key]);
                console.log("loaded into physicsState: " + key);
            }
        };
    }

    const btn_1 = document.getElementById("info-saved-state-1-button");
    const btn_2 = document.getElementById("info-saved-state-2-button");
    const btn_3 = document.getElementById("info-saved-state-3-button");
    const btn_4 = document.getElementById("info-saved-state-4-button");

    handleButton(btn_1, "state_1");
    handleButton(btn_2, "state_2");
    handleButton(btn_3, "state_3");
    handleButton(btn_4, "state_4");

    const reset_btn = document.getElementById("info-reset-saved-states-button");
    reset_btn.onclick = () => {

        console.log("before: " + saves.state_1);

        saves.state_1 = -1;
        btn_1.style.backgroundColor = off_color;
        saves.state_2 = -1;
        btn_2.style.backgroundColor = off_color;
        saves.state_3 = -1;
        btn_3.style.backgroundColor = off_color;
        saves.state_4 = -1;
        btn_4.style.backgroundColor = off_color;

    
        console.log("after: " + saves);

    };

}
