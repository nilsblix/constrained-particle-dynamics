import { PhysicsState } from "./physicsState.js";

function roundToNearest(value, base) {
    return Math.round(value / base) * base;
}

let frames = 0;

export function updateGUI(physicsState, solver, entity_manager, handle_FPS, constants_values) {
    updateDisplayedDebugs(solver, handle_FPS, physicsState);
    updateSliderValues(physicsState, solver, entity_manager, constants_values);

    frames++;
    if (frames % 5 == 0) {
        updateAllGraphs(physicsState, solver);
        frames = 0;
    }

}

function updateDisplayedDebugs(solver, handle_FPS, physicsState) {

    if (!solver.simulating)
        solver.physics_frame_time = -1;

    document.getElementById("system-simulating").innerHTML = solver.simulating ? "TRUE" : "FALSE";
    document.getElementById("system-fr").innerHTML = roundToNearest(handle_FPS.fps, 10);
    document.getElementById("system-sr").innerHTML = roundToNearest(solver.sim_steps * roundToNearest(handle_FPS.fps, 10), 10);

    document.getElementById("physics-mass-objects").innerHTML = physicsState.getDynamicObjectsLength();
    document.getElementById("physics-force-generators").innerHTML = physicsState.getForceGeneratorsLength();
    document.getElementById("physics-constraints").innerHTML = physicsState.getConstraintLength();

    document.getElementById("system-dt").innerHTML = (solver.dt * 10 ** 3).toFixed(3);
    document.getElementById("system-pdt").innerHTML = solver.physics_frame_time.toFixed(3);
    document.getElementById("system-rdt").innerHTML = solver.render_frame_time.toFixed(3);
    document.getElementById("system-cfsdt").innerHTML = physicsState.averaging.averaged_cfsdt.toFixed(3);
    document.getElementById("system-cfs-err").innerHTML = physicsState.CFS_accumulated_error.toFixed(2);
    document.getElementById("system-energy").innerHTML = physicsState.system_energy.toFixed(3);
    document.getElementById("system-c-eval").innerHTML = physicsState.C_value.toFixed(5);
    document.getElementById("system-c-dot-eval").innerHTML = physicsState.C_dot_value.toFixed(5);

}

function updateSliderValues(physicsState, solver, entity_manager, constants_values) {

    // settings window
    const gravity_slider = document.getElementById("settings-gravity-slider");
    const linear_damping_slider = document.getElementById("settings-linear-damping-slider");
    const mouse_spring_slider = document.getElementById("settings-mouse-spring-slider");
    const spring_joint_slider = document.getElementById("settings-spring-joint-slider");
    const omega_constraint_slider = document.getElementById("settings-omega-constraint-slider");
    const lagrange_limit_slider = document.getElementById("settings-lagrange-limit-slider");

    // info window
    const info_steps_slider = document.getElementById("info-steps-slider");

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

    solver.sim_steps = info_steps_slider.value;

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

                // saves[key] = physicsState.JSONstringify();

                saves[key] = physicsState.toObject();

                button.style.backgroundColor = on_color;
                console.log("saved state: " + key);
                console.log(key + ": " + saves[key])
                // console.log("some vec2: " + saves[key].getObjectPositionById(0).toString());
            } else {
                // physicsState = _.cloneDeep(saves[key]);
                // physicsState = Object.assign(Object.create(Object.getPrototypeOf(saves[key])), saves[key]);
                // physicsState.loadDeepClone(_.cloneDeep(saves[key]));
                // physicsState = JSON.parse(saves[key]);

                // physicsState.loadDeepClone(JSON.parse(saves[key]));

                setClassInstance(physicsState, saves[key]);

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

export function setClassInstance(class_instance, source_object) {

    function isInstanceOfClass(obj) {
        // Check if the object is an instance of a class (non-plain object)
        if (typeof obj === 'object' && obj !== null) {
            return obj.constructor !== Object;
        }
        return false; // Not an object or it's null
    }

    function isPlainObject(obj) {
        // Check if the object is a plain object (direct instance of Object)
        return Object.prototype.toString.call(obj) === '[object Object]';
    }

    function getObject(val) {
        if (isInstanceOfClass(val)) {
            if (val instanceof PhysicsState)
                return val.toObject();
            else
                return JSON.parse(JSON.stringify(val));
        } else if (isPlainObject(val))
            return val;
        else
            return { val };
    }

    const a = getObject(class_instance);
    const b = getObject(source_object);

    for (let [key_a, data_a] of Object.entries(a)) {
        for (let [key_b, data_b] of Object.entries(b)) {
            if (key_a === key_b) {
                console.log("key names: a ==> " + key_a + " b ==> " + key_b);
                // data_a = JSON.parse(JSON.stringify(data_b));

                // if (isNotPrimitive(key_a)) {
                //     Object.assign(data_a, JSON.parse(JSON.stringify(data_b))); 
                // } else {
                //     setClassInstance()
                // }

                if (isInstanceOfClass(data_a) && isInstanceOfClass(data_b)) {
                    console.log("recursion " + "A class: " + data_a.constructor.name + " B class: " + data_b.constructor.name);
                    setClassInstance(data_a, data_b);
                } else {
                    console.log("assigned")
                    // Object.assign(data_a, JSON.parse(JSON.stringify(data_b)));
                    class_instance[key_a] = JSON.parse(JSON.stringify(data_b));
                    // Object.assign(class_instance[key_a, JSON.parse(JSON.stringify(data_b))])
                }

            }
        }
    }

}

const system_dt_data = [];
const system_pdt_data = [];
const system_rdt_data = [];
const system_cfsdt_data = [];
const system_energy_data = [];
const system_c_eval_data = [];
const system_c_dot_eval_data = [];

function updateAllGraphs(physicsState, solver) {
    function _max(_default, data) {
        return Math.max(_default, Math.max.apply(Math, data))
    }

    function _min(_default, data) {
        return Math.min(_default, Math.min.apply(Math, data));
    }

    function handleSingularGraph(graph_id, data, new_data, default_min_range, default_max_range) {
        const max = _max(default_max_range, data);
        const min = _min(default_min_range, data);
        const canv = document.getElementById(graph_id);
        updateGraph(solver.simulating, canv, data, new_data, min, default_min_range, max, default_max_range);
    }

    handleSingularGraph("system-dt-graph-canvas", system_dt_data, solver.dt * 10 ** 3, 0, 8); // 0.16 is 60 fps
    handleSingularGraph("system-pdt-graph-canvas", system_pdt_data, solver.physics_frame_time, 0, 8);
    handleSingularGraph("system-rdt-graph-canvas", system_rdt_data, solver.render_frame_time, 0, 8);
    handleSingularGraph("system-cfsdt-graph-canvas", system_cfsdt_data, physicsState.averaging.averaged_cfsdt, 0, 8);
    handleSingularGraph("system-energy-graph-canvas", system_energy_data, physicsState.system_energy, -1000, 1000);
    handleSingularGraph("system-c-eval-graph-canvas", system_c_eval_data, physicsState.C_value, 0, 0.1);
    handleSingularGraph("system-c-dot-eval-graph-canvas", system_c_dot_eval_data, physicsState.C_dot_value, 0, 0.13);
}

// todo make lower bound for vis upper and both lower graph bound

function updateGraph(add_new_data, graph_canvas, data, new_data, min_data_range, default_min, max_data_range, default_max) {
    const c = graph_canvas.getContext("2d");

    // Graph settings
    const graphWidth = graph_canvas.width;
    const graphHeight = graph_canvas.height;
    const maxDataPoints = 100;  // Maximum number of points to display

    drawGraph(c, graphWidth, graphHeight, maxDataPoints, data, min_data_range, default_min, max_data_range, default_max);

    // Generate new data point and push to array
    if (add_new_data) {
        data.push(new_data);
    }

    // Keep the data array within the maximum number of data points
    if (data.length > maxDataPoints) {
        data.shift(); // Remove the oldest data point to make room for the new one
    }

    // Redraw the graph
    drawGraph(c, graphWidth, graphHeight, maxDataPoints, data, min_data_range, default_min, max_data_range, default_max);

}

// Function to draw subtle grid lines
function drawGraph(ctx, graphWidth, graphHeight, maxDataPoints, data, min_data_range, default_min, max_data_range, default_max) {

    // Graph style
    // const lineColor = '#e8ca1e';
    const lineColor = '#ffffff'; 
    const backgroundColor = '#21324e';
    const max_line_color = "#e64132";

    // Clear the canvas with the background color
    ctx.fillStyle = backgroundColor;
    ctx.fillRect(0, 0, graphWidth, graphHeight);

    function scaleY(value, min, max) {
        return (1 - (value - min) / (max - min)) * graphHeight;
    }

    // // Draw vertical grid lines
    // for (let x = 0; x < graphWidth; x += gridSize) {
    //     ctx.beginPath();
    //     ctx.moveTo(x, 0);
    //     ctx.lineTo(x, graphHeight);
    //     ctx.stroke();
    // }

    // // Draw horizontal grid lines
    // for (let y = 0; y < graphHeight; y += gridSize) {
    //     ctx.beginPath();
    //     ctx.moveTo(0, y);
    //     ctx.lineTo(graphWidth, y);
    //     ctx.stroke();
    // }

    // // Draw the axis lines (centered on Y-axis)
    // ctx.beginPath();
    // ctx.moveTo(0, graphHeight / 2);  // X axis
    // ctx.lineTo(graphWidth, graphHeight / 2);
    // ctx.strokeStyle = axisColor;
    // ctx.lineWidth = 0.5;
    // ctx.stroke();

    ctx.strokeStyle = max_line_color;
    ctx.lineWidth = 2;

    ctx.beginPath();
    ctx.moveTo(0, scaleY(default_max, min_data_range, max_data_range));
    ctx.lineTo(graphWidth, scaleY(default_max, min_data_range, max_data_range));
    ctx.stroke();
    ctx.closePath();

    ctx.beginPath();
    ctx.moveTo(0, scaleY(default_min, min_data_range, max_data_range));
    ctx.lineTo(graphWidth, scaleY(default_min, min_data_range, max_data_range));
    ctx.stroke();
    ctx.closePath();

    // Draw the graph data points
    ctx.strokeStyle = lineColor;
    ctx.lineWidth = 1;

    // Loop over the data points and plot them
    ctx.beginPath();
    for (let i = 0; i < data.length; i++) {
        const x = i * (graphWidth / maxDataPoints);
        const y = scaleY(data[i], min_data_range, max_data_range);
        if (i === 0) {
            ctx.moveTo(x, y);  // Start point
        } else {
            ctx.lineTo(x, y);  // Connect points
        }
    }
    ctx.stroke();
    ctx.closePath();
}