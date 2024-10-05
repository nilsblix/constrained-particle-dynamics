import {Vector2} from "./linear_algebra.js";
import {PhysicsState} from "./physicsState.js";
import {DynamicObject} from "./dynamicObject.js";
import {setupScene} from "./demo_scenes.js";
import {entity_manager} from "./entity_manager.js";

const canvas = document.getElementById("myCanvas");
const c = canvas.getContext("2d");

const x_offset = 80;
// const x_offset = 2;
canvas.width = window.innerWidth - x_offset;
// canvas.height = window.innerHeight - y_offset;

export class Units {
    static WIDTH = 10;
    static RATIO = 16 / 9;
    static HEIGHT = this.WIDTH / this.RATIO;

    static scale_c_s = this.WIDTH / canvas.width;
    static scale_s_c = canvas.width / this.WIDTH;

    static render_num_lines_x = 20;
    static render_num_lines_y = this.render_num_lines_x / Units.RATIO;

    static canv_sim_x(pos) {
        return this.WIDTH * pos.x / canvas.width;
    }

    static canv_sim_y(pos) {
        return this.HEIGHT * (canvas.height - pos.y) / canvas.height;
    }

    static canv_sim(pos) {
        return new Vector2(this.canv_sim_x(pos), this.canv_sim_y(pos));
    }

    static sim_canv_x(pos) {
        return canvas.width * pos.x / this.WIDTH;
    }

    static sim_canv_y(pos) {
        return canvas.height * (this.HEIGHT - pos.y) / this.HEIGHT;
    }

    static sim_canv(pos) {
        return new Vector2(this.sim_canv_x(pos), this.sim_canv_y(pos));
    }

    static snap_to_grid_x(pos) {
        const l_x = 2 * this.render_num_lines_x;
        const x = pos.x + 0.5 * this.WIDTH / l_x;
        return this.WIDTH / l_x * Math.floor(l_x / this.WIDTH * x);
    }

    static snap_to_grid_y(pos) {
        const l_y = 2 * this.render_num_lines_y;
        const y = pos.y + 0.5 * this.HEIGHT / l_y;
        return this.HEIGHT / l_y * Math.floor(l_y / this.HEIGHT * y);
    }

    static snap_to_grid(pos) {
        return new Vector2(this.snap_to_grid_x(pos), this.snap_to_grid_y(pos));
    }
}

// canvas.height = Units.scale_s_c * (Units.HEIGHT) - y_offset;
canvas.height = Units.scale_s_c * (Units.HEIGHT);

const rect = canvas.getBoundingClientRect();
const keybinds_wrapper = document.querySelector('.keybinds-dropdown');
keybinds_wrapper.style.top = `${rect.top}px`;
keybinds_wrapper.style.left = `${rect.left}px`;

const debugs_wrapper = document.querySelector('.debugs-dropdown');
debugs_wrapper.style.top = `${keybinds_wrapper.getBoundingClientRect().bottom - 1}px`;
debugs_wrapper.style.left = `${rect.left}px`;

/* 
int main() {
    return 0;
}
*/

function roundToNearest(value, base) {
    return Math.round(value / base) * base;
}

var physicsState = new PhysicsState();
let solver = {
    dt: 1 / 120,
    sim_steps: 8,
    simulating: false,
    physics_frame_time: -1,
    render_frame_time: -1,
    standard_radius: 0.05,
    physicsState_is_null: true,
};

let mouse = {
    canv_pos: new Vector2(0,0),
    sim_pos: new Vector2(0,0),
    on_canvas: false,
    left_down:  false,

    toString() {
        return "canv_pos: " + this.canv_pos + " on_canvas: " + this.on_canvas + " left_down: " + this.left_down;
    }
}

let keyboard = {
    arrow_up:  false,
}

const constants_values = {
    gravity: 8,
    linear_damping: 0.2,
    mouse_spring: 100,
    spring_joint: 70,
    omega_constraint: 50,
    lagrange_limit: 7,
}

// var frame = 0;
function updateDisplayedDebugs() {
    if (!solver.simulating)
        solver.physics_frame_time = -1;

    // if (frame % 1200 == 0) {
    //     solver.dt = 1 / handle_FPS.fps;
    //     solver.sim_steps = Math.floor(980 / handle_FPS.fps);
    // }

    // frame++;

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

function updateSliderValues() {
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

function renderBackground() {
    // colors
    const backgroundColor = "#1a1e21";
    const big_line_color = "rgba(90, 90, 90, 1)"
    const small_line_color = "rgba(40, 40, 40, 1)"
    // vars
    const lines_x = Units.render_num_lines_x;
    const lines_y = Units.render_num_lines_y;

    c.beginPath();
    c.fillStyle = backgroundColor;
    c.rect(0, 0, canvas.width, canvas.height);
    c.fill();
    c.closePath();

    // small lines
    c.strokeStyle = small_line_color;
    c.lineWidth = 0.8;
    for (let i = 0; i < lines_x; i++) {
        const clip_space = i / lines_x;
        const delta = 0.5 * 1 / lines_x;
        c.beginPath();
        c.moveTo((clip_space + delta) * canvas.width, 0);
        c.lineTo((clip_space + delta) * canvas.width, canvas.height);
        c.stroke();
        c.closePath();
    }

    for (let i = lines_y; i > 0; i--) {
        const clip_space = i / lines_y;
        const delta = 0.5 * 1 / lines_y;
        c.beginPath();
        c.moveTo(0, (clip_space + delta) * canvas.height);
        c.lineTo(canvas.width, (clip_space + delta) * canvas.height);
        c.stroke();
        c.closePath();
    }

    // big lines
    c.strokeStyle = big_line_color;
    c.lineWidth = 0.8;
    for (let i = 0; i < lines_x; i++) {
        const clip_space = i / lines_x;
        c.beginPath();
        c.moveTo(clip_space * canvas.width, 0);
        c.lineTo(clip_space * canvas.width, canvas.height);
        c.stroke();
        c.closePath();
    }

    for (let i = lines_y; i > 0; i--) {
        const clip_space = i / lines_y;
        c.beginPath();
        c.moveTo(0, clip_space * canvas.height);
        c.lineTo(canvas.width, clip_space * canvas.height);
        c.stroke();
        c.closePath();
    }

    // draw when adding objects etc to the physics engine
    if (!entity_manager.active)
        return;
    // settings:
    c.fillStyle = "rgba(156, 58, 58, 1)";
    c.strokeStyle = "#000000";
    c.lineWidth = 3;
    // constants
    if (entity_manager.snap_to_grid) {
        const c_pos = Units.sim_canv(Units.snap_to_grid(mouse.sim_pos));
        c.beginPath();
        c.arc(c_pos.x, c_pos.y, 10, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();
    } else {
        c.beginPath();
        c.arc(mouse.canv_pos.x, mouse.canv_pos.y, 10, 0, 2 * Math.PI);
        c.stroke();
        c.fill();
        c.closePath();
    }

    // c.beginPath();
    // c.fillStyle = "#ff0000";
    // const c_pos_2 = Units.sim_canv(new Vector2(1,2));
    // c.arc(c_pos_2.x, c_pos_2.y, 10, 0, 2 * Math.PI);
    // c.fill();
    // c.closePath();

}

function start() {
    solver.simulating = false;
    physicsState.initConstraintManager();

    physicsState.setGravity(constants_values.gravity);
    physicsState.setLinearDampingMU(constants_values.linear_damping);
    physicsState.setMouseSpringStiffness(constants_values.mouse_spring);
    physicsState.setSpringJointStiffness(constants_values.spring_joint);
    physicsState.setOmegaConstraintValue(constants_values.omega_constraint * solver.dt);
    physicsState.setLagrangeLimit(constants_values.lagrange_limit);
    entity_manager.angular_motor_vel = constants_values.omega_constraint * solver.dt;

}

function reset() {
    solver.physicsState_is_null = true;

    start();
}

const averaging = { // cfs time is handled in physicsState class

    swap_frames: 20, 

    pdt_sum: 0,
    pdt_frames: 0,
    rdt_sum: 0,
    rdt_frames: 0,

    check_swap_pdt(solver) {
        if (this.pdt_sum > this.swap_frames) {
            this.pdt_sum /= this.pdt_frames;
            solver.physics_frame_time = this.pdt_sum;
            this.pdt_sum = 0;
            this.pdt_frames = 0;
        }
    },

    check_swap_rdt(solver) {
        if (this.rdt_sum > this.swap_frames) {
            this.rdt_sum /= this.rdt_frames;
            solver.render_frame_time = this.rdt_sum;
            this.rdt_sum = 0;
            this.rdt_frames = 0;
        }
    }

}

function update() {

    updateSliderValues();

    if (entity_manager.active && entity_manager.cubic_bezier_active) {
        entity_manager.update(mouse);
    }

    // console.log(mouse.toString());
    if (mouse.on_canvas && !entity_manager.active)
        physicsState.updateMouse(mouse.sim_pos, mouse.left_down)

    // physics
    if (solver.simulating) {
        let p_st = performance.now();
            physicsState.step_simulation(solver.dt, solver.sim_steps);
        let p_et = performance.now();
        // solver.physics_frame_time = p_et - p_st;
        averaging.pdt_sum += p_et - p_st;
        averaging.pdt_frames++;
        averaging.check_swap_pdt(solver);

    } else if (keyboard.arrow_up) {
        physicsState.step_simulation(solver.dt / solver.sim_steps, 1);
    }

    // render
    let r_st = performance.now();
        c.clearRect(0, 0, canvas.width, canvas.height);
        renderBackground();

        physicsState.render(c);

        if (entity_manager.active) 
            entity_manager.render(c, physicsState, solver, mouse);
        
    let r_et = performance.now();
    averaging.rdt_sum += r_et - r_st;
    averaging.rdt_frames++;
    averaging.check_swap_rdt(solver);

    //debugs
    updateDisplayedDebugs();

    requestAnimationFrame(update);
}

document.addEventListener("keydown", function(event) {
    if (event.key == "s") {
        solver.simulating = !solver.simulating;
    }
    if (event.key == "R") {
        physicsState = new PhysicsState();
        reset();
    }
    if (event.key == "1" && solver.physicsState_is_null && !entity_manager.active) {
        setupScene(physicsState, solver, "pratt truss");
        start();
    }
    if (event.key == "2" && solver.physicsState_is_null && !entity_manager.active) {
        setupScene(physicsState, solver, "king post truss");
        start();
    }
    if (event.key == "3" && solver.physicsState_is_null && !entity_manager.active) {
        setupScene(physicsState, solver, "large bridge structure");
        start();
    }
    if (event.key == "4" && solver.physicsState_is_null && !entity_manager.active) {
        setupScene(physicsState, solver, "crane structure");
        start();
    }
    if (event.key == "ArrowRight" && !solver.simulating) {
        let p_st = performance.now();
            physicsState.step_simulation(solver.dt / solver.sim_steps, 1);
        let p_et = performance.now();
        solver.physics_frame_time = p_et - p_st;
    }
    if (event.key == "ArrowUp" && !solver.simulating) {
        keyboard.arrow_up = true;
    }
    // add to physics manager
    if (event.key == "i") {
        entity_manager.active = !entity_manager.active;
        if (entity_manager.recent_entities.length != 0)
            solver.physicsState_is_null = false;
    }
    if (event.key == "I") {
        entity_manager.snap_to_grid = !entity_manager.snap_to_grid;
    }
    if (event.key == "1" && entity_manager.active) {
        entity_manager.dynamicObject(physicsState, solver, mouse, 1, 1);
    }
    if (event.key == "2" && entity_manager.active) {
        entity_manager.dynamicObject(physicsState, solver, mouse, 4, 2);
    }
    if (event.key == "3" && entity_manager.active) {
        entity_manager.dynamicObject(physicsState, solver, mouse, 9, 3);
    }
    if (event.key == "4" && entity_manager.active) {
        entity_manager.dynamicObject(physicsState, solver, mouse, 32, 8);
    }
    if (event.key == "5" && entity_manager.active) {
        entity_manager.dynamicObject(physicsState, solver, mouse, 64, 14);
    }
    if (event.key == "H" && entity_manager.active) {
        entity_manager.addRagdoll(physicsState, solver, mouse);
    }
    if (event.key == "x" && entity_manager.active) {
        entity_manager.fixedXConstraint(physicsState, solver, mouse);
    }
    if (event.key == "y" && entity_manager.active) {
        entity_manager.fixedYConstraint(physicsState, solver, mouse);
    }
    if (event.key == "p" && entity_manager.active) {
        entity_manager.fixedPosConstraint(physicsState, solver, mouse);
    }
    if (event.key == "t" && entity_manager.active) {
        entity_manager.fixedRotConstraint(physicsState, solver, mouse);
    }
    if (event.key == "m" && entity_manager.active) {
        entity_manager.fixedRotOmegaConstraint(physicsState, solver, mouse);
    }
    if (event.key == "l" && entity_manager.active) {
        const id = physicsState.getObjIndexContainingPos(mouse.sim_pos);
        const mouse_over_dynamicObject = (id != -1);
        if (mouse_over_dynamicObject && !entity_manager.drawing_link_constraint) {
            entity_manager.draw_state.entity = id;
            entity_manager.drawing_link_constraint = true;

        }
    }
    if (event.key == "k" && entity_manager.active) {
        const info = physicsState.getClosestEntity(mouse.sim_pos);
        if (info.entity != null && !entity_manager.drawing_spring_joint) {
            entity_manager.drawing_spring_joint = true;
            entity_manager.draw_state = info;
            // info = (of type) {entity: null, offset: null, prev_theta: null, t_param: t, applied_pos: null};
        }
    }
    if (event.key == "j" && entity_manager.active) {
        const info = physicsState.getClosestEntity(mouse.sim_pos);
        // TEMPORARY: change this next line if offset link constraint can support other entities.
        if (!(info.entity instanceof DynamicObject))
            return;
        if (info.entity != null && !entity_manager.drawing_link_constraint) {
            entity_manager.drawing_link_constraint = true;
            entity_manager.draw_state = info;
        }
    }
    if (event.key == "Backspace" && entity_manager.active) {
        entity_manager.removeMostRecentEntity(physicsState);
    }
    if (event.key == "B" && entity_manager.active) {
        if (!entity_manager.cubic_bezier_active) {
            entity_manager.cubic_bezier_active = true;
            entity_manager.addCubicBezierCurve(mouse);
        } else if (entity_manager.cubic_bezier_active) {
            entity_manager.cubic_bezier_active = false;
        }
    }
    if (event.key == "S" && entity_manager.active && entity_manager.cubic_bezier_active) {
        entity_manager.spawnViaCubicBezier(physicsState, solver, mouse);
    }

});

document.addEventListener("keyup", function(event) {
    if (event.key == "ArrowUp" && !solver.simulating) {
        keyboard.arrow_up = false;
    }
    if (event.key == "l" && entity_manager.active) {
        const id = physicsState.getObjIndexContainingPos(mouse.sim_pos);
        const mouse_over_dynamicObject = (id != -1);
        if (mouse_over_dynamicObject && entity_manager.drawing_link_constraint) {
            if (id != entity_manager.draw_start_object) {
                physicsState.addLinkConstraint(entity_manager.draw_state.entity, id);
                const c_length = physicsState.getConstraintLength();
                entity_manager.recent_entities.push({type: "Constraint", id: c_length - 1});
            }
        }
        entity_manager.drawing_link_constraint = false;
    }
    if (event.key == "k" && entity_manager.active) {
        const info = physicsState.getClosestEntity(mouse.sim_pos);
        if (info.entity != null && entity_manager.drawing_spring_joint && info.entity != entity_manager.draw_state.entity) {
            physicsState.addSpringJointWithStates(entity_manager.draw_state, info);
            const gen_length = physicsState.getForceGeneratorsLength();
            entity_manager.recent_entities.push({type: "ForceGenerator", id: gen_length - 1});
        }
        entity_manager.drawing_spring_joint = false;
    }
    if (event.key == "j" && entity_manager.active) {
        const info = physicsState.getClosestEntity(mouse.sim_pos);
        // TEMPORARY: change this next line if offset link constraint can support other entities.
        if (!(info.entity instanceof DynamicObject))
            entity_manager.drawing_link_constraint = false;
        if (info.entity != null && entity_manager.drawing_link_constraint && info.entity != entity_manager.draw_state.entity) {
            physicsState.addOffsetLinkConstraint(entity_manager.draw_state, info);
            const con_length = physicsState.getConstraintLength();
            entity_manager.recent_entities.push({type: "Constraint", id: con_length - 1});
        }
        entity_manager.drawing_link_constraint = false;
    }
});

canvas.addEventListener("mouseenter", function(event) {
    mouse.on_canvas = true;
});

canvas.addEventListener("mouseleave", function(event) {
    mouse.on_canvas = false;
    mouse.left_down = false;
});

canvas.addEventListener("mousedown", function(event) {
    mouse.left_down = true;
});

canvas.addEventListener("mouseup", function(event) {
    mouse.left_down = false;
});

canvas.addEventListener("mousemove", function(event) {
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.x;
    const y = event.clientY - rect.y;
    mouse.canv_pos = new Vector2(x, y);
    mouse.sim_pos = Units.canv_sim(mouse.canv_pos);
})

const handle_FPS = {
    last_time: 0,
    fps: 0,
}

function measureFrameRate(timestamp) {
    if (handle_FPS.last_time > 0) {
        const delta_t = timestamp - handle_FPS.last_time;
        handle_FPS.fps = 1000 / delta_t;
    }
    
    handle_FPS.last_time = timestamp;
    requestAnimationFrame(measureFrameRate);
}

requestAnimationFrame(measureFrameRate);

start();
update();
