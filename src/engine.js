import { Vector2 } from "./linear_algebra.js";
import { PhysicsState } from "./physicsState.js";
import { DynamicObject } from "./dynamicObject.js";
import { setupScene } from "./demo_scenes.js";
import { editor } from "./editor.js";
import { Units } from "./units.js";
import { updateGUI, handleSavedStates, drawScaleIndicator} from "./gui_helper.js";

var physicsState = new PhysicsState();
let solver = {
    dt: 1 / 120,
    sim_steps: 12,
    simulating: false,
    physics_frame_time: -1,
    render_frame_time: -1,
    physicsState_is_null: true,
};

let saves = {
    state_1: -1,
    state_2: -1,
    state_3: -1,
    state_4: -1,
}

let mouse = {
    canv_pos: new Vector2(0, 0),
    sim_pos: new Vector2(0, 0),
    on_canvas: false,
    left_down: false,

    toString() {
        return "canv_pos: " + this.canv_pos + " on_canvas: " + this.on_canvas + " left_down: " + this.left_down;
    }
}

let keyboard = {
    arrow_up: false,
}

const constants_values = {
    gravity: 8,
    linear_damping: 0.2,
    mouse_spring: 100,
    spring_joint: 70,
    omega_constraint: 50,
    lagrange_limit: 7,
}

const averaging = { // cfs time is handled in physicsState class
    swap_frames: 5,
    pdt_sum: 0,
    pdt_frames: 0,
    rdt_sum: 0,
    rdt_frames: 0,
    systemdt_sum: 0,
    systemdt_frames: 0,

    check_swap_pdt(solver) {
        if (this.pdt_frames > this.swap_frames) {
            this.pdt_sum /= this.pdt_frames;
            solver.physics_frame_time = this.pdt_sum;
            this.pdt_sum = 0;
            this.pdt_frames = 0;
        }
    },

    check_swap_rdt(solver) {
        if (this.rdt_frames > this.swap_frames) {
            this.rdt_sum /= this.rdt_frames;
            solver.render_frame_time = this.rdt_sum;
            this.rdt_sum = 0;
            this.rdt_frames = 0;
        }
    },

    check_swap_systemdt(solver) {
        if (this.systemdt_frames > 3 * this.swap_frames) {
            this.systemdt_sum /= this.systemdt_frames;
            solver.dt = this.systemdt_sum;
            this.systemdt_sum = 0;
            this.systemdt_frames = 0;
        }
    },
}

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
}

function setSimConstants() {
    physicsState.setGravity(constants_values.gravity);
    physicsState.setLinearDampingMU(constants_values.linear_damping);
    physicsState.setMouseSpringStiffness(constants_values.mouse_spring);
    physicsState.setSpringJointStiffness(constants_values.spring_joint);
    physicsState.setOmegaConstraintValue(constants_values.omega_constraint * solver.dt);
    physicsState.setLagrangeLimit(constants_values.lagrange_limit);
    editor.angular_motor_vel = constants_values.omega_constraint * solver.dt;
}

function renderBackground(canvas, c) {
    // colors
    const backgroundColor = "#0c0e11";
    const big_line_color = "#202123"
    const small_line_color = "#141819"
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
    c.lineWidth = 1.2;
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
    if (!editor.active)
        return;
    // settings:
    c.fillStyle = "rgba(156, 58, 58, 1)";
    c.strokeStyle = "#000000";
    c.lineWidth = 3;
    // constants

    // draw the thing specifying if user is in editor or not
        // draw a red mouse
    const red_dot_at_mouse = editor.snap_to_grid ? Units.sim_canv(Units.snap_to_grid(mouse.sim_pos)) : mouse.canv_pos;
    c.beginPath();
    c.arc(red_dot_at_mouse.x, red_dot_at_mouse.y, 8, 0, 2 * Math.PI);
    c.stroke();
    c.fill();
    c.closePath();
        // draw small rectangle in the border

    const num_boxes = 1;
    const rect_width = num_boxes * canvas.width / Units.render_num_lines_x;
    const rect_height = num_boxes * canvas.height / Units.render_num_lines_y;
    const rect_x = canvas.width - rect_width;
    const rect_y = canvas.height - rect_height;

    // const rect_multiple_of_canv = 1 / 10;
    // const rect_sides = new Vector2(rect_multiple_of_canv * canvas.width, rect_multiple_of_canv * canvas.height);
    // const rect_pos = new Vector2(canvas.width - rect_sides.x, canvas.height - rect_sides.y);
    c.beginPath();
    c.rect(rect_x, rect_y, rect_width, rect_height);
    c.stroke();
    c.fill();
    c.closePath();

}

export function start(window, canvas, c) {

    solver.simulating = false;
    physicsState.initConstraintManager();

    setSimConstants();

}

function reset(window, canvas) {
    solver.physicsState_is_null = true;

    solver.simulating = false;
    physicsState.initConstraintManager();

    setSimConstants();
}

export function update(canvas, c) {

    averaging.systemdt_sum += 1 / handle_FPS.fps;
    averaging.systemdt_frames++;
    averaging.check_swap_systemdt(solver);

    if (editor.active && editor.cubic_bezier_active) {
        editor.update(mouse);
    }

    if (mouse.on_canvas && !editor.active)
        physicsState.updateMouse(mouse.sim_pos, mouse.left_down)

    // physics
    if (solver.dt > 100 * 1e-3) {
        requestAnimationFrame(() => update(canvas, c));
        // throw new Error("Skipped current frame as delta time was too high. The next frame will continue as normal");
        return;
    }
    if (solver.simulating && document.hasFocus()) {
        let p_st = performance.now();
        physicsState.process_sim(solver.dt, solver.sim_steps);
        let p_et = performance.now();
        averaging.pdt_sum += p_et - p_st;
        averaging.pdt_frames++;
        averaging.check_swap_pdt(solver);

    } else if (keyboard.arrow_up && document.hasFocus()) {
        physicsState.process_sim(solver.dt / solver.sim_steps, 1);
        solver.simulating = true;
    }

    // render
    let r_st = performance.now();
    c.clearRect(0, 0, canvas.width, canvas.height);
    renderBackground(canvas, c);
    drawScaleIndicator(canvas, c);

    physicsState.render(c);

    if (editor.active)
        editor.render(c, physicsState, solver, mouse);

    let r_et = performance.now();
    averaging.rdt_sum += r_et - r_st;
    averaging.rdt_frames++;
    averaging.check_swap_rdt(solver);

    //debugs
    measureFrameRate(performance.now());

    // TEMPORARY: if i decide to pursue saved states: this is where it gets called/ updated
    // handleSavedStates(physicsState, saves);
    updateGUI(canvas, physicsState, solver, editor, handle_FPS, constants_values, mouse);

    if (keyboard.arrow_up)
        solver.simulating = false;

    requestAnimationFrame(() => update(canvas, c));

}

export function handleInputs(window, canvas) {
    document.addEventListener("keydown", function (event) {
        if (event.key == "s") {
            solver.simulating = !solver.simulating;
        }
        if (event.key == "R") {
            physicsState = new PhysicsState();
            reset(window, canvas);
        }
        if (event.key == "1" && solver.physicsState_is_null && !editor.active) {
            setupScene(physicsState, solver, "pratt truss");
        }
        if (event.key == "2" && solver.physicsState_is_null && !editor.active) {
            setupScene(physicsState, solver, "king post truss");
        }
        if (event.key == "3" && solver.physicsState_is_null && !editor.active) {
            setupScene(physicsState, solver, "large bridge structure");
        }
        if (event.key == "4" && solver.physicsState_is_null && !editor.active) {
            setupScene(physicsState, solver, "crane structure");
        }
        if (event.key == "5" && solver.physicsState_is_null && !editor.active) {
            setupScene(physicsState, solver, "standard pratt truss");
        }
        if (event.key == "ArrowRight" && !solver.simulating) {
            let p_st = performance.now();
            physicsState.process_sim(solver.dt / solver.sim_steps, 1);
            let p_et = performance.now();
            solver.physics_frame_time = p_et - p_st;
        }
        if (event.key == "ArrowUp" && !solver.simulating) {
            keyboard.arrow_up = true;
        }
        // add to physics manager
        if (event.key == "i") {
            editor.active = !editor.active;
            if (editor.recent_entities.length != 0)
                solver.physicsState_is_null = false;
        }
        if (event.key == "I") {
            editor.snap_to_grid = !editor.snap_to_grid;
        }
        if (event.key == "1" && editor.active) {
            editor.dynamicObject(physicsState, mouse, 1, 1 * editor.standard_object_radius);
        }
        if (event.key == "2" && editor.active) {
            editor.dynamicObject(physicsState, mouse, 4, 2 * editor.standard_object_radius);
        }
        if (event.key == "3" && editor.active) {
            editor.dynamicObject(physicsState, mouse, 9, 3 * editor.standard_object_radius);
        }
        if (event.key == "4" && editor.active) {
            editor.dynamicObject(physicsState, mouse, 32, 8 * editor.standard_object_radius);
        }
        if (event.key == "5" && editor.active) {
            editor.dynamicObject(physicsState, mouse, 64, 16 * editor.standard_object_radius);
        }
        if (event.key == "0" && editor.active) {
            editor.dynamicObject(physicsState, mouse);
        }
        if (event.key == "H" && editor.active) {
            editor.addRagdoll(physicsState, solver, mouse);
        }
        if (event.key == "x" && editor.active) {
            editor.fixedXConstraint(physicsState, solver, mouse);
        }
        if (event.key == "y" && editor.active) {
            editor.fixedYConstraint(physicsState, solver, mouse);
        }
        if (event.key == "p" && editor.active) {
            editor.fixedPosConstraint(physicsState, solver, mouse);
        }
        if (event.key == "t" && editor.active) {
            editor.fixedRotConstraint(physicsState, solver, mouse);
        }
        if (event.key == "m" && editor.active) {
            editor.fixedRotOmegaConstraint(physicsState, solver, mouse);
        }
        if (event.key == "l" && editor.active) {
            const id = physicsState.getObjIndexContainingPos(mouse.sim_pos);
            const mouse_over_dynamicObject = (id != -1);
            if (mouse_over_dynamicObject && !editor.drawing_link_constraint) {
                editor.draw_state.entity = id;
                editor.drawing_link_constraint = true;

            }
        }
        if (event.key == "k" && editor.active) {
            const info = physicsState.getClosestEntity(mouse.sim_pos);
            if (info.entity != null && !editor.drawing_spring_joint) {
                editor.drawing_spring_joint = true;
                editor.draw_state = info;
                // info = (of type) {entity: null, offset: null, prev_theta: null, t_param: t, applied_pos: null};
            }
        }
        if (event.key == "j" && editor.active) {
            const info = physicsState.getClosestEntity(mouse.sim_pos);
            // TEMPORARY: change this next line if offset link constraint can support other entities.
            if (!(info.entity instanceof DynamicObject))
                return;
            if (info.entity != null && !editor.drawing_link_constraint) {
                editor.drawing_link_constraint = true;
                editor.draw_state = info;
            }
        }
        if (event.key == "Backspace" && editor.active) {
            editor.removeMostRecentEntity(physicsState);
        }
        if (event.key == "B" && editor.active) {
            if (!editor.cubic_bezier_active) {
                editor.cubic_bezier_active = true;
                editor.addCubicBezierCurve(mouse);
            } else if (editor.cubic_bezier_active) {
                editor.cubic_bezier_active = false;
            }
        }
        if (event.key == "S" && editor.active && editor.cubic_bezier_active) {
            editor.spawnViaCubicBezier(physicsState, solver, mouse);
        }
    });

    document.addEventListener("keyup", function (event) {
        if (event.key == "ArrowUp" && !solver.simulating) {
            keyboard.arrow_up = false;
        }
        if (event.key == "l" && editor.active) {
            const id = physicsState.getObjIndexContainingPos(mouse.sim_pos);
            const mouse_over_dynamicObject = (id != -1);
            if (mouse_over_dynamicObject && editor.drawing_link_constraint) {
                if (id != editor.draw_start_object) {
                    physicsState.addLinkConstraint(editor.draw_state.entity, id);
                    const c_length = physicsState.getConstraintLength();
                    editor.recent_entities.push({ type: "Constraint", id: c_length - 1 });
                }
            }
            editor.drawing_link_constraint = false;
        }
        if (event.key == "k" && editor.active) {
            const info = physicsState.getClosestEntity(mouse.sim_pos);
            if (info.entity != null && editor.drawing_spring_joint && info.entity != editor.draw_state.entity) {
                physicsState.addSpringJointWithStates(editor.draw_state, info);
                const gen_length = physicsState.getForceGeneratorsLength();
                editor.recent_entities.push({ type: "ForceGenerator", id: gen_length - 1 });
            }
            editor.drawing_spring_joint = false;
        }
        if (event.key == "j" && editor.active) {
            const info = physicsState.getClosestEntity(mouse.sim_pos);
            // TEMPORARY: change this next line if offset link constraint can support other entities.
            if (!(info.entity instanceof DynamicObject))
                editor.drawing_link_constraint = false;
            if (info.entity != null && editor.drawing_link_constraint && info.entity != editor.draw_state.entity) {
                physicsState.addOffsetLinkConstraint(editor.draw_state, info);
                const con_length = physicsState.getConstraintLength();
                editor.recent_entities.push({ type: "Constraint", id: con_length - 1 });
            }
            editor.drawing_link_constraint = false;
        }
    });

    canvas.addEventListener("mouseenter", function (event) {
        mouse.on_canvas = true;
    });

    canvas.addEventListener("mouseleave", function (event) {
        mouse.on_canvas = false;
        mouse.left_down = false;
    });

    canvas.addEventListener("mousedown", function (event) {
        mouse.left_down = true;
    });

    canvas.addEventListener("mouseup", function (event) {
        mouse.left_down = false;
    });

    canvas.addEventListener("mousemove", function (event) {
        const rect = canvas.getBoundingClientRect();
        const x = event.clientX - rect.x;
        const y = event.clientY - rect.y;
        mouse.canv_pos = new Vector2(x, y);
        mouse.sim_pos = Units.canv_sim(mouse.canv_pos);
    })

}