import {Vector2} from "./linear_algebra.js";
import {PhysicsState} from "./physicsState.js";
import {DynamicObject} from "./dynamicObject.js"
import {FixedXConstraint, FixedYConstraint, LineConstraint} from "./numeric_constraints.js";

const canvas = document.getElementById("myCanvas");
const c = canvas.getContext("2d");

const x_offset = 30;
const y_offset = 50;
canvas.width = window.innerWidth - x_offset;
canvas.height = window.innerHeight - y_offset;

export class Units {
    static WIDTH = 10;
    static HEIGHT = canvas.height * this.WIDTH / canvas.width;

    static scale_c_s = this.WIDTH / canvas.width;
    static scale_s_c = canvas.width / this.WIDTH;

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
        return canvas.height * (this.WIDTH - pos.y) / this.WIDTH;
    }

    static sim_canv(pos) {
        return new Vector2(this.sim_canv_x(pos), this.sim_canv_y(pos));
    }
}

/* void main() {
    return 0;
}
*/

var physicsState = new PhysicsState();
let solver = {
    dt: 1 / 60,
    sim_steps: 8,
    simulating: false,
    physics_frame_time: -1,
    render_frame_time: -1,
};

function updateDisplayedDebugs() {
    if (!solver.simulating)
        solver.physics_frame_time = -1;

    document.getElementById("physics_frame_time").innerHTML = solver.physics_frame_time.toFixed(3);
    document.getElementById("render_frame_time").innerHTML = solver.render_frame_time.toFixed(3);
    document.getElementById("CFS_ms").innerHTML = physicsState.CFS_ms.toFixed(3);
    document.getElementById("CFS_accumulated_error").innerHTML = physicsState.CFS_accumulated_error.toFixed(3);
    document.getElementById("system_energy").innerHTML = physicsState.system_energy.toFixed(3);
    document.getElementById("C_value").innerHTML = physicsState.C_value.toFixed(3);
    document.getElementById("C_dot_value").innerHTML = physicsState.C_dot_value.toFixed(3);
}

function setupScene(version) {
    switch (version) {
        case 1:
            const id_0 = new DynamicObject(new Vector2(5, 5), 1, 0.5);
            const id_1 = new DynamicObject(new Vector2(2, 5), 1, 0.5);
            const id_2 = new DynamicObject(new Vector2(8, 5), 1, 0.5);

            physicsState.addObject(id_0);
            physicsState.addObject(id_1);
            physicsState.addObject(id_2);

            physicsState.addConstraint(new FixedYConstraint(0, id_0.pos.y));

            break;
    }
}

function start() {
    setupScene(1);
    physicsState.initConstraintManager();
}

function update() {

    // physics
    if (solver.simulating) {
        let p_st = performance.now();
            physicsState.step_simulation(solver.dt, solver.sim_steps);
        let p_et = performance.now();
        solver.physics_frame_time = p_et - p_st;
    }

    // render
    let r_st = performance.now();
        c.clearRect(0, 0, canvas.width, canvas.height);
        physicsState.render(canvas, c);
    let r_et = performance.now();
    solver.render_frame_time = r_et - r_st;

    //debugs
    updateDisplayedDebugs();

    requestAnimationFrame(update);
}

document.addEventListener("keydown", function(event) {
    if (event.key == "s") {
        solver.simulating = !solver.simulating;
    }
    if (event.key == "r") {
        physicsState = new PhysicsState();
        start();
    }
    if (event.key == "ArrowRight" && !solver.simulating) {
        let p_st = performance.now();
            physicsState.step_simulation(solver.dt, solver.sim_steps);
        let p_et = performance.now();
        solver.physics_frame_time = p_et - p_st;
    }
});




start();
update();