import {Vector2} from "./linear_algebra.js";
import {PhysicsState} from "./physicsState.js";

const canvas = document.getElementById("myCanvas");
const c = canvas.getContext("2d");

const x_offset = 30;
const y_offset = 50;
canvas.width = window.innerWidth - x_offset;
canvas.height = window.innerHeight - y_offset;

class units {
    static WIDTH = 10;
    static HEIGHT = canvas.height * WIDTH / canvas.width;

    static canv_sim_x(pos) {
        return this.WIDTH * pos.x / canvas.width;
    }

    static canv_sim_y(pos) {
        return this.HEIGHT * (canvas.height - pos.y) / canvas.height;
    }

    static canv_sim(pos) {
        return new Vector2(canv_sim_x(pos), canv_sim_y(pos));
    }

    static sim_canv_x(pos) {
        return canvas.width * pos.x / this.WIDTH;
    }

    static sim_canv_y(pos) {
        return canvas.height * (this.WIDTH - pos.y) / this.WIDTH;
    }

    static sim_canv(pos) {
        return new Vector2(sim_canv_x(pos), sim_canv_y(pos));
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
    render_frame_time: -1
};

function updateDisplayedDebugs() {
    document.getElementById("physics_frame_time").innerHTML = solver.physics_frame_time.toFixed(3);
    document.getElementById("render").innerHTML = solver.render_frame_time.toFixed(3);
    document.getElementById("CFS_ms").innerHTML = physicsState.CFS_ms.toFixed(3);
    document.getElementById("CFS_accumulated_error").innerHTML = physicsState.CFS_accumulated_error.toFixed(3);
    document.getElementById("system_energy").innerHTML = physicsState.system_energy.toFixed(3);
    document.getElementById("C_value").innerHTML = physicsState.C_value.toFixed(3);
    document.getElementById("C_dot_value").innerHTML = physicsState.C_dot_value.toFixed(3);
}

function start() {

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
    // func
    let r_et = performance.now();
    solver.render_frame_time = r_et - r_st;

    //debugs
    updateDisplayedDebugs();

    requestAnimationFrame(update);
}




start();
update();