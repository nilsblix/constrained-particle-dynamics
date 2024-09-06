import {Vector2, Vector, SparseMatrix} from "./linear_algebra.js";
import {ConstraintManager, ConstraintForceSolver} from "./constraints.js";
import {Gravity, LinearDamping} from "./forceGenerators.js";
// import {DynamicObject} from "./dynamicObject.js";
import {Units} from "./main.js";

export class PhysicsState {
    static g_EPSILON = 1e-5;

    #m_objects = [];
    #m_constraints = [];
    #m_forceGenerators = [];

    // constraintManager
    #CM = new ConstraintManager();
    // constraintForceSolver
    #CFS = new ConstraintForceSolver();

    constructor() {
        this.mouseSpringActive = false;
        this.addForceGenerator(new Gravity());
        this.addForceGenerator(new LinearDamping());

        // debugs:
        this.CFS_ms = 0;
        this.CFS_accumulated_error = 0;
        this.system_energy = 0;
        this.C_value = 0;
        this.C_dot_value = 0;

        // init functions:
        this.initConstraintManager();
    }

    #updateDebugValues() {
        this.CFS_accumulated_error = this.#CM.accumulated_error;
        this.system_energy = this.#getSystemEnergy();
        this.C_value = this.#CM.C.magnitude();
        this.C_dot_value = this.#CM.C_dot.magnitude();
    }

    #updateConstraintManager() {
        this.#CM.q =        new Vector(2 * this.#m_objects.length);
        this.#CM.q_dot =    new Vector(2 * this.#m_objects.length);
        this.#CM.Q =        new Vector(2 * this.#m_objects.length);
        this.#CM.W =        new Vector(2 * this.#m_objects.length);
        for (let i = 0; i < this.#m_objects.length; i++) {
            this.#CM.q.elements[2 * i] =        this.#m_objects[i].pos.x;
            this.#CM.q.elements[2 * i + 1] =    this.#m_objects[i].pos.y;

            this.#CM.q_dot.elements[2 * i] =        this.#m_objects[i].vel.x;
            this.#CM.q_dot.elements[2 * i + 1] =    this.#m_objects[i].vel.y;

            this.#CM.Q.elements[2 * i] =        this.#m_objects[i].force.x;
            this.#CM.Q.elements[2 * i + 1] =    this.#m_objects[i].force.y;

            this.#CM.W.elements[2 * i] =        this.#m_objects[i].wass;
            this.#CM.W.elements[2 * i + 1] =    this.#m_objects[i].wass;
        }

        this.#CM.C =        new Vector(this.#m_constraints.length);
        this.#CM.C_dot =    new Vector(this.#m_constraints.length);

        this.#CM.J =        new SparseMatrix(this.#m_constraints.length, 2 * this.#m_objects.length);
        this.#CM.J_dot =    new SparseMatrix(this.#m_constraints.length, 2 * this.#m_objects.length);

        for (let ith_row = 0; ith_row < this.#m_constraints.length; ith_row++) {
            this.#CM.C.elements[ith_row] = this.#m_constraints[ith_row].C_i(this.#CM.q);
            this.#CM.C_dot.elements[ith_row] = this.#m_constraints[ith_row].C_i_dot(this.#CM.q, this.#CM.q_dot);

            this.#CM.J.elements.push(
                ...this.#m_constraints[ith_row].J_i(this.#CM.q, ith_row)
            );
            
            this.#CM.J_dot.elements.push(
                ...this.#m_constraints[ith_row].J_i_dot(this.#CM.q, this.#CM.q_dot, ith_row)
            );
        }

        this.#CM.b = this.#CM.getB();

    }

    initConstraintManager() {
        this.#CM = new ConstraintManager();

        this.#CM.q =        new Vector(2 * this.#m_objects.length);
        this.#CM.q_dot =    new Vector(2 * this.#m_objects.length);
        this.#CM.Q =        new Vector(2 * this.#m_objects.length);
        this.#CM.W =        new Vector(2 * this.#m_objects.length);
        this.#CM.lambda =   new Vector(this.#m_constraints.length);
        this.#CM.J =        new SparseMatrix(this.#m_constraints.length, 2 * this.#m_objects.length);
        this.#CM.J_dot =    new SparseMatrix(this.#m_constraints.length, 2 * this.#m_objects.length);
        this.#CM.accumulated_error = 0;
        this.#CM.C =        new Vector(this.#m_constraints.length);
        this.#CM.C_dot =    new Vector(this.#m_constraints.length);

        this.#CM.lambda = Vector.set_zero_vector(this.#CM.lambda);
    }

    step_simulation(dt, steps = 1) {
        for (let s = 0; s < steps; s++) {
            const sub_dt = dt / steps;

            // apply forces
            for (let i = 0; i < this.#m_forceGenerators.length; i++) {
                this.#m_forceGenerators[i].apply(this.#m_objects);
            }

            // resolve constraints
            if (this.#m_constraints != 0) {
                this.#updateConstraintManager();

                let CFS_st = performance.now();
                    this.#CM.lambda = this.#CFS.CGM(this.#CM, 64) // last int is iteration-count
                let CFS_et = performance.now();
                this.CFS_ms = CFS_et - CFS_st;

                const Q_hat = SparseMatrix.matT_mult_vec(this.#CM.J, this.#CM.lambda);
                for (let i = 0; i < this.#m_objects.length; i++) {
                    const d_force = new Vector2(Q_hat.elements[2 * i], Q_hat.elements[2 * i + 1]);
                    this.#m_objects[i].force = Vector2.addVectors(this.#m_objects[i].force, d_force);
                }
            }

            // integrate
            for (let i = 0; i < this.#m_objects.length; i++) {
                this.#m_objects[i].symplecticEuler(sub_dt);
            }
            // console.log("obj len: " + this.#m_objects.length);

            // not really part of physics_step, more for debugging and data displayed
            this.#updateDebugValues();

        }
    }

    render(canvas, c) { // c is the canvas context

        // render dynamicObjects
        for (let i = 0; i < this.#m_objects.length; i++) {
            const pos = this.#m_objects[i].pos;
            const canv_pos = Units.sim_canv(pos);
            const radius = this.#m_objects[i].drawing_radius * Units.scale_s_c;

            c.beginPath();
            c.fillStyle = "#FF0000";
            c.arc(canv_pos.x, canv_pos.y, radius, 0, 2 * Math.PI);
            c.fill();
            c.closePath();
        }

    }

    addObject(obj) {
        this.#m_objects.push(obj);
    }

    removeObject(obj) {
        this.#m_objects.removeObject(obj);
    }

    addForceGenerator(gen) {
        this.#m_forceGenerators.push(gen);
    }

    removeForceGenerator(gen) {
        this.#m_forceGenerators.remove(gen);
    }

    addConstraint(con) {
        this.#m_constraints.push(con);
    }

    removeConstraint(con) {
        this.#m_constraints.remove(con);
    }

    #getSystemEnergy() {
        let energy = 0;

        // dynamicObjects kinetic energy

        for (let i = 0; i < this.#m_objects.length; i++) {
            energy += this.#m_objects[i].calculateKineticEnergy();
        }
        

        // work done against forceGenerators forces
        for (let i = 0; i < this.#m_forceGenerators.length; i++) {
            energy += this.#m_forceGenerators[i].getEnergyApplied(this.#m_objects);
        }

        return energy;

    }

}