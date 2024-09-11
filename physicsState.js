import {Vector2, Vector, SparseMatrix} from "./linear_algebra.js";
import {ConstraintManager, ConstraintForceSolver} from "./constraint_solvers.js";
import {Gravity, LinearDamping, MouseSpring} from "./forceGenerators.js";
// import {DynamicObject} from "./dynamicObject.js";
// import {Units} from "./main.js"; 
import {FixedXConstraint, FixedYConstraint, LineConstraint} from "./core_constraints.js";
import {FixedPosConstraint} from "./extended_constraints.js";

export class PhysicsState {
   
    // physics depend on these:
    #m_objects = [];
    #m_constraints = [];
    #m_forceGenerators = [];
    // ¿optional? physics
    #m_mouseSpring = new MouseSpring(0);

    // constraintManager
    #CM = new ConstraintManager();
    // constraintForceSolver
    #CFS = new ConstraintForceSolver();

    // rendering depend on these:
    #m_renderedConstraints = [];

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
        this.s_mouse_pos = new Vector2(0,0);

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
        // this.#CM.A = this.#CM.getA();

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

    updateMouse(mouse_pos, mouse_down) {
        this.#m_mouseSpring.mouse_pos = mouse_pos;

        if (mouse_down && !this.#m_mouseSpring.active) {
            if (this.#m_mouseSpring.getClosestDynamicObject(this.#m_objects)) {
                console.log("Mouse Spring Active");
                this.#m_mouseSpring.active = true;
            }
        }

        if (!mouse_down && this.#m_mouseSpring.active) {
            console.log("Mouse Spring Deactivated");
            this.#m_mouseSpring.active = false;
        }
    }

    step_simulation(dt, steps = 1) {
        for (let s = 0; s < steps; s++) {
            const sub_dt = dt / steps;

            // apply forces
            for (let i = 0; i < this.#m_forceGenerators.length; i++) {
                this.#m_forceGenerators[i].apply(this.#m_objects);
            }

            if (this.#m_mouseSpring.active) 
                this.#m_mouseSpring.apply(this.#m_objects);

            // resolve constraints
            if (this.#m_constraints.length != 0) {
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

    render(c) { // c is the canvas context

        // constraints
        let index_phys_const = 0;
        for (let i = 0; i < this.#m_renderedConstraints.length; i++) {
            if (this.#m_renderedConstraints[i] instanceof LineConstraint) {
                this.#m_renderedConstraints[i].render(c, this.#m_objects, this.#CM.lambda.elements[index_phys_const]);
            } else if (this.#m_renderedConstraints[i] instanceof FixedYConstraint) {
                this.#m_renderedConstraints[i].render(c, this.#m_objects, this.#CM.lambda.elements[index_phys_const]);
            } else if (this.#m_renderedConstraints[i] instanceof FixedPosConstraint) {
                index_phys_const++;
                this.#m_renderedConstraints[i].render(c, this.#m_objects, this.#CM.lambda.elements[index_phys_const]);
            } else if (this.#m_renderedConstraints[i] instanceof FixedXConstraint) {
                this.#m_renderedConstraints[i].render(c, this.#m_objects, this.#CM.lambda.elements[index_phys_const]);
            }
            index_phys_const++;
        }

 
        // objects
        // for (let i = 0; i < this.#m_objects.length; i++) {
        //     this.#m_objects[i].render(c);
        // }

        // force generators
        for (let i = 0; i < this.#m_forceGenerators.length; i++) {
            this.#m_forceGenerators[i].render(c, this.#m_objects)
        }

        // this mouse spring
        if (this.#m_mouseSpring.active) {
            this.#m_mouseSpring.render(c, this.#m_objects);
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

    addRenderedConstraint(con) {
        this.#m_renderedConstraints.push(con);
    }

    removeRenderedConstraint(con) {
        this.#m_renderedConstraints.remove(con);
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

        energy += this.#m_mouseSpring.getEnergyApplied(this.#m_objects);

        return energy;

    }

    getConstraintLength() {
        return this.#m_constraints.length;
    }

    addFixedXConstraint(id) {
        const con = new FixedXConstraint(id, this.#m_objects[id].pos.x);
        this.addConstraint(con);
        this.addRenderedConstraint(con);
    }

    addFixedYConstraint(id) {
        const con = new FixedYConstraint(id, this.#m_objects[id].pos.y);
        this.addConstraint(con);
        this.addRenderedConstraint(con);
    }

    addFixedPosConstraint(id) {
        this.addConstraint(new FixedXConstraint(id, this.#m_objects[id].pos.x));
        this.addConstraint(new FixedYConstraint(id, this.#m_objects[id].pos.y));
        this.addRenderedConstraint(new FixedPosConstraint(id, this.#m_objects[id].pos));
    }

    addLineConstraint(id1, id2) {
        const dist = Vector2.distance(this.#m_objects[id1].pos, this.#m_objects[id2].pos);
        const con = new LineConstraint(id1, id2, dist)
        this.addConstraint(con);
        this.addRenderedConstraint(con);
    }

}