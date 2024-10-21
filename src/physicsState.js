import { Vector2, Vector, SparseMatrix } from "./linear_algebra.js";
import { ConstraintManager, ConstraintForceSolver } from "./constraint_solvers.js";
import { Gravity, Wind, SpringJoint, LinearDamping, MouseSpring } from "./forceGenerators.js";
// import {DynamicObject} from "./dynamicObject.js";
import { Units } from "./units.js";
import { Colours, LineWidths, Extras } from "./render_settings.js";
import { FixedXConstraint, FixedYConstraint, LinkConstraint, OffsetLinkConstraint, FixedRotationConstraint, FixedOmegaConstraint } from "./core_constraints.js";
import { FixedPosConstraint } from "./extended_constraints.js";

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

    // simulation constants:
    #m_gravity = 1.6;
    #m_linear_damping_MU = 0.5;
    #m_spring_joint_stiffness = 5;
    #m_omega_constraint_value = 20;
    #lagrange_mult_limit = 1 * 10 ** 4;

    constructor() {
        this.addForceGenerator(new Gravity());
        // this.addForceGenerator(new Wind());
        this.addForceGenerator(new LinearDamping());

        // debugs:
        this.averaging = {
            swap_frames: 10,

            averaged_cfsdt: -1,

            cfsdt_sum: 0,
            cfsdt_frames: 0,

            check_swap_cfsdt() {
                if (this.cfsdt_sum > this.swap_frames) {
                    this.cfsdt_sum /= this.cfsdt_frames;
                    this.averaged_cfsdt = this.cfsdt_sum;
                    this.cfsdt_sum = 0;
                    this.cfsdt_frames = 0;
                }
            },

        }

        this.CFS_accumulated_error = 0;
        this.system_energy = 0;
        this.C_value = 0;
        this.C_dot_value = 0;
        this.s_mouse_pos = new Vector2(0, 0);

        // init functions:
        this.initConstraintManager();
    }

    JSONstringify() {

        function clone(a) {
            return a;
        }

        return JSON.stringify({
            ["#m_objects"]: this.#m_objects,
            ["m_forceGenerators"]: this.#m_forceGenerators,
            ["m_constraints"]: this.#m_constraints,
            ["CM"]: this.#CM,
            ["CFS"]: this.#CFS,
            ["m_renderedConstraints"]: this.#m_renderedConstraints,

            ["swap_frames"]: clone(this.averaging.swap_frames),
            ["averaged_cfsdt"]: clone(this.averaging.averaged_cfsdt),
            ["cfsdt_sum"]: clone(this.averaging.cfsdt_sum),
            ["cfsdt_frames"]: clone(this.averaging.cfsdt_frames),

            ["CFS_accumulated_error"]: clone(this.CFS_accumulated_error),
            ["system_energy"]: clone(this.system_energy),
            ["C_value"]: clone(this.C_value),
            ["C_dot_value"]: clone(this.C_dot_value),
            ["s_mouse_pos"]: clone(this.s_mouse_pos),
        });
    }

    toObject() {

        function clone(a) {
            return a;
        }

        return {
            m_objects: clone(this.#m_objects),
            m_forceGenerators: clone(this.#m_forceGenerators),
            m_constraints: clone(this.#m_constraints),
            m_mouseSpring: clone(this.#m_mouseSpring),
            // q_elem: this.#CM.q.elements,
            // q_dot_elem: this.#CM.q_dot.elements,
            // Q_elem: this.#CM.Q.elements,
            // W_elem: this.#CM.W.elements,
            // lambda_elem: this.#CM.lambda.elements,
            // J_elem: 
            CM: clone(this.#CM),
            CFS: clone(this.#CFS),
            m_renderedConstraints: clone(this.#m_renderedConstraints),

            swap_frames: clone(this.averaging.swap_frames),
            averaged_cfsdt: clone(this.averaging.averaged_cfsdt),
            cfsdt_sum: clone(this.averaging.cfsdt_sum),
            cfsdt_frames: clone(this.averaging.cfsdt_frames),

            CFS_accumulated_error: clone(this.CFS_accumulated_error),
            system_energy: clone(this.system_energy),
            C_value: clone(this.C_value),
            C_dot_value: clone(this.C_dot_value),
            s_mouse_pos: clone(this.s_mouse_pos),
        }
    }

    loadStateObject(a) { // a is a deep clone

        function assign(target, source) {
            Object.assign(target, source);
            // target = structuredClone(source);
            // target = _.cloneDeep(source);3
        }

        assign(this.#m_objects, a.m_objects);
        assign(this.#m_forceGenerators, a.m_forceGenerators);
        assign(this.#m_constraints, a.m_constraints);
        assign(this.#m_mouseSpring, a.m_mouseSpring);
        assign(this.#CM, a.CM);
        assign(this.#CFS, a.CFS);
        assign(this.#m_renderedConstraints, a.m_renderedConstraints);
        assign(this.averaging.swap_frames, a.swap_frames);
        assign(this.averaging.averaged_cfsdt, a.averaged_cfsdt);
        assign(this.averaging.cfsdt_sum, a.cfsdt_sum);
        assign(this.averaging.cfsdt_frames, a.cfsdt_frames);
        assign(this.CFS_accumulated_error, a.CFS_accumulated_error);
        assign(this.system_energy, a.system_energy);
        assign(this.C_value, a.C_value);
        assign(this.C_dot_value, a.C_dot_value);
        assign(this.s_mouse_pos, a.s_mouse_pos);

    }

    #updateDebugValues() {
        this.CFS_accumulated_error = this.#CM.accumulated_error;
        this.system_energy = this.#getSystemEnergy();
        this.C_value = this.#CM.C.magnitude();
        this.C_dot_value = this.#CM.C_dot.magnitude();
    }

    #updateSimulationConstants() {
        this.#m_forceGenerators[0].gravity = this.#m_gravity;
        this.#m_forceGenerators[1].MU = this.#m_linear_damping_MU;

        for (let i = 0; i < this.#m_forceGenerators.length; i++) {
            if (this.#m_forceGenerators[i] instanceof SpringJoint)
                this.#m_forceGenerators[i].setStiffness(this.#m_spring_joint_stiffness);
        }

        for (let i = 0; i < this.#m_constraints.length; i++) {
            if (this.#m_constraints[i] instanceof FixedOmegaConstraint)
                this.#m_constraints[i].velocity = this.#m_omega_constraint_value;
        }

    }

    #updateConstraintManager() {
        this.#CM.q = new Vector(3 * this.#m_objects.length);
        this.#CM.q_dot = new Vector(3 * this.#m_objects.length);
        this.#CM.Q = new Vector(3 * this.#m_objects.length);
        this.#CM.W = new Vector(3 * this.#m_objects.length);
        for (let i = 0; i < this.#m_objects.length; i++) {
            this.#CM.q.elements[3 * i] = this.#m_objects[i].pos.x;
            this.#CM.q.elements[3 * i + 1] = this.#m_objects[i].pos.y;
            this.#CM.q.elements[3 * i + 2] = this.#m_objects[i].theta;

            this.#CM.q_dot.elements[3 * i] = this.#m_objects[i].vel.x;
            this.#CM.q_dot.elements[3 * i + 1] = this.#m_objects[i].vel.y;
            this.#CM.q_dot.elements[3 * i + 2] = this.#m_objects[i].omega;

            this.#CM.Q.elements[3 * i] = this.#m_objects[i].force.x;
            this.#CM.Q.elements[3 * i + 1] = this.#m_objects[i].force.y;
            this.#CM.Q.elements[3 * i + 2] = this.#m_objects[i].tau;

            this.#CM.W.elements[3 * i] = this.#m_objects[i].w;
            this.#CM.W.elements[3 * i + 1] = this.#m_objects[i].w;
            // this.#CM.W.elements[3 * i + 2] =    this.#m_objects[i].w;
            this.#CM.W.elements[3 * i + 2] = 1 / this.#m_objects[i].I;
        }

        this.#CM.C = new Vector(this.#m_constraints.length);
        this.#CM.C_dot = new Vector(this.#m_constraints.length);

        this.#CM.J = new SparseMatrix(this.#m_constraints.length, 3 * this.#m_objects.length);
        this.#CM.J_dot = new SparseMatrix(this.#m_constraints.length, 3 * this.#m_objects.length);

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

        // check if lambda's length is correct
        // should only change when main.add_to_physics is active is applying its functions
        // TEMPORARY COMMENTS
        this.#CM.lambda = new Vector(this.#m_constraints.length);
        // if (this.#CM.lambda.elements.length < this.#m_constraints.length) {
        //     while (this.#CM.lambda.elements.length < this.#m_constraints.length) {
        //         this.#CM.lambda.extend(0);
        //     }
        // } else if (this.#CM.lambda.elements.length > this.#m_constraints.length) {
        //     while (this.#CM.lambda.elements.length > this.#m_constraints.length) {
        //         this.#CM.lambda.truncate();
        //     }
        // }

    }

    initConstraintManager() {
        this.#CM = new ConstraintManager();

        this.#CM.q = new Vector(3 * this.#m_objects.length);
        this.#CM.q_dot = new Vector(3 * this.#m_objects.length);
        this.#CM.Q = new Vector(3 * this.#m_objects.length);
        this.#CM.W = new Vector(3 * this.#m_objects.length);
        this.#CM.lambda = new Vector(this.#m_constraints.length);
        this.#CM.J = new SparseMatrix(this.#m_constraints.length, 3 * this.#m_objects.length);
        this.#CM.J_dot = new SparseMatrix(this.#m_constraints.length, 3 * this.#m_objects.length);
        this.#CM.accumulated_error = 0;
        this.#CM.C = new Vector(this.#m_constraints.length);
        this.#CM.C_dot = new Vector(this.#m_constraints.length);

        this.#CM.lambda = Vector.set_zero_vector(this.#CM.lambda);
    }

    updateMouse(mouse_pos, mouse_down) {
        this.#m_mouseSpring.mouse_pos = mouse_pos;

        if (mouse_down && !this.#m_mouseSpring.active) {
            if (this.#m_mouseSpring.getClosestObject(this.#m_objects, this.#m_constraints)) {
                // console.log("Mouse Spring Active");
                this.#m_mouseSpring.active = true;
            }
        }

        if (!mouse_down && this.#m_mouseSpring.active) {
            // console.log("Mouse Spring Deactivated");
            this.#m_mouseSpring.active = false;
        }

        const obj = this.#m_mouseSpring.object;
        if (this.#m_objects.indexOf(obj) == -1 && this.#m_forceGenerators.indexOf(obj) == -1 && this.#m_constraints.indexOf(obj) == -1)
            this.#m_mouseSpring.active = false;
    }

    process_sim(dt, steps = 1) {
        this.#updateSimulationConstants();

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
                // TEMPORARY
                // console.log("C length: " + this.#CM.C.elements.length);
                // console.log("lambda length: " + this.#CM.lambda.elements.length);
                this.#CM.lambda = this.#CFS.CGM(this.#CM, 96) // last int is iteration-count
                let CFS_et = performance.now();
                this.averaging.cfsdt_sum += CFS_et - CFS_st;
                this.averaging.cfsdt_frames++;
                this.averaging.check_swap_cfsdt();

                const Q_hat = SparseMatrix.matT_mult_vec(this.#CM.J, this.#CM.lambda);
                for (let i = 0; i < this.#m_objects.length; i++) {
                    const d_force = new Vector2(Q_hat.elements[3 * i], Q_hat.elements[3 * i + 1]);
                    const d_tau = Q_hat.elements[3 * i + 2];
                    this.#m_objects[i].force = Vector2.addVectors(this.#m_objects[i].force, d_force);
                    this.#m_objects[i].tau += d_tau;
                }
            }

            // integrate
            for (let i = 0; i < this.#m_objects.length; i++) {
                this.#m_objects[i].symplecticEulerTranslation(sub_dt);
            }
            for (let i = 0; i < this.#m_objects.length; i++) {
                this.#m_objects[i].symplecticEulerRotation(sub_dt);
            }
            // console.log("obj len: " + this.#m_objects.length);

            // not really part of physics_step, more for debugging and data displayed
            this.#updateDebugValues();

            this.handleBreakages();

        }
    }

    render(c) { // c is the canvas context

        let physics_id = 0;

        for (let i = 0; i < this.#m_objects.length; i++) {
            this.#m_objects[i].render(c);
        }

        for (let k = 0; k < this.#m_renderedConstraints.length; k++) {
            const con = this.#m_renderedConstraints[k];
            if (con instanceof LinkConstraint) {
                con.render(c, this.#m_objects, this.#CM.lambda.elements[physics_id], this.#lagrange_mult_limit);
                this.#m_objects[con.id1].render(c);
                this.#m_objects[con.id2].render(c);
            } else if (con instanceof FixedPosConstraint) {
                physics_id++;
            } else if (con instanceof FixedOmegaConstraint) {
                con.render(c, this.#m_objects, this.#CM.lambda.elements[physics_id]);
                this.#m_objects[con.id].render(c);
            }

            physics_id++;
        }

        for (let i = 0; i < this.#m_objects.length; i++) {
            physics_id = 0;
            // this.#m_objects[i].render(c);

            for (let k = 0; k < this.#m_renderedConstraints.length; k++) {
                const con = this.#m_renderedConstraints[k];
                if (con instanceof LinkConstraint) {
                    // con.render(c, this.#m_objects, this.#CM.lambda.elements[index_phys_const], this.#lagrange_mult_limit);
                } else if (con instanceof OffsetLinkConstraint) {
                    // con.render(c, this.#m_objects, this.#CM.lambda.elements[physics_id], this.#lagrange_mult_limit);
                } else if ((con instanceof FixedPosConstraint) && con.id == i) {
                    con.render(c, this.#m_objects, this.#CM.lambda.elements[physics_id]);
                    physics_id++;
                } else if ((con instanceof FixedXConstraint) && con.p_id == i) {
                    con.render(c, this.#m_objects, this.#CM.lambda.elements[physics_id]);
                } else if ((con instanceof FixedYConstraint) && con.p_id == i) {
                    con.render(c, this.#m_objects, this.#CM.lambda.elements[physics_id]);
                } else if ((con instanceof FixedRotationConstraint) && con.id == i) {
                    con.render(c, this.#m_objects, this.#CM.lambda.elements[physics_id]);
                }

                if (con instanceof FixedPosConstraint)
                    physics_id++;

                physics_id++;
            }

        }

        physics_id = 0;
        for (let k = 0; k < this.#m_renderedConstraints.length; k++) {
            const con = this.#m_renderedConstraints[k];
            if (con instanceof OffsetLinkConstraint) {
                con.render(c, this.#m_objects, this.#CM.lambda.elements[physics_id], this.#lagrange_mult_limit);
            } else if (con instanceof FixedPosConstraint) {
                physics_id++;
            }
            physics_id++;
        }

        // force generators
        for (let i = 0; i < this.#m_forceGenerators.length; i++) {
            this.#m_forceGenerators[i].render(c, this.#m_objects)
        }

        // this mouse spring
        if (this.#m_mouseSpring.active) {
            this.#m_mouseSpring.render(c, this.#m_objects);
        }

    }

    handleBreakages() {
        for (let i = 0; i < this.#m_constraints.length; i++) {
            const con = this.#m_constraints[i];
            const lagrange_mult = this.#CM.lambda.elements[i];
            if (con instanceof LinkConstraint || con instanceof OffsetLinkConstraint) {
                if (lagrange_mult < - this.#lagrange_mult_limit) {
                    this.removeConstraint(con);
                    // const rendered_id = this.#m_renderedConstraints.indexOf(con);'
                    this.removeRenderedConstraint(con);
                }
            }
        }

        const objects = this.#m_objects;
        const gens = this.#m_forceGenerators;
        const constraints = this.#m_constraints;
        function entityExists(obj) {
            return (objects.indexOf(obj) != -1 || gens.indexOf(obj) != -1 || constraints.indexOf(obj) != -1);
        }

        for (let i = 0; i < this.#m_forceGenerators.length; i++) {
            const gen = this.#m_forceGenerators[i];
            if (gen instanceof SpringJoint) {
                if (!entityExists(gen.state_1.entity) || !entityExists(gen.state_1.entity)) {
                    this.removeForceGenerator(gen);
                }
            }
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
        const id = this.#m_forceGenerators.indexOf(gen);
        this.#m_forceGenerators.splice(id, 1);
    }

    addConstraint(con) {
        this.#m_constraints.push(con);
    }

    removeConstraint(con) {
        const id = this.#m_constraints.indexOf(con);
        this.#m_constraints.splice(id, 1);
    }

    addRenderedConstraint(con) {
        this.#m_renderedConstraints.push(con);
    }

    removeRenderedConstraint(con) {
        const id = this.#m_renderedConstraints.indexOf(con);
        this.#m_renderedConstraints.splice(id, 1);
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

        const m_energy = this.#m_mouseSpring.getEnergyApplied(this.#m_objects);
        energy += m_energy;

        return energy;

    }

    setGravity(g) {
        this.#m_gravity = g;
    }

    setLinearDampingMU(mu) {
        this.#m_linear_damping_MU = mu;
    }

    setSpringJointStiffness(value) {
        this.#m_spring_joint_stiffness = value;
    }

    setMouseSpringStiffness(value) {
        this.#m_mouseSpring.setStiffness(value);
    }

    setOmegaConstraintValue(value) {
        this.#m_omega_constraint_value = value;
    }

    setLagrangeLimit(value) {
        this.#lagrange_mult_limit = 10 ** value;
    }

    getObjIndexContainingPos(pos) {
        for (let i = 0; i < this.#m_objects.length; i++) {
            const v = Vector2.subtractVectors(this.#m_objects[i].pos, pos);
            const dist2 = v.x * v.x + v.y * v.y;
            const rad = this.#m_objects[i].radius;
            if (dist2 < rad * rad)
                return i;
        }
        return -1;
    }

    getDynamicObjectsLength() {
        return this.#m_objects.length;
    }

    getForceGeneratorsLength() {
        return this.#m_forceGenerators.length;
    }

    getConstraintLength() {
        return this.#m_constraints.length;
    }

    getRenderedConstraintLength() {
        return this.#m_renderedConstraints.length;
    }

    removeByIdDynamicObject(id) {
        this.#m_objects.splice(id, 1);
    }

    removeByIdForceGenerator(id) {
        this.#m_forceGenerators.splice(id, 1);
    }

    removeByIdConstraint(id) {
        // TODO
    }

    removeLastConstraint() {
        const constraint = this.#m_renderedConstraints.pop();
        if (constraint instanceof FixedPosConstraint) {
            this.#m_constraints.pop();
            this.#m_constraints.pop();
        } else {
            this.#m_constraints.pop();
        }
    }

    addSpringJointWithStates(state_1, state_2) {
        const gen = new SpringJoint(state_1, state_2, this.#m_objects);
        this.addForceGenerator(gen);
    }

    addSpringJoint(id1, id2) {
        const obj1 = this.#m_objects[id1];
        const obj2 = this.#m_objects[id2];
        const state_1 = { entity: obj1, offset: Vector2.zero, prev_theta: obj1.theta, t_param: null, applied_pos: obj1.pos };
        const state_2 = { entity: obj2, offset: Vector2.zero, prev_theta: obj2.theta, t_param: null, applied_pos: obj2.pos };
        const gen = new SpringJoint(state_1, state_2, this.#m_objects);
        this.addForceGenerator(gen);
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

    addLinkConstraint(id1, id2) {
        const dist = Vector2.distance(this.#m_objects[id1].pos, this.#m_objects[id2].pos);
        const con = new LinkConstraint(id1, id2, dist)
        this.addConstraint(con);
        this.addRenderedConstraint(con);
    }

    addOffsetLinkConstraint(state_1, state_2) {
        const con = new OffsetLinkConstraint(state_1, state_2, this.#m_objects);
        this.addConstraint(con);
        this.addRenderedConstraint(con);
    }

    addFixedRotConstraint(id) {
        const con = new FixedRotationConstraint(id, this.#m_objects[id].theta);
        this.addConstraint(con);
        this.addRenderedConstraint(con);
    }

    addFixedOmegaConstraint(id, vel) {
        const con = new FixedOmegaConstraint(id, this.#m_objects[id].theta, vel);
        this.addConstraint(con);
        this.addRenderedConstraint(con);
    }

    getObjectPositionById(id) {
        return this.#m_objects[id].pos;
    }

    getClosestEntity(pos) {
        // dynamicObject
        for (let i = 0; i < this.#m_objects.length; i++) {
            const v = Vector2.subtractVectors(this.#m_objects[i].pos, pos);
            const dist2 = v.sqr_magnitude();
            const rad = this.#m_objects[i].radius;
            if (dist2 < rad * rad) {
                const offset = Vector2.subtractVectors(pos, this.#m_objects[i].pos);
                return { entity: this.#m_objects[i], offset: offset, prev_theta: this.#m_objects[i].theta, t_param: null, applied_pos: pos };
            }
        }

        // constraints
        for (let i = 0; i < this.#m_constraints.length; i++) {
            const con = this.#m_constraints[i];
            if (con instanceof LinkConstraint) {
                const A = this.#m_objects[con.id1];
                const B = this.#m_objects[con.id2];
                const C_B = Vector2.subtractVectors(pos, B.pos);
                const A_B = Vector2.subtractVectors(A.pos, B.pos);
                const t = Math.max(Math.min(C_B.dot(A_B) / A_B.sqr_magnitude(), 1), 0);
                const proj = Vector2.addVectors(B.pos, Vector2.scaleVector(A_B, t));
                const dist2 = Vector2.subtractVectors(proj, pos).sqr_magnitude();
                const sim_line_width = Units.scale_c_s * LineWidths.OUTER_LINKCONSTRAINT;
                if (dist2 < sim_line_width * sim_line_width) {
                    return { entity: con, offset: null, prev_theta: null, t_param: t, applied_pos: proj };
                }
            }
        }

        return { entity: null, offset: null, prev_theta: null, t_param: null, applied_pos: null };

    }



}