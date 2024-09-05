import {Vector2, Vector, SparseMatrix, SparseMatrixBlock} from "./linear_algebra.js";

export class ConstraintManager {
    #k_s = 300;
    #k_d = 40;
    constructor() {
        this.q = new Vector();
        this.q_dot = new Vector();
        this.Q = new Vector();
        this.W = new Vector();
        this.lambda = new Vector();
        this.J = new SparseMatrix();
        this.J_dot = new SparseMatrix();

        this.accumulated_error = 0;
        this.C = new Vector();
        this.C_dot = new Vector();

        this.b = new Vector();
    }

    A(x) {
        let Ax = SparseMatrix.matT_mult_vec(this.J, x);
        Ax = SparseMatrix.mat_mult_vec(this.W, Ax);
        Ax = SparseMatrix.mat_mult_vec(this.J, Ax);
        return Ax;
    }

    getB() {
        let first = (SparseMatrix.mat_mult_vec(this.J_dot, q_dot)).negate();
        let second = SparseMatrix.mat_mult_vec(this.J, SparseMatrix.mat_mult_vec(W, Q));
        let third = Vector.scale_vector(k_s, this.C);
        let fourth = Vector.scale_vector(k_d, this.C_dot);

        let one_two = Vector.sub_vectors(first, second);
        let three_four = Vector.add(third, fourth);
        return Vector.sub_vectors(one_two, three_four);
    }
}

export class ConstraintForceSolver {
    static m_minError = 1e-7;
    #upped_iteration_count = 128;

    constructor() {}

    CGM(CM, iterations = 64) {
        let n = CM.b.elements.length;
        let max_iter = iterations;

        let x = CM.lambda;
        let r = Vector.sub_vectors(CH.b, CH.A(x));
        let rk_mag2 = r.sqr_magnitude();

        if (rk_mag2 < m_minError) {
            console.log("Premature lambda found on iteration: 0 (before iterating)");
            return x;
        } else if (rk_mag2 > 1) {
            console.log("Upped CGM iteration count due to too high residual squared mag");
            max_iter = this.#upped_iteration_count;
        }

        let p = r;

        for (let k = 0; k < max_iter; k++) {
            let Ap = CM.A(p);

            const alpha = rk_mag2 / Vector.dot(p, Ap); // p dot Ap is such that p is conjugate to itself wrt A
            x = Vector.add_vectors(x, Vector.scale_vector(alpha, p));
            r = Vector.sub_vectors(CH.b, CH.A(x));

            const rk1_mag2 = r.sqr_magnitude();

            if (rk1_mag2 < m_minError) {
                console.log("Premature lambda found on iteration: " + k + " (while iterating)");
                break;
            }

            const beta = rk1_mag2 / rk_mag2;
            p = Vector.add_vectors(r, Vector.scale_vector(beta, p));
            rk_mag2 = rk1_mag2;
        }

        let error_mag = (Vector.sub_vectors(CM.b, CM.A(x))).magnitude();
        CM.accumulated_error += error_mag;

        return x;

    }
}