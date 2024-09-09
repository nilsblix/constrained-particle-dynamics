import {Vector2, Vector, SparseMatrix, SparseMatrixBlock} from "./linear_algebra.js";

export class ConstraintManager {
    #k_s = 300;
    #k_d = 40;
    constructor() {
        this.q = new Vector(0);
        this.q_dot = new Vector(0);
        this.Q = new Vector(0);
        this.W = new Vector(0);
        this.lambda = new Vector(0);
        this.J = new SparseMatrix(0, 0);
        this.J_dot = new SparseMatrix(0, 0);

        this.accumulated_error = 0;
        this.C = new Vector(0);
        this.C_dot = new Vector(0);

        this.b = new Vector(0);
        this.A = new SparseMatrix(0, 0);
    }

    // A(x) { // return JWJ_T * x
    //     let Ax = SparseMatrix.matT_mult_vec(this.J, x);
    //     Ax = Vector.elem_by_elem_mult_vec(this.W, Ax);
    //     Ax = SparseMatrix.mat_mult_vec(this.J, Ax);
    //     return Ax;
    // }

    getA() {
        const W_matrix = new SparseMatrix(this.q.elements.length, this.q.elements.length);
        for (let i = 0; i < this.q.elements.length; i++) {
            W_matrix.elements.push(new SparseMatrixBlock(i, i, this.W.elements[i]));
        }
        let A = SparseMatrix.mat_mult(W_matrix, SparseMatrix.transpose(this.J));
        A = SparseMatrix.mat_mult(this.J, A);
        return A;
    }

    getB() { // return - J_dot * Q_dot -  JWQ - k_s * C - k_d * C
        const first = Vector.get_negated(SparseMatrix.mat_mult_vec(this.J_dot, this.q_dot));
        const second = SparseMatrix.mat_mult_vec(this.J, Vector.elem_by_elem_mult_vec(this.W, this.Q));
        const third = Vector.scale_vector(this.#k_s, this.C);
        const fourth = Vector.scale_vector(this.#k_d, this.C_dot);

        const one_two = Vector.sub_vectors(first, second);
        const three_four = (Vector.add_vectors(third, fourth));
        return Vector.sub_vectors(one_two, three_four);
    }
}

export class ConstraintForceSolver {
    #m_minError = 1e-4;
    #upped_iteration_count = 512;

    constructor() {}

    CGM(CM, iterations = 64) {
        let max_iter = iterations;

        /*
            TODO:
                shits not working, check console
                definition of this jacobi easy preconditioner:
                    Ax = b
                    M = diag(A)
                    solve for M^-1 A x = M^-1 b
                    instaed of A x = b for better convergence
                not sure this is going to work but might aswell try as it is not a huge implementation

        */

        // preconditioners
        const m_inv = Vector.inv(CM.A.get_diagonal());

        function pre_A(x) {
            return Vector.elem_by_elem_mult_vec(m_inv, SparseMatrix.mat_mult_vec(CM.A, x));
        }

        const pre_b = Vector.elem_by_elem_mult_vec(m_inv, CM.b);

        function pre_b_minus_pre_A(x) {
            return Vector.sub_vectors(pre_b, pre_A(x));
        }
        // pre-end

        let x = CM.lambda;
        let r = pre_b_minus_pre_A(x);
        let rk_mag2 = r.sqr_magnitude();

        if (rk_mag2 < this.#m_minError * this.#m_minError) {
            console.log("Premature lambda found on iteration: 0 (before iterating)");
            return x;
        } else if (Math.sqrt(rk_mag2) > 1) {
            console.log("Upped CGM iteration count due to too high residual squared mag: " + Math.sqrt(rk_mag2));
            max_iter = this.#upped_iteration_count;
        }

        let p = r;

        for (let k = 0; k < max_iter; k++) {
            let m_inv_Ap = pre_A(p);

            const alpha = rk_mag2 / Vector.dot(p, m_inv_Ap); // p dot Ap is such that p is conjugate to itself wrt A
            x = Vector.add_vectors(x, Vector.scale_vector(alpha, p));
            r = pre_b_minus_pre_A(x);

            const rk1_mag2 = r.sqr_magnitude();

            if (rk1_mag2 < this.#m_minError * this.#m_minError) {
                console.log("Premature lambda found on iteration: " + (k + 1) + " (while iterating)");
                break;
            }

            const beta = rk1_mag2 / rk_mag2;
            p = Vector.add_vectors(r, Vector.scale_vector(beta, p));
            rk_mag2 = rk1_mag2;

            if (k == max_iter - 1)
                console.warn("CFS: All iterations were used, ==> not converging");

        }

        let error_mag = (pre_b_minus_pre_A(x)).magnitude();
        CM.accumulated_error += error_mag;

        return x;

    }
}