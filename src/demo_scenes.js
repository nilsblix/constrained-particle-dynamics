import {Units} from "./units.js";
import {DynamicObject} from "./dynamicObject.js";
import {Vector2} from "./linear_algebra.js";
import { editor } from "./editor.js";

/*
    Things to know if one where to build a new demo scene
    --> Nothing. Everything gets automatically picked up by the pipeline
*/

export function setupScene(physicsState, solver, editor, version) {
    switch (version) {
        case "null":
            break;

        case "standard pratt truss":
            const pratt_truss_radius_2 = 0.05;
            const height_pratt_truss_2 = Units.HEIGHT / 2;

            const pratt_p_delta_2 = 0.6;

            const a0 = new DynamicObject(new Vector2(2, height_pratt_truss_2), 1, pratt_truss_radius_2);
            const a1 = new DynamicObject(new Vector2(3, height_pratt_truss_2), 1, pratt_truss_radius_2);
            const a2 = new DynamicObject(new Vector2(3, 1 + height_pratt_truss_2), 1, pratt_truss_radius_2);
            const a3 = new DynamicObject(new Vector2(4, height_pratt_truss_2), 1, pratt_truss_radius_2);
            const a4 = new DynamicObject(new Vector2(4, 1 + height_pratt_truss_2), 1, pratt_truss_radius_2);
            const a5 = new DynamicObject(new Vector2(5, height_pratt_truss_2), 1, pratt_truss_radius_2);
            const a6 = new DynamicObject(new Vector2(5, 1 + height_pratt_truss_2), 1, pratt_truss_radius_2);
            const a7 = new DynamicObject(new Vector2(6, height_pratt_truss_2), 1, pratt_truss_radius_2);
            const a8 = new DynamicObject(new Vector2(6, 1 + height_pratt_truss_2), 1, pratt_truss_radius_2);
            const a9 = new DynamicObject(new Vector2(7, height_pratt_truss_2), 1, pratt_truss_radius_2);
            const a10 = new DynamicObject(new Vector2(7, 1 + height_pratt_truss_2), 1, pratt_truss_radius_2);
            const a11 = new DynamicObject(new Vector2(8, height_pratt_truss_2), 1, pratt_truss_radius_2)
            
            physicsState.addObject(a0);
            physicsState.addObject(a1);
            physicsState.addObject(a2);
            physicsState.addObject(a3);
            physicsState.addObject(a4);
            physicsState.addObject(a5);
            physicsState.addObject(a6);
            physicsState.addObject(a7);
            physicsState.addObject(a8);
            physicsState.addObject(a9);
            physicsState.addObject(a10);
            physicsState.addObject(a11);

            physicsState.addFixedPosConstraint(0);
            physicsState.addFixedYConstraint(11);

            physicsState.addLinkConstraint(0, 2);
            physicsState.addLinkConstraint(2, 4);
            physicsState.addLinkConstraint(4, 6);
            physicsState.addLinkConstraint(6, 8);
            physicsState.addLinkConstraint(8, 10);
            physicsState.addLinkConstraint(10, 11);
            physicsState.addLinkConstraint(2, 3);
            physicsState.addLinkConstraint(4, 5);
            physicsState.addLinkConstraint(5, 8);
            physicsState.addLinkConstraint(7, 10);
            
            physicsState.addLinkConstraint(1, 2);
            physicsState.addLinkConstraint(3, 4);
            physicsState.addLinkConstraint(5, 6);
            physicsState.addLinkConstraint(7, 8);
            physicsState.addLinkConstraint(9, 10);
            physicsState.addLinkConstraint(0, 1);
            physicsState.addLinkConstraint(1, 3);
            physicsState.addLinkConstraint(3, 5);
            physicsState.addLinkConstraint(5, 7);
            physicsState.addLinkConstraint(7, 9);
            physicsState.addLinkConstraint(9, 11);

            break;

        case "pratt truss":

            const pratt_truss_radius = 0.05;
            const height_pratt_truss = Units.HEIGHT / 2;

            const pratt_p_delta = 0.6;

            const t0 = new DynamicObject(new Vector2(2, height_pratt_truss), 1, pratt_truss_radius);
            const t1 = new DynamicObject(new Vector2(3, height_pratt_truss), 1, pratt_truss_radius);
            const t2 = new DynamicObject(new Vector2(3, 1 + height_pratt_truss), 1, pratt_truss_radius);
            const t3 = new DynamicObject(new Vector2(4, height_pratt_truss), 1, pratt_truss_radius);
            const t4 = new DynamicObject(new Vector2(4, 1 + height_pratt_truss), 1, pratt_truss_radius);
            const t5 = new DynamicObject(new Vector2(5, height_pratt_truss), 1, pratt_truss_radius);
            const t6 = new DynamicObject(new Vector2(5, 1 + height_pratt_truss), 1, pratt_truss_radius);
            const t7 = new DynamicObject(new Vector2(6, height_pratt_truss), 1, pratt_truss_radius);
            const t8 = new DynamicObject(new Vector2(6, 1 + height_pratt_truss), 1, pratt_truss_radius);
            const t9 = new DynamicObject(new Vector2(7, height_pratt_truss), 1, pratt_truss_radius);
            const t10 = new DynamicObject(new Vector2(7, 1 + height_pratt_truss), 1, pratt_truss_radius);
            const t11 = new DynamicObject(new Vector2(8, height_pratt_truss), 1, pratt_truss_radius);
            
            const t12 = new DynamicObject(new Vector2(2, 2+height_pratt_truss), 1, 2*pratt_truss_radius);
            const t13 = new DynamicObject(new Vector2(8, 2+height_pratt_truss), 5, 3*pratt_truss_radius);

            const t14 = new DynamicObject(new Vector2(2, 1+height_pratt_truss), 1, pratt_truss_radius);
            const t15 = new DynamicObject(new Vector2(8, 1+height_pratt_truss), 1, pratt_truss_radius);

            const t16 = new DynamicObject(new Vector2(5, -pratt_p_delta+height_pratt_truss), 1, 1*pratt_truss_radius);
            const t17 = new DynamicObject(new Vector2(5, -2*pratt_p_delta+height_pratt_truss), 5, 2.5*pratt_truss_radius);

            physicsState.addObject(t0);
            physicsState.addObject(t1);
            physicsState.addObject(t2);
            physicsState.addObject(t3);
            physicsState.addObject(t4);
            physicsState.addObject(t5);
            physicsState.addObject(t6);
            physicsState.addObject(t7);
            physicsState.addObject(t8);
            physicsState.addObject(t9);
            physicsState.addObject(t10);
            physicsState.addObject(t11);

            physicsState.addObject(t12);
            physicsState.addObject(t13);

            physicsState.addObject(t14);
            physicsState.addObject(t15);

            physicsState.addObject(t16);
            physicsState.addObject(t17);

            physicsState.addFixedPosConstraint(12);
            physicsState.addFixedYConstraint(13);

            physicsState.addLinkConstraint(0, 2);
            physicsState.addLinkConstraint(2, 4);
            physicsState.addLinkConstraint(4, 6);
            physicsState.addLinkConstraint(6, 8);
            physicsState.addLinkConstraint(8, 10);
            physicsState.addLinkConstraint(10, 11);
            physicsState.addLinkConstraint(2, 3);
            physicsState.addLinkConstraint(4, 5);
            physicsState.addLinkConstraint(5, 8);
            physicsState.addLinkConstraint(7, 10);
            
            physicsState.addLinkConstraint(1, 2);
            physicsState.addLinkConstraint(3, 4);
            physicsState.addLinkConstraint(5, 6);
            physicsState.addLinkConstraint(7, 8);
            physicsState.addLinkConstraint(9, 10);
            physicsState.addLinkConstraint(0, 1);
            physicsState.addLinkConstraint(1, 3);
            physicsState.addLinkConstraint(3, 5);
            physicsState.addLinkConstraint(5, 7);
            physicsState.addLinkConstraint(7, 9);
            physicsState.addLinkConstraint(9, 11);

            // links
            // physicsState.addLinkConstraint(0, 14);
            physicsState.addLinkConstraint(12, 14);
            // physicsState.addLinkConstraint(11, 15);
            physicsState.addLinkConstraint(15, 13);
            // springs
            physicsState.addSpringJoint(0, 14);
            // physicsState.addSpringJoint(12, 14);
            physicsState.addSpringJoint(11, 15);
            // physicsState.addSpringJoint(15, 13);

            physicsState.addLinkConstraint(5, 16);
            physicsState.addLinkConstraint(16, 17);


            break;

        case "king post truss":

            const kp_truss_height = Units.HEIGHT / 2;
            const kp_truss_radius = 0.07;
            let delta_h = 0.8;

            const kp0 = new DynamicObject(new Vector2(3.5, kp_truss_height), 1);
            const kp1 = new DynamicObject(new Vector2(4.25, kp_truss_height), 1);
            const kp2 = new DynamicObject(new Vector2(5, kp_truss_height), 1);
            const kp3 = new DynamicObject(new Vector2(5.75, kp_truss_height), 1);
            const kp4 = new DynamicObject(new Vector2(6.5, kp_truss_height), 1);
            const kp5 = new DynamicObject(new Vector2(4.25, delta_h+kp_truss_height), 1);
            const kp6 = new DynamicObject(new Vector2(5.75, delta_h+kp_truss_height), 1);
            const kp7 = new DynamicObject(new Vector2(5, 2*delta_h+kp_truss_height), 1);

            const kp8 = new DynamicObject(new Vector2(5, -0.5 + kp_truss_height), 1);
            const kp9 = new DynamicObject(new Vector2(5, -1 + kp_truss_height), 1);
            const kp10 = new DynamicObject(new Vector2(5, -1.5 + kp_truss_height), 10, 0.15 + kp_truss_radius);


            physicsState.addObject(kp0);
            physicsState.addObject(kp1);
            physicsState.addObject(kp2);
            physicsState.addObject(kp3);
            physicsState.addObject(kp4);
            physicsState.addObject(kp5);
            physicsState.addObject(kp6);
            physicsState.addObject(kp7);

            physicsState.addObject(kp8);
            physicsState.addObject(kp9);
            physicsState.addObject(kp10);


            // physicsState.addConstraint(new FixedXConstraint(0, kp0.pos.x));
            // physicsState.addConstraint(new FixedYConstraint(0, kp0.pos.y));
            physicsState.addFixedPosConstraint(0);
            // physicsState.addFixedPosConstraint(0);
            physicsState.addFixedYConstraint(4);

            physicsState.addLinkConstraint(0, 1);
            physicsState.addLinkConstraint(1, 2);
            physicsState.addLinkConstraint(2, 3);
            physicsState.addLinkConstraint(3, 4);
            physicsState.addLinkConstraint(0, 5);
            physicsState.addLinkConstraint(1, 5);
            physicsState.addLinkConstraint(2, 5);
            physicsState.addLinkConstraint(2, 6);
            physicsState.addLinkConstraint(3, 6);
            physicsState.addLinkConstraint(4, 6);
            physicsState.addLinkConstraint(5, 7);
            physicsState.addLinkConstraint(2, 7);
            physicsState.addLinkConstraint(6, 7);

            physicsState.addLinkConstraint(2, 8);
            physicsState.addLinkConstraint(8, 9);
            physicsState.addLinkConstraint(9, 10);
            






            break;
        
        case "large bridge structure":

            const lbs_radius = 0.05;

            const p = [
            new DynamicObject(new Vector2(3.4, 1),   1, lbs_radius),
            new DynamicObject(new Vector2(3, 1.6),   1, lbs_radius),
            new DynamicObject(new Vector2(3.8, 1.6), 1, lbs_radius),
            new DynamicObject(new Vector2(3, 2.4),   1, lbs_radius),
            new DynamicObject(new Vector2(3.8, 2.4),   1, lbs_radius),
            new DynamicObject(new Vector2(3, 3.2),   1, lbs_radius),
            new DynamicObject(new Vector2(3.8, 3.2),   1, lbs_radius),
            new DynamicObject(new Vector2(3, 4),   1, lbs_radius),
            new DynamicObject(new Vector2(3.8, 4),   1, lbs_radius),
            new DynamicObject(new Vector2(4.6, 3.2),   1, lbs_radius),
            new DynamicObject(new Vector2(4.6, 4),   1, lbs_radius),
            new DynamicObject(new Vector2(5.4, 3.2),   1, lbs_radius),
            new DynamicObject(new Vector2(5.4, 4),   1, lbs_radius),
            new DynamicObject(new Vector2(6.2, 3.2),   1, lbs_radius),
            new DynamicObject(new Vector2(6.2, 4),   1, lbs_radius),
            new DynamicObject(new Vector2(7, 3.2),   1, lbs_radius),
            new DynamicObject(new Vector2(7, 4),   1, lbs_radius),
            new DynamicObject(new Vector2(7, 2.4),   1, lbs_radius),
            new DynamicObject(new Vector2(6.2, 2.4),   1, lbs_radius),
            new DynamicObject(new Vector2(7, 1.6),   1, lbs_radius),
            new DynamicObject(new Vector2(6.2, 1.6),   1, lbs_radius),
            new DynamicObject(new Vector2(6.6, 1),   1, lbs_radius),
            new DynamicObject(new Vector2(3.4, 4.8),   1, lbs_radius),
            new DynamicObject(new Vector2(6.6, 4.8),   1, lbs_radius),
            new DynamicObject(new Vector2(1.6, 3.6),   1, lbs_radius),
            new DynamicObject(new Vector2(1.6, 3.2),   1, lbs_radius),
            new DynamicObject(new Vector2(1.6, 2.8),   1, lbs_radius),
            new DynamicObject(new Vector2(1.6, 2.4),   10, 4*lbs_radius)
            ]    

            for (let i = 0; i < p.length; i++) {
                physicsState.addObject(p[i]);
            }

            physicsState.addFixedPosConstraint(0);
            physicsState.addFixedYConstraint(21);

            physicsState.addLinkConstraint(0, 1);
            physicsState.addLinkConstraint(0, 2);
            physicsState.addLinkConstraint(1, 2);
            //
            physicsState.addLinkConstraint(1, 3);
            physicsState.addLinkConstraint(1, 4);
            physicsState.addLinkConstraint(2, 3);
            physicsState.addLinkConstraint(2, 4);
            physicsState.addLinkConstraint(3, 4);
            //
            physicsState.addLinkConstraint(3, 5);
            physicsState.addLinkConstraint(3, 6);
            physicsState.addLinkConstraint(4, 5);
            physicsState.addLinkConstraint(4, 6);
            physicsState.addLinkConstraint(5, 6);
            //
            physicsState.addLinkConstraint(5, 7);
            physicsState.addLinkConstraint(5, 8);
            physicsState.addLinkConstraint(6, 7);
            physicsState.addLinkConstraint(6, 8);
            physicsState.addLinkConstraint(7, 8);
            //
            physicsState.addLinkConstraint(6, 10);
            physicsState.addLinkConstraint(6, 9);
            physicsState.addLinkConstraint(8, 9);
            physicsState.addLinkConstraint(8, 10);
            physicsState.addLinkConstraint(9, 10);
            //
            physicsState.addLinkConstraint(9, 11);
            physicsState.addLinkConstraint(9, 12);
            physicsState.addLinkConstraint(10, 11);
            physicsState.addLinkConstraint(10, 12);
            physicsState.addLinkConstraint(11, 12);
            //
            physicsState.addLinkConstraint(11, 13);
            physicsState.addLinkConstraint(11, 14);
            physicsState.addLinkConstraint(12, 13);
            physicsState.addLinkConstraint(12, 14);
            physicsState.addLinkConstraint(13, 14);
            //
            physicsState.addLinkConstraint(13, 15);
            physicsState.addLinkConstraint(13, 16);
            physicsState.addLinkConstraint(14, 15);
            physicsState.addLinkConstraint(14, 16);
            physicsState.addLinkConstraint(15, 16);
            //
            physicsState.addLinkConstraint(17, 13);
            physicsState.addLinkConstraint(17, 15);
            physicsState.addLinkConstraint(18, 13);
            physicsState.addLinkConstraint(18, 15);
            physicsState.addLinkConstraint(17, 18);
            //
            physicsState.addLinkConstraint(19, 17);
            physicsState.addLinkConstraint(19, 18);
            physicsState.addLinkConstraint(20, 17);
            physicsState.addLinkConstraint(20, 18);
            physicsState.addLinkConstraint(19, 20);
            //
            physicsState.addLinkConstraint(19, 21);
            physicsState.addLinkConstraint(20, 21);
            //
            physicsState.addLinkConstraint(7, 22);
            physicsState.addLinkConstraint(8, 22);
            //
            physicsState.addLinkConstraint(14, 23);
            physicsState.addLinkConstraint(16, 23);
            //
            physicsState.addLinkConstraint(5, 24);
            physicsState.addLinkConstraint(7, 24);
            //
            physicsState.addLinkConstraint(24, 25);
            physicsState.addLinkConstraint(25, 26);
            physicsState.addLinkConstraint(26, 27);

            // physicsState.addFixedRotConstraint(27);
            // physicsState.addFixedOmegaConstraint(27, 0.3);



            break;
    
        case "crane structure" :
            const c0 = new DynamicObject(new Vector2(3, 0.5), 1, editor.standard_object_radius);
            const c1 = new DynamicObject(new Vector2(4, 0.5), 1, editor.standard_object_radius);
            const c2 = new DynamicObject(new Vector2(3, 1.5), 1, editor.standard_object_radius);
            const c3 = new DynamicObject(new Vector2(4, 1.5), 1, editor.standard_object_radius);
            const c4 = new DynamicObject(new Vector2(3, 2.5), 1, editor.standard_object_radius);
            const c5 = new DynamicObject(new Vector2(4, 2.5), 1, editor.standard_object_radius);
            const c6 = new DynamicObject(new Vector2(3, 3.5), 1, editor.standard_object_radius);
            const c7 = new DynamicObject(new Vector2(4, 3.5), 1, editor.standard_object_radius);
            const c8 = new DynamicObject(new Vector2(3, 4.5), 1, editor.standard_object_radius);
            const c9 = new DynamicObject(new Vector2(4, 4.5), 1, editor.standard_object_radius);
            const c10 = new DynamicObject(new Vector2(5, 3.5), 1, editor.standard_object_radius);
            const c11 = new DynamicObject(new Vector2(6, 3.5), 1, editor.standard_object_radius);
            const c12 = new DynamicObject(new Vector2(7, 3.5), 1, editor.standard_object_radius);
            const c13 = new DynamicObject(new Vector2(7, 4), 1, editor.standard_object_radius);
            const c14 = new DynamicObject(new Vector2(6, 4 + 1/6), 1, editor.standard_object_radius);
            const c15 = new DynamicObject(new Vector2(5, 4 + 1/3), 1, editor.standard_object_radius);
            const c16 = new DynamicObject(new Vector2(2, 3.5), 1, editor.standard_object_radius);
            const c17 = new DynamicObject(new Vector2(7, 3.2), 1, editor.standard_object_radius);
            const c18 = new DynamicObject(new Vector2(7, 2.9), 1, editor.standard_object_radius);
            const c19 = new DynamicObject(new Vector2(7, 2.6), 5, 3*editor.standard_object_radius);

            physicsState.addObject(c0);
            physicsState.addObject(c1);
            physicsState.addObject(c2);
            physicsState.addObject(c3);
            physicsState.addObject(c4);
            physicsState.addObject(c5);
            physicsState.addObject(c6);
            physicsState.addObject(c7);
            physicsState.addObject(c8);
            physicsState.addObject(c9);
            physicsState.addObject(c10);
            physicsState.addObject(c11);
            physicsState.addObject(c12);
            physicsState.addObject(c13);
            physicsState.addObject(c14);
            physicsState.addObject(c15);
            physicsState.addObject(c16);
            physicsState.addObject(c17);
            physicsState.addObject(c18);
            physicsState.addObject(c19);

            physicsState.addFixedPosConstraint(0);
            physicsState.addFixedYConstraint(1);
            // physicsState.addFixedPosConstraint(16);

            //
            physicsState.addLinkConstraint(0, 1);
            //
            physicsState.addLinkConstraint(0, 2);
            physicsState.addLinkConstraint(1, 3);
            physicsState.addLinkConstraint(0, 3);
            physicsState.addLinkConstraint(2, 3);
            //
            physicsState.addLinkConstraint(2, 4);
            physicsState.addLinkConstraint(3, 5);
            physicsState.addLinkConstraint(3, 4);
            physicsState.addLinkConstraint(4, 5);
            //
            physicsState.addLinkConstraint(4, 6);
            physicsState.addLinkConstraint(5, 7);
            physicsState.addLinkConstraint(4, 7);
            physicsState.addLinkConstraint(6, 7);
            //
            physicsState.addLinkConstraint(6, 8);
            physicsState.addLinkConstraint(7, 9);
            physicsState.addLinkConstraint(7, 8);
            physicsState.addLinkConstraint(8, 9);
            // outer line on attachment
            physicsState.addLinkConstraint(7, 10);
            physicsState.addLinkConstraint(10, 11);
            physicsState.addLinkConstraint(11, 12);
            physicsState.addLinkConstraint(12, 13); // is vertical, also outer line
            physicsState.addLinkConstraint(13, 14);
            physicsState.addLinkConstraint(14, 15);
            physicsState.addLinkConstraint(9, 15);
            // inner diagonals
            physicsState.addLinkConstraint(7, 15);
            physicsState.addLinkConstraint(11, 15);
            physicsState.addLinkConstraint(11, 13);
            // inner verticals
            physicsState.addLinkConstraint(10, 15);
            physicsState.addLinkConstraint(11, 14);
            // left triangle
            physicsState.addLinkConstraint(6, 16);
            physicsState.addLinkConstraint(8, 16);
            // right, weight
            physicsState.addLinkConstraint(12, 17);
            physicsState.addLinkConstraint(17, 18);
            physicsState.addLinkConstraint(18, 19);
            // left, weight

            break;
    
        case "rotating crane structure":

            const o_s = [
            new DynamicObject(new Vector2(5, 0.5), 1, editor.standard_object_radius), // 0
            new DynamicObject(new Vector2(5, 1.5), 1, editor.standard_object_radius), // 1
            new DynamicObject(new Vector2(5, 2.5), 1, editor.standard_object_radius), // 2
            new DynamicObject(new Vector2(5, 3.5), 1, editor.standard_object_radius), // 3
            new DynamicObject(new Vector2(5, 4.5), 1, editor.standard_object_radius), // 4
            new DynamicObject(new Vector2(6, 3.5), 1, editor.standard_object_radius), // 5
            new DynamicObject(new Vector2(7, 3.5), 1, editor.standard_object_radius), // 6
            new DynamicObject(new Vector2(8, 3.5), 1, editor.standard_object_radius), // 7
            new DynamicObject(new Vector2(7, 3.5 + 1/3), 1, editor.standard_object_radius), // 8
            new DynamicObject(new Vector2(6, 4 + 1/6), 1, editor.standard_object_radius), // 9
            new DynamicObject(new Vector2(4.5, 1.5), 1, editor.standard_object_radius), // 10
            new DynamicObject(new Vector2(4, 2.5), 1, editor.standard_object_radius), // 11
            new DynamicObject(new Vector2(3.5, 3.5), 1, editor.standard_object_radius), // 12
            new DynamicObject(new Vector2(0.5, 0.5), 1, editor.standard_object_radius), // 13
            new DynamicObject(new Vector2(1.1, 1.1), 1, editor.standard_object_radius), // 14
            new DynamicObject(new Vector2(1.7, 1.7), 1, editor.standard_object_radius), // 15
            new DynamicObject(new Vector2(2.3, 2.3), 1, editor.standard_object_radius), // 16
            new DynamicObject(new Vector2(8, 3), 1, editor.standard_object_radius), // 17
            new DynamicObject(new Vector2(8, 2.5), 1, editor.standard_object_radius), // 18
            new DynamicObject(new Vector2(8, 2), 25, 4.5 * editor.standard_object_radius), // 19
            new DynamicObject(new Vector2(4.25, 4), 1, editor.standard_object_radius), // 20
            new DynamicObject(new Vector2(2.9, 2.9), 1, editor.standard_object_radius), // 21
            ]

            for (let i = 0; i < o_s.length; i++) {
                physicsState.addObject(o_s[i]);
            }

            physicsState.addFixedPosConstraint(0);
            physicsState.addFixedPosConstraint(13);
            
            // main vertical line
            physicsState.addLinkConstraint(0, 1);
            physicsState.addLinkConstraint(1, 2);
            physicsState.addLinkConstraint(2, 3);
            physicsState.addLinkConstraint(3, 4);
            // outline crane part
            physicsState.addLinkConstraint(3, 5);
            physicsState.addLinkConstraint(5, 6);
            physicsState.addLinkConstraint(6, 7);
            physicsState.addLinkConstraint(7, 8);
            physicsState.addLinkConstraint(8, 9);
            physicsState.addLinkConstraint(9, 4);
            // diagonals crane part
            physicsState.addLinkConstraint(3, 9);
            physicsState.addLinkConstraint(6, 9);
            physicsState.addLinkConstraint(6, 8);
            physicsState.addLinkConstraint(5, 9);
            // left crane body
            physicsState.addLinkConstraint(0, 10);
            physicsState.addLinkConstraint(10, 11);
            physicsState.addLinkConstraint(11, 12);
            physicsState.addLinkConstraint(12, 20);
            physicsState.addLinkConstraint(20, 4);
            // left body diagonals
            physicsState.addLinkConstraint(1, 10);
            physicsState.addLinkConstraint(1, 11);
            physicsState.addLinkConstraint(2, 11);
            physicsState.addLinkConstraint(2, 12);
            physicsState.addLinkConstraint(3, 12);
            physicsState.addLinkConstraint(3, 20);
            // crane support
            physicsState.addLinkConstraint(13, 14);
            physicsState.addLinkConstraint(14, 15);
            physicsState.addLinkConstraint(15, 16);
            physicsState.addLinkConstraint(16, 21);
            physicsState.addLinkConstraint(21, 12);
            // chain
            physicsState.addLinkConstraint(7, 17);
            physicsState.addLinkConstraint(17, 18);
            physicsState.addLinkConstraint(18, 19);







            break;
    
    }

    solver.physicsState_is_null = false;
    solver.simulating = false;

}