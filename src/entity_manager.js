import {Units} from "./units.js";
import {DynamicObject} from "./dynamicObject.js";
import {Vector2} from "./linear_algebra.js";
import {CubicBezier} from "./bezier_curve.js";
import {Colours, LineWidths, Extras} from "./render_settings.js";

export const entity_manager = {
    // honestly update this shit. it is definately as fucked as can be. plz it looks SO ass.
    active: false,
    snap_to_grid: false,
    drawing_link_constraint: false,
    drawing_spring_joint: false,
    draw_state: {entity: null, offset: null, prev_theta: null, t_param: null, applied_pos: null},
    recent_entities: [],
    cubic_bezier_active: false,
    cubic_bezier_curve: null,
    standard_interactable_radius: 0.1,
    num_objects_in_bezier: 20,
    object_bezier_mass: 1,
    angular_motor_vel: 20,

    dynamicObject(physicsState, solver, mouse, mass, radius_mult) {
        const pos = this.snap_to_grid ? Units.snap_to_grid(mouse.sim_pos) : mouse.sim_pos;
        physicsState.addObject(new DynamicObject(pos, mass, radius_mult * solver.standard_radius)); // TEMPORARY: 20 and 5 should be 1
        const length = physicsState.getDynamicObjectsLength();
        this.recent_entities.push({type: "DynamicObject", id: length - 1});
    },

    fixedXConstraint(physicsState, solver, mouse) {
        const id = physicsState.getObjIndexContainingPos(mouse.sim_pos);
        physicsState.addFixedXConstraint(id);
        const length = physicsState.getRenderedConstraintLength();
        this.recent_entities.push({type: "Constraint", id: length - 1});
    },

    fixedYConstraint(physicsState, solver, mouse) {
        const id = physicsState.getObjIndexContainingPos(mouse.sim_pos);
        physicsState.addFixedYConstraint(id);
        const length = physicsState.getRenderedConstraintLength();
        this.recent_entities.push({type: "Constraint", id: length - 1});
    },

    fixedPosConstraint(physicsState, solver, mouse) {
        const id = physicsState.getObjIndexContainingPos(mouse.sim_pos);
        physicsState.addFixedPosConstraint(id);
        const length = physicsState.getRenderedConstraintLength();
        this.recent_entities.push({type: "Constraint", id: length - 1});
    },

    fixedRotConstraint(physicsState, solver, mouse) {
        const id = physicsState.getObjIndexContainingPos(mouse.sim_pos) 
        physicsState.addFixedRotConstraint(id);
        const length = physicsState.getRenderedConstraintLength();
        this.recent_entities.push({type: "Constraint", id: length - 1});   
    },

    fixedRotOmegaConstraint(physicsState, solver, mouse) {
        const id = physicsState.getObjIndexContainingPos(mouse.sim_pos)     
        const vel = solver.dt * this.angular_motor_vel;
        physicsState.addFixedOmegaConstraint(id, vel);
        const length = physicsState.getRenderedConstraintLength();
        this.recent_entities.push({type: "Constraint", id: length - 1});   
    },

    removeMostRecentEntity(physicsState) {
        // if no recent entities exist, dont do anything
        if (this.recent_entities.length == 0)
            return;

        const last_entity = this.recent_entities.pop();
        if (last_entity.type == "DynamicObject")
            physicsState.removeByIdDynamicObject(last_entity.id);
        if (last_entity.type == "ForceGenerator")
            physicsState.removeByIdForceGenerator(last_entity.id);
        if (last_entity.type == "Constraint")
            physicsState.removeLastConstraint();
    },

    // add other stuff to the scene -->>>
    addRagdoll(physicsState, solver, mouse) {
        const pos = this.snap_to_grid ? Units.snap_to_grid(mouse.sim_pos) : mouse.sim_pos;
        const rad = solver.standard_radius;
        const object_id_offset = physicsState.getDynamicObjectsLength();
        const constraint_id_offset = physicsState.getConstraintLength();
    
        const self = this;
        function m_linkConstraint(id1, id2) {
            physicsState.addLinkConstraint(id1 + object_id_offset, id2 + object_id_offset);
            self.recent_entities.push({type: "Constraint", id: constraint_id_offset - 1});
        }
    
        // objects
        const objects = [ // objects in their own space
            new DynamicObject(new Vector2(0,0),         3, 3 * rad),        // 0
            new DynamicObject(new Vector2(0, -0.24),    1, rad),   // 1
            new DynamicObject(new Vector2(-0.2, -0.3),  1, rad), // 2
            new DynamicObject(new Vector2(-0.15, -0.9), 1, rad), // 3
            new DynamicObject(new Vector2( 0.15, -0.9), 1, rad), // 4
            new DynamicObject(new Vector2(0.2, -0.3),   1, rad), // 5
            new DynamicObject(new Vector2(-0.5, -0.3),  1, rad), // 6
            new DynamicObject(new Vector2(-0.8, -0.3),  1, rad), // 7
            new DynamicObject(new Vector2(0.5, -0.3),   1, rad), // 8
            new DynamicObject(new Vector2(0.8, -0.3),   1, rad), // 9
            new DynamicObject(new Vector2(-0.2, -1.2),  1, rad), // 10
            new DynamicObject(new Vector2(-0.25, -1.5),   1, rad), // 11
            new DynamicObject(new Vector2(0.2, -1.2),   1, rad), // 12
            new DynamicObject(new Vector2(0.25, -1.5),   1, rad), // 13
        ]

        for (let i = 0; i < objects.length; i++) {
            objects[i].pos = Vector2.addVectors(objects[i].pos, pos);
            physicsState.addObject(objects[i]);
            this.recent_entities.push({type: "DynamicObject", id: i + object_id_offset});
        }
    
        // constraints
        // torso
        // vertical / horizontal
        m_linkConstraint(2, 3);
        m_linkConstraint(3, 4);
        m_linkConstraint(4, 5);
        m_linkConstraint(2, 5);
        // diagonals
        m_linkConstraint(2, 4);
        m_linkConstraint(3, 5);
        // left arm
        m_linkConstraint(2, 6);
        m_linkConstraint(6, 7);
        // right arm
        m_linkConstraint(5, 8);
        m_linkConstraint(8, 9);
        // left leg
        m_linkConstraint(3, 10);
        m_linkConstraint(10, 11);
        // right leg
        m_linkConstraint(4, 12);
        m_linkConstraint(12, 13);
        // traps
        m_linkConstraint(1, 2);
        m_linkConstraint(1, 5);
        // head
        m_linkConstraint(0, 1);
        
    },

    addCubicBezierCurve(mouse) {
        // its flipped so that no matter if mouse pos if left or right of
        // units.width the bezier curve will always reflect correctly
        // also change according to snap_to_grid
        const pos1 =    !this.snap_to_grid ? 
                            mouse.sim_pos.x < Units.WIDTH / 2 ? 
                            mouse.sim_pos : new Vector2(Units.WIDTH - mouse.sim_pos.x, mouse.sim_pos.y)
                        :   mouse.sim_pos.x < Units.WIDTH / 2 ? 
                            Units.snap_to_grid(mouse.sim_pos) : Units.snap_to_grid(new Vector2(Units.WIDTH - mouse.sim_pos.x, mouse.sim_pos.y));
        const pos2 =    !this.snap_to_grid ? 
                            mouse.sim_pos.x < Units.WIDTH / 2 ? 
                            new Vector2(Units.WIDTH - mouse.sim_pos.x, mouse.sim_pos.y) : mouse.sim_pos
                        :   mouse.sim_pos.x < Units.WIDTH / 2 ? 
                            Units.snap_to_grid(new Vector2(Units.WIDTH - mouse.sim_pos.x, mouse.sim_pos.y)) : Units.snap_to_grid(mouse.sim_pos);
        this.cubic_bezier_curve = new CubicBezier(pos1, pos2, this.standard_interactable_radius);
    },

    spawnViaCubicBezier(physicsState, solver, mouse) {
        const rad = solver.standard_radius;
        const object_id_offset = physicsState.getDynamicObjectsLength();
        const constraint_id_offset = physicsState.getConstraintLength();
    
        const self = this;
        function m_linkConstraint(id1, id2) {
            physicsState.addLinkConstraint(id1 + object_id_offset, id2 + object_id_offset);
            self.recent_entities.push({type: "Constraint", id: constraint_id_offset - 1});
        }

        for (let i = 0; i <= this.num_objects_in_bezier; i++) {
            const t = i / this.num_objects_in_bezier;
            const position = this.cubic_bezier_curve.getPointOnCurve(t);
            const object = new DynamicObject(position, this.object_bezier_mass, rad);
            physicsState.addObject(object);
            this.recent_entities.push({type: "DynamicObject", id: i + object_id_offset});
        }

        for (let i = 0; i < this.num_objects_in_bezier; i++) {
            m_linkConstraint(i, i + 1);
        }

        this.cubic_bezier_curve = null;
        this.cubic_bezier_active = false;

    },

    update(mouse) {
        this.cubic_bezier_curve.update(this.snap_to_grid, mouse);
    },

    render(c, physicsState, solver, mouse) {
        if (this.cubic_bezier_active)
            this.cubic_bezier_curve.render(c, this.num_objects_in_bezier);

        if ((this.drawing_link_constraint || this.drawing_spring_joint)) {
            let p1;
            if (this.drawing_link_constraint) {
                if (this.draw_state.entity instanceof DynamicObject)
                    p1 = this.draw_state.applied_pos;
                else
                    p1 = physicsState.getObjectPositionById(this.draw_state.entity);
            } else {
                p1 = this.draw_state.applied_pos;
            }

            const p2 = mouse.sim_pos;
            c.lineCap = "round";
            c.strokeStyle = Colours.INNER_CUBIC_BEZIER_SUPPORTING_LINES;
            c.lineWidth = LineWidths.INNER_CUBIC_BEZIER_LINE;
            c.beginPath();
            c.moveTo(Units.sim_canv_x(p1), Units.sim_canv_y(p1));
            c.lineTo(Units.sim_canv_x(p2), Units.sim_canv_y(p2));
            c.stroke();
            c.closePath();
        }

    }, 

}