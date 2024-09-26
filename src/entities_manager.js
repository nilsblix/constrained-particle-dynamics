import {Units} from "./main.js";
import {DynamicObject} from "./dynamicObject.js";
import {Vector2} from "./linear_algebra.js";

export const entities_manager = {
    active: false,
    snap_to_grid: false,
    line_start_id: -1,
    drawing_line: false,
    drawing_spring_joint: false,
    recent_entities: [],

    dynamicObject(physicsState, solver, mouse) {
        const mouse_sim_pos = Units.canv_sim(mouse.canv_pos);
        const pos = this.snap_to_grid ? Units.snap_to_grid(mouse_sim_pos) : mouse_sim_pos;
        physicsState.addObject(new DynamicObject(pos, 1, solver.standard_radius));
        const length = physicsState.getDynamicObjectsLength();
        this.recent_entities.push({type: "DynamicObject", id: length - 1});
    },

    fixedXConstraint(physicsState, solver, mouse) {
        const mouse_sim_pos = Units.canv_sim(mouse.canv_pos);
        const id = physicsState.getObjIndexContainingPos(mouse_sim_pos);
        physicsState.addFixedXConstraint(id);
        const length = physicsState.getRenderedConstraintLength();
        this.recent_entities.push({type: "Constraint", id: length - 1});
    },

    fixedYConstraint(physicsState, solver, mouse) {
        const mouse_sim_pos = Units.canv_sim(mouse.canv_pos);
        const id = physicsState.getObjIndexContainingPos(mouse_sim_pos);
        physicsState.addFixedYConstraint(id);
        const length = physicsState.getRenderedConstraintLength();
        this.recent_entities.push({type: "Constraint", id: length - 1});
    },

    fixedPosConstraint(physicsState, solver, mouse) {
        const mouse_sim_pos = Units.canv_sim(mouse.canv_pos);
        const id = physicsState.getObjIndexContainingPos(mouse_sim_pos);
        physicsState.addFixedPosConstraint(id);
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
        const mouse_sim_pos = Units.canv_sim(mouse.canv_pos);
        const pos = this.snap_to_grid ? Units.snap_to_grid(mouse_sim_pos) : mouse_sim_pos;
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

}