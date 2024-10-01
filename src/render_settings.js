export const Colours = {
    INNER_DYNAMIC_OBJECT: "rgba(250, 201, 67, 1)",
    OUTER_DYNAMIC_OBJECT: "rgba(0,0,0,1)",
    INNER_SPRING_ENDS: "#ffffff",
    OUTER_SPRING_ENDS: "#000000",
    INNER_MOUSE_SPRING_MOUSE_CIRCLE: "rgba(156, 58, 58, 1)",
    OUTER_MOUSE_SPRING_MOUSE_CIRCLE: "#000000",
    INNER_SPRING_SEGMENTS: "#ffffff",
    OUTER_SPRING_SEGMENTS: "#000000",
    INNER_FIXEDPOSCONSTRAINT_CONNECTION_TO_OBJECT: "#ffffff",
    OUTER_FIXEDPOSCONSTRAINT_CONNECTION_TO_OBJECT: "#000000",
    INNER_FIXEDPOSCONSTRAINT_HORIZONTAL_LINE: "#ffffff",
    OUTER_FIXEDPOSCONSTRAINT_HORIZONTAL_LINE: "#000000",
    INNER_FIXEDYCONSTRAINT_CONNECTION_TO_OBJECT: "#ffffff",
    OUTER_FIXEDYCONSTRAINT_CONNECTION_TO_OBJECT: "#000000",
    INNER_FIXEDYCONSTRAINT_HORIZONTAL_LINE: "#ffffff",
    OUTER_FIXEDYCONSTRAINT_HORIZONTAL_LINE: "#000000",
    INNER_FIXEDXCONSTRAINT_VERTICAL_LINE: "#ffffff",
    OUTER_FIXEDXCONSTRAINT_VERTICAL_LINE: "#000000",
    INNER_FIXEDXCONSTRAINT_HORIZONTAL_LINE: "#ffffff",
    OUTER_FIXEDXCONSTRAINT_HORIZONTAL_LINE: "#000000",
    INNER_LINKCONSTRAINT: "#ffffff",
    OUTER_LINKCONSTRAINT: "#000000",
    INNER_INTERACTABLEOBJECT: "#c4c4c4",
    OUTER_INTERACTABLEOBJECT: "#000000",
    INNER_CUBIC_BEZIER: "#c4c4c4",
    OUTER_CUBIC_BEZIER: "#000000",
    INNER_PREVIEW_BEZIER_OBJECTS: "#cf4b23",
    OUTER_PREVIEW_BEZIER_OBJECTS: "#000000",
    INNER_CUBIC_BEZIER_SUPPORTING_LINES: "rgba(200, 200, 200, 0.3)",
    INNER_FIXED_OMEGA_CONSTRAINT: "#d93030",
    OUTER_FIXED_OMEGA_CONSTRAINT: "#000000",
}

export const LineWidths = {
    DYNAMIC_OBJECT: 2,
    SPRING_ENDS: 2,
    // MOUSE_SPRING_ENDS_BORDER: 2,
    MOUSE_SPRING_MOUSE_CIRCLE: 2,
    // MOUSE_SPRING_MOUSE_CIRCLE_BORDER: 2,
    SPRING_SEGMENTS: 3, 
    SPRING_SEGMENTS_BORDER: 1, 
    SPRING_EXTREME_SEGMENTS: 4, // by extreme it is the first or last segment (orthogonal to A - B)
    SPRING_EXTREME_SEGMENTS_BORDER: 1, 
    OUTER_FIXEDPOSCONSTRAINT_CONNECTION_TO_OBJECT: 2, // the outer border width
    INNER_FIXEDPOSCONSTRAINT_CONNECTION_TO_OBJECT: 3, // inner width}
    OUTER_FIXEDPOSCONSTRAINT_HORIZONTAL_LINE: 5, // the outer border width
    INNER_FIXEDPOSCONSTRAINT_HORIZONTAL_LINE: 3, // inner width}
    OUTER_FIXEDYCONSTRAINT_CONNECTION_TO_OBJECT: 2, // the outer border width
    INNER_FIXEDYCONSTRAINT_CONNECTION_TO_OBJECT: 3, // inner width
    OUTER_FIXEDYCONSTRAINT_HORIZONTAL_LINE: 5, // the outer border width
    INNER_FIXEDYCONSTRAINT_HORIZONTAL_LINE: 3, // inner width
    OUTER_FIXEDXCONSTRAINT_VERTICAL_LINE: 5,
    INNER_FIXEDXCONSTRAINT_VERTICAL_LINE: 3,
    OUTER_FIXEDXCONSTRAINT_HORIZONTAL_LINE: 5,
    INNER_FIXEDXCONSTRAINT_HORIZONTAL_LINE: 3,
    INNER_LINKCONSTRAINT: 6,
    OUTER_LINKCONSTRAINT: 10,
    INTERACTABLEOBJECT: 2,
    INNER_CUBIC_BEZIER_LINE: 6,
    OUTER_CUBIC_BEZIER_LINE: 10,
    INNER_FIXED_OMEGA_CONSTRAINT: 5,
    OUTER_FIXED_OMEGA_CONSTRAINT: 3,
}


export const Extras = {
    SPRING_EXTREME_ENDCAPS: "round", // by extreme it is the first or last segment (orthogonal to A - B)
    SPRING_SEGMENTS_ENDCAPS: "round", 
    FIXEDPOSCONSTRAINT_CONNECTION_TO_OBJECT_ENDCAPS: "butt",
    FIXEDPOSCONSTRAINT_HORIZONTAL_LINE_ENDCAPS: "round",
    FIXEDYCONSTRAINT_CONNECTION_TO_OBJECT_ENDCAPS: "butt",
    FIXEDYCONSTRAINT_HORIZONTAL_LINE_ENDCAPS: "round",
    FIXEDXCONSTRAINT_VERTICAL_LINE_ENDCAPS: "round",
    FIXEDXCONSTRAINT_HORIZONTAL_LINE_ENDCAPS: "round",
    LINKCONSTRAINT_STRESS_MULTIPLIER: 0.6,
    LINKCONSTRAINT_STRESS_BOOL: true,
    LINKCONSTRAINT_ENDCAPS: "round",
    MOUSESPRING_NUM_SEGMENTS: 18,
    SPRINGJOINT_NUM_SEGMENTS: 16,
    CUBIC_BEZIER_NUM_SEGMENTS: 100,
    PREVIEW_BEZIER_OBJECTS_RADIUS: 0.05,
    DYNAMIC_OBJECT_ROTATIONAL_RADIUS_MULT: 0.15,
    DYNAMIC_OBJECT_ROTATIONAL_POSITION_MULT: 0.6,
    SPRING_JOINT_ENDS_RADIUS: 0.025,
    FIXED_OMEGA_CONSTRAINT_ANGLE: Math.PI / 4, // the angle from the y-axle
    FIXED_OMEGA_CONSTRAINT_TRIANGLE_SIZE: 0.06,
}

