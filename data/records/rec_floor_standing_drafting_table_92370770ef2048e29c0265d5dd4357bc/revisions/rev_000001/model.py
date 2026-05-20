from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_drafting_table")

    graphite = model.material("graphite_powder_coat", color=(0.10, 0.11, 0.12, 1.0))
    dark_steel = model.material("dark_welded_steel", color=(0.05, 0.055, 0.06, 1.0))
    satin_steel = model.material("satin_machined_steel", color=(0.55, 0.58, 0.60, 1.0))
    board_green = model.material("matte_drafting_board", color=(0.72, 0.78, 0.68, 1.0))
    pale_edge = model.material("laminated_pale_edge", color=(0.86, 0.82, 0.70, 1.0))
    drawer_mat = model.material("shallow_aluminum_drawer", color=(0.34, 0.36, 0.38, 1.0))
    rubber = model.material("black_rubber", color=(0.015, 0.015, 0.014, 1.0))

    def box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    def cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
        part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    # Root: a broad welded T/H base and fixed vertical column, sized like a
    # full-scale professional drafting table rather than a desktop stand.
    base = model.part("welded_base")
    box(base, "wide_cross_tube", (1.62, 0.085, 0.075), (0.0, 0.26, 0.050), dark_steel)
    box(base, "foot_tube_0", (0.095, 0.84, 0.070), (-0.66, 0.02, 0.045), dark_steel)
    box(base, "foot_tube_1", (0.095, 0.84, 0.070), (0.66, 0.02, 0.045), dark_steel)
    box(base, "center_spine", (0.11, 0.74, 0.060), (0.0, 0.01, 0.082), dark_steel)
    box(base, "column", (0.100, 0.100, 1.34), (0.0, 0.26, 0.750), graphite)
    box(base, "column_foot_plate", (0.32, 0.24, 0.035), (0.0, 0.26, 0.132), graphite)
    box(base, "front_brace", (0.065, 0.58, 0.055), (0.0, 0.055, 0.265), graphite, rpy=(math.radians(22), 0.0, 0.0))
    box(base, "rear_brace", (0.065, 0.42, 0.055), (0.0, 0.385, 0.285), graphite, rpy=(math.radians(-26), 0.0, 0.0))
    for i, x in enumerate((-0.66, 0.66)):
        for j, y in enumerate((-0.38, 0.42)):
            cyl(base, f"leveling_pad_{i}_{j}", 0.052, 0.018, (x, y, 0.009), rubber)

    # Height carriage: a four-plate sliding collar that visibly clears the
    # fixed square column, plus forward trunnion cheeks that carry the tilt pin.
    carriage = model.part("carriage")
    box(carriage, "sleeve_side_0", (0.030, 0.210, 0.340), (-0.083, 0.0, 0.0), graphite)
    box(carriage, "sleeve_side_1", (0.030, 0.210, 0.340), (0.083, 0.0, 0.0), graphite)
    box(carriage, "sleeve_face_0", (0.195, 0.030, 0.340), (0.0, -0.083, 0.0), graphite)
    box(carriage, "sleeve_face_1", (0.195, 0.030, 0.340), (0.0, 0.083, 0.0), graphite)
    box(carriage, "front_glide_pad", (0.060, 0.018, 0.200), (0.0, -0.059, 0.030), rubber)
    box(carriage, "rear_glide_pad", (0.060, 0.018, 0.200), (0.0, 0.059, 0.030), rubber)
    box(carriage, "locking_boss", (0.085, 0.050, 0.075), (0.0, -0.123, 0.075), satin_steel)
    cyl(carriage, "height_lock_knob", 0.035, 0.055, (0.0, -0.175, 0.075), satin_steel, rpy=(math.pi / 2.0, 0.0, 0.0))
    box(carriage, "tilt_arm", (0.70, 0.530, 0.055), (0.0, -0.335, 0.185), graphite)
    box(carriage, "tilt_cheek_0", (0.055, 0.065, 0.165), (-0.315, -0.585, 0.255), graphite)
    box(carriage, "tilt_cheek_1", (0.055, 0.065, 0.165), (0.315, -0.585, 0.255), graphite)
    cyl(carriage, "tilt_pin", 0.020, 0.72, (0.0, -0.585, 0.330), satin_steel, rpy=(0.0, math.pi / 2.0, 0.0))

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.26, 0.720)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.18, lower=0.0, upper=0.34),
    )

    # Main board frame pivots about the trunnion line.  Its frame includes the
    # large board, a rigid side rail that carries the small reference wing, and
    # short drawer runners under the front half of the board.
    board_frame = model.part("board_frame")
    cyl(board_frame, "tilt_barrel", 0.024, 0.54, (0.0, 0.0, 0.0), satin_steel, rpy=(0.0, math.pi / 2.0, 0.0))
    box(board_frame, "hinge_web", (0.54, 0.055, 0.040), (0.0, -0.004, 0.018), graphite)
    box(board_frame, "main_panel", (1.22, 0.96, 0.034), (-0.155, -0.065, 0.046), board_green)
    box(board_frame, "front_frame_rail", (1.34, 0.046, 0.045), (-0.12, -0.575, 0.022), graphite)
    box(board_frame, "rear_frame_rail", (1.34, 0.046, 0.045), (-0.12, 0.445, 0.022), graphite)
    box(board_frame, "outer_side_rail", (0.046, 1.05, 0.045), (-0.785, -0.065, 0.022), graphite)
    box(board_frame, "wing_carry_rail", (0.046, 1.05, 0.045), (0.480, -0.065, 0.022), graphite)
    box(board_frame, "center_stiffener", (0.080, 0.50, 0.070), (-0.12, -0.315, -0.015), graphite)
    box(board_frame, "wing_hinge_leaf", (0.034, 0.92, 0.012), (0.440, -0.065, 0.056), satin_steel)
    for i, y in enumerate((-0.405, -0.065, 0.275)):
        cyl(board_frame, f"wing_hinge_knuckle_{i}", 0.013, 0.175, (0.483, y, 0.066), satin_steel, rpy=(-math.pi / 2.0, 0.0, 0.0))
        box(board_frame, f"wing_hinge_tab_{i}", (0.020, 0.120, 0.012), (0.464, y, 0.058), satin_steel)
    box(board_frame, "runner_0", (0.040, 0.450, 0.030), (-0.375, -0.355, -0.090), satin_steel)
    box(board_frame, "runner_1", (0.040, 0.450, 0.030), (0.245, -0.355, -0.090), satin_steel)
    box(board_frame, "runner_back_stop", (0.68, 0.035, 0.045), (-0.065, -0.130, -0.075), satin_steel)
    box(board_frame, "runner_hanger", (0.060, 0.040, 0.090), (-0.065, -0.130, -0.036), graphite)

    model.articulation(
        "carriage_to_board",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=board_frame,
        origin=Origin(xyz=(0.0, -0.585, 0.330)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.05),
    )

    # Split-top reference wing.  The wing frame is at its own hinge line and
    # the panel extends from that edge, making it clearly supported by the
    # main board frame rather than hovering beside it.
    wing = model.part("reference_wing")
    box(wing, "wing_panel", (0.305, 0.86, 0.032), (0.180, 0.0, -0.030), board_green)
    box(wing, "wing_outer_edge", (0.035, 0.86, 0.042), (0.350, 0.0, -0.026), pale_edge)
    box(wing, "wing_front_edge", (0.300, 0.030, 0.038), (0.185, -0.445, -0.027), pale_edge)
    box(wing, "wing_rear_edge", (0.300, 0.030, 0.038), (0.185, 0.445, -0.027), pale_edge)
    box(wing, "wing_hinge_leaf", (0.034, 0.82, 0.010), (0.045, 0.0, -0.018), satin_steel)
    for i, y in enumerate((-0.170, 0.170)):
        cyl(wing, f"hinge_knuckle_{i}", 0.012, 0.165, (0.0, y, 0.0), satin_steel, rpy=(-math.pi / 2.0, 0.0, 0.0))
        box(wing, f"hinge_tab_{i}", (0.025, 0.110, 0.016), (0.020, y, -0.012), satin_steel)

    model.articulation(
        "board_to_wing",
        ArticulationType.REVOLUTE,
        parent=board_frame,
        child=wing,
        origin=Origin(xyz=(0.483, -0.065, 0.066)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=1.35),
    )

    # Shallow accessory drawer, below the board and captured by the two short
    # runners on the board frame.  It slides forward toward the drafter.
    drawer = model.part("drawer")
    box(drawer, "tray_bottom", (0.560, 0.450, 0.022), (-0.065, -0.030, 0.000), drawer_mat)
    box(drawer, "front_face", (0.610, 0.030, 0.090), (-0.065, -0.265, 0.045), drawer_mat)
    box(drawer, "rear_lip", (0.560, 0.024, 0.040), (-0.065, 0.190, -0.005), drawer_mat)
    box(drawer, "side_wall_0", (0.030, 0.440, 0.085), (-0.245, -0.030, 0.042), drawer_mat)
    box(drawer, "side_wall_1", (0.030, 0.440, 0.085), (0.175, -0.030, 0.042), drawer_mat)
    box(drawer, "slide_flange_0", (0.055, 0.320, 0.018), (-0.335, -0.006, 0.016), satin_steel)
    box(drawer, "slide_flange_1", (0.055, 0.320, 0.018), (0.205, -0.006, 0.016), satin_steel)
    box(drawer, "flange_web_0", (0.120, 0.320, 0.018), (-0.290, -0.006, 0.014), drawer_mat)
    box(drawer, "flange_web_1", (0.090, 0.320, 0.018), (0.175, -0.006, 0.014), drawer_mat)
    cyl(drawer, "drawer_pull", 0.014, 0.400, (-0.065, -0.290, 0.060), satin_steel, rpy=(0.0, math.pi / 2.0, 0.0))

    model.articulation(
        "board_to_drawer",
        ArticulationType.PRISMATIC,
        parent=board_frame,
        child=drawer,
        origin=Origin(xyz=(-0.065, -0.355, -0.130)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.300),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("welded_base")
    carriage = object_model.get_part("carriage")
    board_frame = object_model.get_part("board_frame")
    wing = object_model.get_part("reference_wing")
    drawer = object_model.get_part("drawer")

    height = object_model.get_articulation("base_to_carriage")
    tilt = object_model.get_articulation("carriage_to_board")
    wing_hinge = object_model.get_articulation("board_to_wing")
    drawer_slide = object_model.get_articulation("board_to_drawer")

    ctx.check(
        "four primary drafting table mechanisms",
        len(object_model.articulations) == 4,
        details=f"found {[joint.name for joint in object_model.articulations]}",
    )

    ctx.allow_overlap(
        board_frame,
        carriage,
        elem_a="tilt_barrel",
        elem_b="tilt_pin",
        reason="The main board's hinge barrel is intentionally modeled around the carriage tilt pin.",
    )
    ctx.allow_overlap(
        board_frame,
        carriage,
        elem_a="hinge_web",
        elem_b="tilt_pin",
        reason="The simplified hinge saddle is shown as a solid web wrapped around the hidden tilt pin.",
    )

    ctx.expect_within(
        carriage,
        base,
        axes="xy",
        inner_elem="sleeve_face_0",
        outer_elem="column",
        margin=0.075,
        name="carriage sleeve is centered around the fixed column",
    )
    ctx.expect_overlap(
        board_frame,
        carriage,
        axes="x",
        elem_a="tilt_barrel",
        elem_b="tilt_pin",
        min_overlap=0.50,
        name="main board tilt barrel is carried across the carriage pin",
    )
    ctx.expect_overlap(
        board_frame,
        carriage,
        axes="x",
        elem_a="hinge_web",
        elem_b="tilt_pin",
        min_overlap=0.50,
        name="hinge saddle remains centered on the tilt pin",
    )
    ctx.expect_overlap(
        wing,
        board_frame,
        axes="y",
        elem_a="wing_panel",
        elem_b="wing_carry_rail",
        min_overlap=0.80,
        name="reference wing is supported along the board edge",
    )
    ctx.expect_overlap(
        drawer,
        board_frame,
        axes="y",
        elem_a="slide_flange_0",
        elem_b="runner_0",
        min_overlap=0.28,
        name="closed drawer is retained by the short runner",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({height: 0.34}):
        raised_carriage = ctx.part_world_position(carriage)
    ctx.check(
        "carriage slides upward on vertical support",
        rest_carriage is not None
        and raised_carriage is not None
        and raised_carriage[2] > rest_carriage[2] + 0.30,
        details=f"rest={rest_carriage}, raised={raised_carriage}",
    )

    rest_board_aabb = ctx.part_element_world_aabb(board_frame, elem="rear_frame_rail")
    with ctx.pose({tilt: 0.85}):
        tilted_board_aabb = ctx.part_element_world_aabb(board_frame, elem="rear_frame_rail")
    ctx.check(
        "main board tilts upward about horizontal hinge",
        rest_board_aabb is not None
        and tilted_board_aabb is not None
        and tilted_board_aabb[1][2] > rest_board_aabb[1][2] + 0.25,
        details=f"rest={rest_board_aabb}, tilted={tilted_board_aabb}",
    )

    rest_wing_aabb = ctx.part_element_world_aabb(wing, elem="wing_outer_edge")
    with ctx.pose({wing_hinge: 1.0}):
        raised_wing_aabb = ctx.part_element_world_aabb(wing, elem="wing_outer_edge")
    ctx.check(
        "reference wing folds upward on its own edge hinge",
        rest_wing_aabb is not None
        and raised_wing_aabb is not None
        and raised_wing_aabb[1][2] > rest_wing_aabb[1][2] + 0.20,
        details=f"rest={rest_wing_aabb}, raised={raised_wing_aabb}",
    )

    rest_drawer = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.30}):
        pulled_drawer = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            board_frame,
            axes="y",
            elem_a="slide_flange_0",
            elem_b="runner_0",
            min_overlap=0.030,
            name="open drawer still has retained runner engagement",
        )
    ctx.check(
        "accessory drawer slides forward on guide runners",
        rest_drawer is not None
        and pulled_drawer is not None
        and pulled_drawer[1] < rest_drawer[1] - 0.25,
        details=f"rest={rest_drawer}, pulled={pulled_drawer}",
    )

    return ctx.report()


object_model = build_object_model()
