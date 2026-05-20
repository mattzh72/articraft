from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_X = 0.154
BODY_Y = 0.120
BODY_Z = 0.128
WALL_T = 0.007
BOTTOM_T = 0.008
TOP_T = 0.018
FRONT_T = 0.008
DRAWER_TRAVEL = 0.048


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_pencil_sharpener")

    body_red = model.material("body_red", rgba=(0.77, 0.14, 0.12, 1.0))
    trim_black = model.material("trim_black", rgba=(0.12, 0.12, 0.13, 1.0))
    drawer_black = model.material("drawer_black", rgba=(0.16, 0.16, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.73, 0.75, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_X, BODY_Y, BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_T * 0.5)),
        material=body_red,
        name="bottom_plate",
    )
    wall_height = BODY_Z - BOTTOM_T
    wall_x = BODY_X - FRONT_T
    wall_center_x = -FRONT_T * 0.5
    wall_center_z = BOTTOM_T + wall_height * 0.5
    body.visual(
        Box((wall_x, WALL_T, wall_height)),
        origin=Origin(xyz=(wall_center_x, BODY_Y * 0.5 - WALL_T * 0.5, wall_center_z)),
        material=body_red,
        name="side_wall_0",
    )
    body.visual(
        Box((wall_x, WALL_T, wall_height)),
        origin=Origin(xyz=(wall_center_x, -BODY_Y * 0.5 + WALL_T * 0.5, wall_center_z)),
        material=body_red,
        name="side_wall_1",
    )
    body.visual(
        Box((WALL_T, BODY_Y - 2.0 * WALL_T, wall_height)),
        origin=Origin(
            xyz=(-BODY_X * 0.5 + WALL_T * 0.5, 0.0, wall_center_z),
        ),
        material=body_red,
        name="back_wall",
    )
    body.visual(
        Box((wall_x, BODY_Y - 2.0 * WALL_T, TOP_T)),
        origin=Origin(
            xyz=(wall_center_x, 0.0, BODY_Z - TOP_T * 0.5),
        ),
        material=body_red,
        name="top_cover",
    )
    body.visual(
        Box((FRONT_T, BODY_Y - 2.0 * WALL_T, 0.038)),
        origin=Origin(
            xyz=(BODY_X * 0.5 - FRONT_T * 0.5, 0.0, 0.089),
        ),
        material=body_red,
        name="front_upper",
    )
    body.visual(
        Box((FRONT_T, BODY_Y - 2.0 * WALL_T, 0.010)),
        origin=Origin(
            xyz=(BODY_X * 0.5 - FRONT_T * 0.5, 0.0, 0.013),
        ),
        material=body_red,
        name="lower_lip",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(
            xyz=(BODY_X * 0.5 + 0.006, 0.0, 0.091),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=trim_black,
        name="pencil_port",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(0.026, 0.0, BODY_Z + 0.002)),
        material=trim_black,
        name="knob_boss",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(
            xyz=(0.006, BODY_Y * 0.5 - 0.002, 0.082),
            rpy=(1.5707963267948966, 0.0, 0.0),
        ),
        material=steel,
        name="crank_bushing",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.118, 0.092, 0.004)),
        origin=Origin(xyz=(-0.059, 0.0, 0.002)),
        material=drawer_black,
        name="drawer_floor",
    )
    drawer.visual(
        Box((0.118, 0.004, 0.040)),
        origin=Origin(xyz=(-0.059, 0.044, 0.022)),
        material=drawer_black,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.118, 0.004, 0.040)),
        origin=Origin(xyz=(-0.059, -0.044, 0.022)),
        material=drawer_black,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((0.004, 0.092, 0.040)),
        origin=Origin(xyz=(-0.116, 0.0, 0.022)),
        material=drawer_black,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.012, 0.100, 0.050)),
        origin=Origin(xyz=(0.006, 0.0, 0.025)),
        material=drawer_black,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.010, 0.040, 0.010)),
        origin=Origin(xyz=(0.013, 0.0, 0.034)),
        material=trim_black,
        name="drawer_pull",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="crank_axle",
    )
    crank.visual(
        Cylinder(radius=0.013, length=0.007),
        origin=Origin(xyz=(0.0, 0.0065, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="crank_hub",
    )
    crank.visual(
        Box((0.046, 0.008, 0.008)),
        origin=Origin(xyz=(0.023, 0.014, -0.004)),
        material=steel,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(xyz=(0.046, 0.014, -0.018)),
        material=steel,
        name="handle_post",
    )
    crank.visual(
        Cylinder(radius=0.0065, length=0.024),
        origin=Origin(xyz=(0.046, 0.026, -0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="handle_grip",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.028,
                0.015,
                body_style="skirted",
                top_diameter=0.022,
                skirt=KnobSkirt(0.034, 0.004, flare=0.04),
                grip=KnobGrip(style="fluted", count=14, depth=0.0008),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "selector_knob",
        ),
        material=trim_black,
        name="knob_shell",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.065, 0.0, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.10,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(0.006, BODY_Y * 0.5, 0.082)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=12.0),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(0.026, 0.0, BODY_Z + 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    selector_knob = object_model.get_part("selector_knob")
    drawer_slide = object_model.get_articulation("body_to_drawer")
    crank_joint = object_model.get_articulation("body_to_crank")
    knob_joint = object_model.get_articulation("body_to_selector_knob")

    ctx.expect_within(
        drawer,
        body,
        axes="yz",
        margin=0.006,
        name="drawer stays inside body width and height envelope",
    )

    closed_front = ctx.part_element_world_aabb(drawer, elem="drawer_front")
    frame_front = ctx.part_element_world_aabb(body, elem="front_upper")
    flush_ok = False
    flush_details = f"drawer_front={closed_front!r}, frame_front={frame_front!r}"
    if closed_front is not None and frame_front is not None:
        flush_ok = abs(closed_front[1][0] - frame_front[1][0]) <= 0.0015
    ctx.check("drawer front sits flush when closed", flush_ok, flush_details)

    closed_max_x = closed_front[1][0] if closed_front is not None else None
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="drawer_floor",
            min_overlap=0.055,
            name="drawer remains captured at full extension",
        )
        extended_front = ctx.part_element_world_aabb(drawer, elem="drawer_front")
    moved_out = False
    moved_details = f"closed_max_x={closed_max_x!r}, extended_front={extended_front!r}"
    if closed_max_x is not None and extended_front is not None:
        moved_out = extended_front[1][0] > closed_max_x + 0.035
    ctx.check("drawer slides outward from the front", moved_out, moved_details)

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    handle_rest = _aabb_center(ctx.part_element_world_aabb(crank, elem="handle_grip"))
    with ctx.pose({crank_joint: math.pi / 2.0}):
        handle_quarter = _aabb_center(ctx.part_element_world_aabb(crank, elem="handle_grip"))
    crank_sweeps = False
    crank_details = f"rest={handle_rest!r}, quarter={handle_quarter!r}"
    if handle_rest is not None and handle_quarter is not None:
        dx = abs(handle_quarter[0] - handle_rest[0])
        dz = abs(handle_quarter[2] - handle_rest[2])
        crank_sweeps = dx > 0.045 and dz > 0.018
    ctx.check("crank handle sweeps through a turning arc", crank_sweeps, crank_details)

    knob_aabb = ctx.part_element_world_aabb(selector_knob, elem="knob_shell")
    port_aabb = ctx.part_element_world_aabb(body, elem="pencil_port")
    knob_center = _aabb_center(knob_aabb)
    port_center = _aabb_center(port_aabb)
    knob_position_ok = False
    knob_details = f"knob_center={knob_center!r}, port_center={port_center!r}"
    if knob_center is not None and port_center is not None:
        knob_position_ok = (
            abs(knob_center[1] - port_center[1]) <= 0.006
            and knob_center[2] > port_center[2] + 0.030
        )
    ctx.check("selector knob sits above the pencil port", knob_position_ok, knob_details)

    controls_ok = (
        crank_joint.articulation_type == ArticulationType.CONTINUOUS
        and knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and crank_joint.motion_limits is not None
        and knob_joint.motion_limits is not None
        and crank_joint.motion_limits.lower is None
        and crank_joint.motion_limits.upper is None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None
    )
    ctx.check(
        "crank and selector are continuous controls",
        controls_ok,
        details=(
            f"crank_type={crank_joint.articulation_type!r}, "
            f"crank_limits={crank_joint.motion_limits!r}, "
            f"knob_type={knob_joint.articulation_type!r}, "
            f"knob_limits={knob_joint.motion_limits!r}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
