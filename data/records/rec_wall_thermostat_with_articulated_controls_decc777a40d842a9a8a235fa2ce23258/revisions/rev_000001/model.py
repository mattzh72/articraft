from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.152
BODY_H = 0.122
BODY_D = 0.032
BODY_CORNER_R = 0.013
BODY_FRONT_EDGE_R = 0.003

DIAL_OUTER_D = 0.070
DIAL_INNER_D = 0.054
DIAL_T = 0.007
DIAL_CENTER_Y = -0.002

BUTTON_W = 0.022
BUTTON_H = 0.010
BUTTON_T = 0.004
BUTTON_STEM_W = 0.016
BUTTON_STEM_H = 0.006
BUTTON_STEM_D = 0.006
BUTTON_TRAVEL = 0.0025
BUTTON_ROW_Y = -0.047
BUTTON_XS = (-0.034, 0.0, 0.034)
BUTTON_POCKET_W = BUTTON_W
BUTTON_POCKET_H = BUTTON_H
BUTTON_POCKET_D = 0.0075

FLAP_W = 0.114
FLAP_H = 0.020
FLAP_OPENING_H = 0.024
FLAP_T = 0.0036
FLAP_TRAVEL = 1.25
FLAP_HINGE_Y = 0.056
FLAP_HINGE_Z = BODY_D + 0.0002
FLAP_POCKET_D = 0.011
HINGE_PIN_R = 0.0024
HINGE_PIN_L = 0.028
HINGE_CUFF_R = 0.00325
HINGE_CUFF_INNER_R = 0.00285
HINGE_CUFF_L = 0.026


def _box_cutter(width: float, height: float, depth: float, *, x: float, y: float, z_min: float):
    return cq.Workplane("XY").box(width, height, depth).translate((x, y, z_min + (depth * 0.5)))


def _build_housing_shape():
    housing = cq.Workplane("XY").box(BODY_W, BODY_H, BODY_D).translate((0.0, 0.0, BODY_D * 0.5))
    housing = housing.edges("|Z").fillet(BODY_CORNER_R)
    housing = housing.faces(">Z").edges().fillet(BODY_FRONT_EDGE_R)

    dial_dish = (
        cq.Workplane("XY")
        .center(0.0, DIAL_CENTER_Y)
        .circle(0.026)
        .extrude(0.0014)
        .translate((0.0, 0.0, BODY_D - 0.0014))
    )
    housing = housing.cut(dial_dish)

    flap_pocket = _box_cutter(
        FLAP_W,
        FLAP_OPENING_H,
        FLAP_POCKET_D,
        x=0.0,
        y=FLAP_HINGE_Y - (FLAP_OPENING_H * 0.5),
        z_min=BODY_D - FLAP_POCKET_D,
    )
    housing = housing.cut(flap_pocket)

    button_pockets = None
    for x_pos in BUTTON_XS:
        pocket = _box_cutter(
            BUTTON_POCKET_W,
            BUTTON_POCKET_H,
            BUTTON_POCKET_D,
            x=x_pos,
            y=BUTTON_ROW_Y,
            z_min=BODY_D - BUTTON_POCKET_D,
        )
        button_pockets = pocket if button_pockets is None else button_pockets.union(pocket)
    if button_pockets is not None:
        housing = housing.cut(button_pockets)

    hinge_relief = (
        cq.Workplane("YZ")
        .workplane(offset=0.0)
        .center(FLAP_HINGE_Y, FLAP_HINGE_Z)
        .circle(HINGE_CUFF_R + 0.0004)
        .extrude((HINGE_CUFF_L * 0.5) + 0.001, both=True)
    )
    housing = housing.cut(hinge_relief)

    hinge_pin = (
        cq.Workplane("YZ")
        .workplane(offset=0.0)
        .center(FLAP_HINGE_Y, FLAP_HINGE_Z)
        .circle(HINGE_PIN_R)
        .extrude(HINGE_PIN_L * 0.5, both=True)
    )
    housing = housing.union(hinge_pin)

    return housing


def _build_dial_ring_shape():
    ring = cq.Workplane("XY").circle(DIAL_OUTER_D * 0.5).circle(DIAL_INNER_D * 0.5).extrude(DIAL_T)
    ring = ring.faces(">Z").edges().chamfer(0.0011)
    return ring


def _build_button_cap_shape():
    cap = cq.Workplane("XY").box(BUTTON_W, BUTTON_H, BUTTON_T).translate((0.0, 0.0, BUTTON_T * 0.5))
    cap = cap.edges("|Z").fillet(0.0016)
    return cap


def _build_flap_shape():
    panel = cq.Workplane("XY").box(FLAP_W, FLAP_H, FLAP_T).translate(
        (0.0, -((FLAP_H * 0.5) + HINGE_CUFF_R), FLAP_T * 0.5)
    )
    panel = panel.edges("|Z").fillet(0.0022)
    cuff_outer = cq.Workplane("YZ").circle(HINGE_CUFF_R).extrude(HINGE_CUFF_L * 0.5, both=True)
    cuff_inner = cq.Workplane("YZ").circle(HINGE_CUFF_INNER_R).extrude((HINGE_CUFF_L * 0.5) + 0.001, both=True)
    cuff = cuff_outer.cut(cuff_inner)
    return panel.union(cuff)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    housing_color = model.material("housing_color", rgba=(0.88, 0.89, 0.87, 1.0))
    ring_color = model.material("ring_color", rgba=(0.72, 0.74, 0.76, 1.0))
    button_color = model.material("button_color", rgba=(0.79, 0.80, 0.79, 1.0))
    flap_color = model.material("flap_color", rgba=(0.84, 0.85, 0.83, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shape(), "thermostat_housing"),
        material=housing_color,
        name="housing_shell",
    )

    dial_ring = model.part("dial_ring")
    dial_ring.visual(
        mesh_from_cadquery(_build_dial_ring_shape(), "thermostat_dial_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=ring_color,
        name="dial_ring",
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=dial_ring,
        origin=Origin(xyz=(0.0, DIAL_CENTER_Y, BODY_D)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    button_cap_mesh = mesh_from_cadquery(_build_button_cap_shape(), "thermostat_button_cap")
    for index, x_pos in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            button_cap_mesh,
            material=button_color,
            name="button_cap",
        )
        button.visual(
            Box((BUTTON_STEM_W, BUTTON_STEM_H, BUTTON_STEM_D)),
            origin=Origin(xyz=(0.0, 0.0, -(BUTTON_STEM_D * 0.5))),
            material=button_color,
            name="button_stem",
        )
        model.articulation(
            f"button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(x_pos, BUTTON_ROW_Y, BODY_D)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    setup_flap = model.part("setup_flap")
    setup_flap.visual(
        mesh_from_cadquery(_build_flap_shape(), "thermostat_setup_flap"),
        material=flap_color,
        name="flap_panel",
    )
    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=setup_flap,
        origin=Origin(xyz=(0.0, FLAP_HINGE_Y, FLAP_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=0.0,
            upper=FLAP_TRAVEL,
        ),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    dial_ring = object_model.get_part("dial_ring")
    flap = object_model.get_part("setup_flap")
    dial_spin = object_model.get_articulation("dial_spin")
    flap_hinge = object_model.get_articulation("flap_hinge")

    buttons = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [object_model.get_articulation(f"button_{index}_press") for index in range(3)]

    dial_limits = dial_spin.motion_limits
    ctx.check(
        "dial_joint_is_continuous",
        dial_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_spin.articulation_type!r}",
    )
    ctx.check(
        "dial_joint_is_unbounded",
        dial_limits is not None and dial_limits.lower is None and dial_limits.upper is None,
        details=f"limits={dial_limits!r}",
    )
    dial_aabb = ctx.part_element_world_aabb(dial_ring, elem="dial_ring")
    ctx.check(
        "dial ring seats on the housing face",
        dial_aabb is not None and abs(dial_aabb[0][2] - BODY_D) <= 0.0005,
        details=f"dial_aabb={dial_aabb!r}",
    )

    button_rest_positions = [ctx.part_world_position(button) for button in buttons]
    for index, button in enumerate(buttons):
        button_aabb = ctx.part_element_world_aabb(button, elem="button_cap")
        ctx.check(
            f"button_{index} starts seated at the front face",
            button_aabb is not None and abs(button_aabb[0][2] - BODY_D) <= 0.0005,
            details=f"button_aabb={button_aabb!r}",
        )

    for index, joint in enumerate(button_joints):
        joint_limits = joint.motion_limits
        pressed_value = BUTTON_TRAVEL if joint_limits is None or joint_limits.upper is None else joint_limits.upper
        with ctx.pose({joint: pressed_value}):
            moved_pos = ctx.part_world_position(buttons[index])
            neighbor_positions = [ctx.part_world_position(buttons[other]) for other in range(3)]

        rest_pos = button_rest_positions[index]
        ctx.check(
            f"button_{index} depresses inward",
            rest_pos is not None and moved_pos is not None and moved_pos[2] < rest_pos[2] - 0.0015,
            details=f"rest={rest_pos}, pressed={moved_pos}",
        )
        stationary_ok = True
        stationary_details = []
        for other in range(3):
            if other == index:
                continue
            rest_other = button_rest_positions[other]
            moved_other = neighbor_positions[other]
            same_place = (
                rest_other is not None
                and moved_other is not None
                and abs(moved_other[2] - rest_other[2]) <= 1e-8
            )
            stationary_ok = stationary_ok and same_place
            stationary_details.append((other, rest_other, moved_other))
        ctx.check(
            f"button_{index} moves independently",
            stationary_ok,
            details=f"neighbors={stationary_details!r}",
        )

    ctx.expect_overlap(
        flap,
        housing,
        axes="x",
        elem_a="flap_panel",
        elem_b="housing_shell",
        min_overlap=0.090,
        name="setup flap spans the thermostat width",
    )

    flap_closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_hinge: FLAP_TRAVEL}):
        flap_open_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    flap_closed_center = _aabb_center(flap_closed_aabb)
    flap_open_center = _aabb_center(flap_open_aabb)
    ctx.check(
        "setup flap opens upward",
        flap_closed_aabb is not None
        and flap_open_aabb is not None
        and flap_open_aabb[0][1] > flap_closed_aabb[0][1] + 0.010
        and flap_open_center is not None
        and flap_closed_center is not None
        and flap_open_center[2] > flap_closed_center[2] + 0.006,
        details=f"closed={flap_closed_aabb!r}, open={flap_open_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
