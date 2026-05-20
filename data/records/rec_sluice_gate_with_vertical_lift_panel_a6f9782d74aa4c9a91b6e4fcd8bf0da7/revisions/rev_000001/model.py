from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OPENING_W = 0.42
OPENING_H = 0.56
FRAME_OUTER_W = 0.64
FRAME_FRONT_T = 0.06
FRAME_OUTER_H = 0.90
FRAME_CENTER_Z = 0.33
GUIDE_W = 0.10
GUIDE_D = 0.12
GUIDE_H = 0.96
GUIDE_CENTER_Z = 0.33
GUIDE_CENTER_Y = -0.04
TOP_DECK_W = 0.34
TOP_DECK_D = 0.14
TOP_DECK_T = 0.10
TOP_DECK_Z = 0.81

PANEL_W = 0.46
PANEL_T = 0.018
PANEL_H = 0.62
PANEL_TRAVEL = 0.28
STEM_R = 0.013
STEM_L = 0.34
PANEL_CENTER_Y = -0.048

HOUSING_W = 0.24
HOUSING_D = 0.18
HOUSING_BASE_T = 0.014
HOUSING_BODY_H = 0.14
HOUSING_TOP_H = 0.03
HOUSING_WHEEL_Z = 0.106
HOUSING_BOSS_R = 0.020
HOUSING_BOSS_L = 0.036
WHEEL_MOUNT_Y = (HOUSING_D / 2.0) + HOUSING_BOSS_L - 0.008

DOOR_T = 0.010
DOOR_W = 0.092
DOOR_H = 0.104
HINGE_R = 0.006
DOOR_HINGE_X = (HOUSING_W / 2.0) + 0.010
DOOR_HINGE_Y = -0.058
DOOR_BASE_Z = 0.032

WHEEL_R_OUTER = 0.095
WHEEL_R_INNER = 0.078
WHEEL_R_HUB = 0.020


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _frame_shape() -> cq.Workplane:
    frame = _box((FRAME_OUTER_W, FRAME_FRONT_T, FRAME_OUTER_H), (0.0, 0.0, FRAME_CENTER_Z))
    frame = frame.cut(_box((OPENING_W, FRAME_FRONT_T + 0.02, OPENING_H), (0.0, 0.0, OPENING_H / 2.0)))

    guide_offset_x = (OPENING_W / 2.0) + (GUIDE_W / 2.0)
    for direction in (-1.0, 1.0):
        guide = _box((GUIDE_W, GUIDE_D, GUIDE_H), (direction * guide_offset_x, GUIDE_CENTER_Y, GUIDE_CENTER_Z))
        slot = _box(
            (0.036, 0.070, 0.86),
            (direction * ((PANEL_W / 2.0) - 0.002), PANEL_CENTER_Y, 0.33),
        )
        frame = frame.union(guide).cut(slot)

    top_deck = _box((TOP_DECK_W, TOP_DECK_D, TOP_DECK_T), (0.0, PANEL_CENTER_Y, TOP_DECK_Z))
    stem_slot = _box((0.050, 0.050, 0.36), (0.0, PANEL_CENTER_Y, 0.69))

    return frame.union(top_deck).cut(stem_slot)


def _gate_panel_shape() -> cq.Workplane:
    panel = _box((PANEL_W, PANEL_T, PANEL_H), (0.0, 0.0, PANEL_H / 2.0))
    panel = panel.union(_box((0.36, 0.010, 0.42), (0.0, 0.010, 0.30)))
    panel = panel.union(_box((0.16, 0.036, 0.050), (0.0, 0.0, PANEL_H + 0.020)))
    stem = cq.Workplane("XY").circle(STEM_R).extrude(STEM_L).translate((0.0, 0.0, PANEL_H))
    panel = panel.union(stem)
    panel = panel.union(_box((0.080, 0.028, 0.030), (0.0, 0.0, PANEL_H - 0.005)))
    return panel


def _housing_shape() -> cq.Workplane:
    housing = _box((0.18, 0.14, HOUSING_BASE_T), (0.0, 0.0, HOUSING_BASE_T / 2.0))
    hood = (
        cq.Workplane("XY")
        .rect(HOUSING_W, HOUSING_D)
        .workplane(offset=0.105)
        .rect(0.17, 0.11)
        .loft(combine=True)
        .translate((0.0, 0.0, HOUSING_BASE_T))
    )
    housing = housing.union(hood)

    front_neck = (
        cq.Workplane("XZ")
        .circle(0.028)
        .extrude(0.040)
        .translate((0.0, 0.086, HOUSING_WHEEL_Z))
    )
    front_boss = (
        cq.Workplane("XZ")
        .circle(HOUSING_BOSS_R)
        .extrude(HOUSING_BOSS_L)
        .translate((0.0, WHEEL_MOUNT_Y, HOUSING_WHEEL_Z))
    )
    stem_bore = cq.Workplane("XY").circle(0.022).extrude(0.16)
    door_recess = _box((0.014, 0.098, 0.108), (0.117, -0.010, 0.084))
    hinge_bracket = _box((0.016, 0.016, 0.104), (0.122, DOOR_HINGE_Y, DOOR_BASE_Z + (DOOR_H / 2.0)))
    lower_knuckle = cq.Workplane("XY").circle(HINGE_R).extrude(0.032).translate((DOOR_HINGE_X, DOOR_HINGE_Y, 0.002))
    upper_knuckle = cq.Workplane("XY").circle(HINGE_R).extrude(0.032).translate((DOOR_HINGE_X, DOOR_HINGE_Y, 0.070))

    return housing.union(front_neck).union(front_boss).union(hinge_bracket).union(lower_knuckle).union(upper_knuckle).cut(stem_bore).cut(door_recess)


def _inspection_door_shape() -> cq.Workplane:
    door = _box((DOOR_T, DOOR_W, DOOR_H), (0.008, DOOR_W / 2.0, DOOR_H / 2.0))
    door = door.union(_box((0.006, 0.016, 0.040), (0.003, 0.008, 0.052)))
    door = door.union(cq.Workplane("XY").circle(HINGE_R).extrude(0.036).translate((0.0, 0.0, 0.034)))
    door = door.union(_box((0.006, 0.022, 0.030), (0.012, DOOR_W - 0.012, 0.052)))
    return door


def _handwheel_shape() -> cq.Workplane:
    hub = cq.Workplane("XZ").circle(WHEEL_R_HUB).extrude(0.022).translate((0.0, 0.022, 0.0))
    rim = (
        cq.Workplane("XZ")
        .circle(WHEEL_R_OUTER)
        .circle(WHEEL_R_INNER)
        .extrude(0.012)
        .translate((0.0, 0.022, 0.0))
    )
    vertical_spoke = _box((0.018, 0.012, 0.164), (0.0, 0.016, 0.0))
    horizontal_spoke = _box((0.164, 0.012, 0.018), (0.0, 0.016, 0.0))
    return hub.union(rim).union(vertical_spoke).union(horizontal_spoke)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="irrigation_sluice_gate")

    model.material("galvanized_steel", rgba=(0.62, 0.67, 0.72, 1.0))
    model.material("gate_steel", rgba=(0.34, 0.40, 0.45, 1.0))
    model.material("housing_green", rgba=(0.33, 0.46, 0.33, 1.0))
    model.material("wheel_black", rgba=(0.12, 0.12, 0.12, 1.0))
    model.material("door_gray", rgba=(0.70, 0.72, 0.74, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "sluice_frame"),
        material="galvanized_steel",
        name="frame_body",
    )

    gate_panel = model.part("gate_panel")
    gate_panel.visual(
        mesh_from_cadquery(_gate_panel_shape(), "gate_panel"),
        material="gate_steel",
        name="gate_body",
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "operator_housing"),
        material="housing_green",
        name="housing_shell",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_handwheel_shape(), "handwheel"),
        material="wheel_black",
        name="wheel_body",
    )

    inspection_door = model.part("inspection_door")
    inspection_door.visual(
        mesh_from_cadquery(_inspection_door_shape(), "inspection_door"),
        material="door_gray",
        name="door_panel",
    )

    model.articulation(
        "gate_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate_panel,
        origin=Origin(xyz=(0.0, PANEL_CENTER_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=PANEL_TRAVEL, effort=250.0, velocity=0.12),
    )

    model.articulation(
        "frame_to_housing",
        ArticulationType.FIXED,
        parent=frame,
        child=housing,
        origin=Origin(xyz=(0.0, PANEL_CENTER_Y, TOP_DECK_Z + (TOP_DECK_T / 2.0))),
    )

    model.articulation(
        "wheel_input",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=handwheel,
        origin=Origin(xyz=(0.0, WHEEL_MOUNT_Y, HOUSING_WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=8.0),
    )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=inspection_door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_BASE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=2.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate_panel = object_model.get_part("gate_panel")
    housing = object_model.get_part("housing")
    handwheel = object_model.get_part("handwheel")
    inspection_door = object_model.get_part("inspection_door")
    gate_lift = object_model.get_articulation("gate_lift")
    wheel_input = object_model.get_articulation("wheel_input")
    door_hinge = object_model.get_articulation("housing_to_door")
    limits = gate_lift.motion_limits

    ctx.expect_origin_distance(gate_panel, frame, axes="x", max_dist=0.001, name="gate stays centered in the frame")
    ctx.expect_overlap(
        gate_panel,
        frame,
        axes="x",
        min_overlap=0.42,
        elem_a="gate_body",
        elem_b="frame_body",
        name="gate spans the framed opening width",
    )

    if limits is not None and limits.upper is not None:
        rest_pos = ctx.part_world_position(gate_panel)
        with ctx.pose({gate_lift: limits.upper}):
            ctx.expect_overlap(
                gate_panel,
                frame,
                axes="z",
                elem_a="gate_body",
                elem_b="frame_body",
                min_overlap=0.28,
                name="lifted gate remains retained in the guides",
            )
            raised_pos = ctx.part_world_position(gate_panel)
        ctx.check(
            "gate rises upward",
            rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.20,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )

    ctx.expect_gap(
        housing,
        frame,
        axis="z",
        elem_a="housing_shell",
        elem_b="frame_body",
        max_gap=0.001,
        max_penetration=0.0,
        name="operator housing seats on the top crossmember",
    )
    ctx.expect_contact(
        handwheel,
        housing,
        elem_a="wheel_body",
        elem_b="housing_shell",
        name="handwheel remains supported by the gearbox boss",
    )
    ctx.expect_contact(
        inspection_door,
        housing,
        elem_a="door_panel",
        elem_b="housing_shell",
        name="inspection door stays supported on its side hinge",
    )
    ctx.allow_overlap(
        housing,
        inspection_door,
        elem_a="housing_shell",
        elem_b="door_panel",
        reason="The inspection flap uses simplified interleaved hinge knuckles that intentionally share the hinge pin volume at the housing side.",
    )

    closed_door_aabb = ctx.part_world_aabb(inspection_door)
    with ctx.pose({door_hinge: 1.1, wheel_input: 1.2}):
        open_door_aabb = ctx.part_world_aabb(inspection_door)
    ctx.check(
        "inspection door opens outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.03,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
