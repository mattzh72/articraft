from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_T = 0.006
PLATE_H = 0.032
PIN_R = 0.010
PIN_HEAD_R = 0.013
PIN_HEAD_LEN = 0.003

OUTER_PLATE_Y = 0.024
INNER_PLATE_Y = 0.010

OUTER_RUNG_X = 0.014
OUTER_RUNG_Z = 0.010
INNER_RUNG_X = 0.012
INNER_RUNG_Z = 0.010

BRACKET_DEPTH = 0.055
LINK_1_LEN = 0.180
LINK_2_LEN = 0.170
LINK_3_LEN = 0.140
TIP_TAB_LEN = 0.035


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Workplane("XY").box(sx, sy, sz).translate((cx, cy, cz))


def _combine(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _plate_with_holes(
    *,
    start_x: float,
    length: float,
    y_center: float,
    hole_x_positions: tuple[float, ...] = (),
    plate_height: float = PLATE_H,
    plate_thickness: float = PLATE_T,
    hole_radius: float = PIN_R,
) -> cq.Workplane:
    plate = (
        cq.Workplane("XZ")
        .center(start_x + length / 2.0, 0.0)
        .slot2D(length, plate_height)
        .extrude(plate_thickness, both=True)
        .translate((0.0, y_center, 0.0))
    )
    for hole_x in hole_x_positions:
        cutter = (
            cq.Workplane("XZ")
            .center(hole_x, 0.0)
            .circle(hole_radius)
            .extrude(plate_thickness + 0.004, both=True)
            .translate((0.0, y_center, 0.0))
        )
        plate = plate.cut(cutter)
    return plate


def _pin_body() -> cq.Workplane:
    shaft_len = 2.0 * OUTER_PLATE_Y + PLATE_T
    shaft = cq.Workplane("XZ").circle(PIN_R).extrude(shaft_len, both=True)
    left_head = (
        cq.Workplane("XZ")
        .circle(PIN_HEAD_R)
        .extrude(PIN_HEAD_LEN, both=True)
        .translate((0.0, -(shaft_len / 2.0 + PIN_HEAD_LEN / 2.0), 0.0))
    )
    right_head = (
        cq.Workplane("XZ")
        .circle(PIN_HEAD_R)
        .extrude(PIN_HEAD_LEN, both=True)
        .translate((0.0, shaft_len / 2.0 + PIN_HEAD_LEN / 2.0, 0.0))
    )
    return _combine(shaft, left_head, right_head)


def _inner_link_body(length: float, *, tip_tab: bool = False) -> cq.Workplane:
    span_y = 2.0 * INNER_PLATE_Y + PLATE_T
    hole_positions = (0.0,) if tip_tab else (0.0, length)
    solids = [
        _plate_with_holes(
            start_x=0.0,
            length=length,
            y_center=-INNER_PLATE_Y,
            hole_x_positions=hole_positions,
        ),
        _plate_with_holes(
            start_x=0.0,
            length=length,
            y_center=INNER_PLATE_Y,
            hole_x_positions=hole_positions,
        ),
        _box((INNER_RUNG_X, span_y, INNER_RUNG_Z), (0.42 * length, 0.0, 0.0)),
    ]
    if tip_tab:
        solids.extend(
            [
                _box((0.016, span_y, 0.016), (length - 0.010, 0.0, 0.0)),
                _box((TIP_TAB_LEN, 0.014, 0.016), (length + TIP_TAB_LEN / 2.0, 0.0, 0.0)),
            ]
        )
    else:
        solids.append(_box((INNER_RUNG_X, span_y, INNER_RUNG_Z), (0.72 * length, 0.0, 0.0)))
    return _combine(*solids)


def _outer_link_body(length: float) -> cq.Workplane:
    span_y = 2.0 * OUTER_PLATE_Y + PLATE_T
    return _combine(
        _plate_with_holes(
            start_x=0.0,
            length=length,
            y_center=-OUTER_PLATE_Y,
            hole_x_positions=(0.0, length),
        ),
        _plate_with_holes(
            start_x=0.0,
            length=length,
            y_center=OUTER_PLATE_Y,
            hole_x_positions=(0.0, length),
        ),
        _box((OUTER_RUNG_X, span_y, OUTER_RUNG_Z), (0.36 * length, 0.0, 0.0)),
        _box((OUTER_RUNG_X, span_y, OUTER_RUNG_Z), (0.70 * length, 0.0, 0.0)),
    )


def _root_bracket_body() -> cq.Workplane:
    cheek_left = _plate_with_holes(
        start_x=-BRACKET_DEPTH,
        length=BRACKET_DEPTH,
        y_center=-OUTER_PLATE_Y,
        hole_x_positions=(0.0,),
    )
    cheek_right = _plate_with_holes(
        start_x=-BRACKET_DEPTH,
        length=BRACKET_DEPTH,
        y_center=OUTER_PLATE_Y,
        hole_x_positions=(0.0,),
    )
    back_plate = _box((0.010, 0.074, 0.090), (-BRACKET_DEPTH - 0.005, 0.0, -0.012))
    rear_web = _box((0.020, 0.060, 0.024), (-0.045, 0.0, -0.032))
    lower_bridge = _box((0.022, 0.060, 0.020), (-0.020, 0.0, -0.032))
    foot_pad = _box((0.032, 0.060, 0.014), (-0.034, 0.0, -0.044))
    return _combine(cheek_left, cheek_right, back_plate, rear_web, lower_bridge, foot_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_frame_lever_chain")

    model.material("bracket_paint", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("steel_link", rgba=(0.56, 0.59, 0.63, 1.0))
    model.material("pin_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    model.material("tip_zinc", rgba=(0.72, 0.74, 0.76, 1.0))

    root_bracket = model.part("root_bracket")
    root_bracket.visual(
        mesh_from_cadquery(_root_bracket_body(), "root_bracket"),
        material="bracket_paint",
        name="body",
    )
    root_bracket.inertial = Inertial.from_geometry(
        Box((0.080, 0.074, 0.090)),
        mass=2.8,
        origin=Origin(xyz=(-0.045, 0.0, -0.012)),
    )

    pin_1 = model.part("pin_1")
    pin_1.visual(mesh_from_cadquery(_pin_body(), "pin_1"), material="pin_dark", name="body")
    pin_1.inertial = Inertial.from_geometry(
        Box((0.026, 0.060, 0.026)),
        mass=0.12,
        origin=Origin(),
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(_inner_link_body(LINK_1_LEN), "link_1"),
        material="steel_link",
        name="body",
    )
    link_1.inertial = Inertial.from_geometry(
        Box((LINK_1_LEN, 0.028, 0.032)),
        mass=1.0,
        origin=Origin(xyz=(LINK_1_LEN / 2.0, 0.0, 0.0)),
    )

    pin_2 = model.part("pin_2")
    pin_2.visual(mesh_from_cadquery(_pin_body(), "pin_2"), material="pin_dark", name="body")
    pin_2.inertial = Inertial.from_geometry(
        Box((0.026, 0.060, 0.026)),
        mass=0.12,
        origin=Origin(),
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(_outer_link_body(LINK_2_LEN), "link_2"),
        material="steel_link",
        name="body",
    )
    link_2.inertial = Inertial.from_geometry(
        Box((LINK_2_LEN, 0.054, 0.032)),
        mass=1.25,
        origin=Origin(xyz=(LINK_2_LEN / 2.0, 0.0, 0.0)),
    )

    pin_3 = model.part("pin_3")
    pin_3.visual(mesh_from_cadquery(_pin_body(), "pin_3"), material="pin_dark", name="body")
    pin_3.inertial = Inertial.from_geometry(
        Box((0.026, 0.060, 0.026)),
        mass=0.12,
        origin=Origin(),
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(_inner_link_body(LINK_3_LEN, tip_tab=True), "link_3"),
        material="tip_zinc",
        name="body",
    )
    link_3.inertial = Inertial.from_geometry(
        Box((LINK_3_LEN + TIP_TAB_LEN, 0.028, 0.032)),
        mass=0.9,
        origin=Origin(xyz=((LINK_3_LEN + TIP_TAB_LEN) / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "root_to_pin1",
        ArticulationType.FIXED,
        parent=root_bracket,
        child=pin_1,
        origin=Origin(),
    )
    model.articulation(
        "pin1_to_link1",
        ArticulationType.REVOLUTE,
        parent=pin_1,
        child=link_1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=-0.35, upper=1.20),
    )
    model.articulation(
        "link1_to_pin2",
        ArticulationType.FIXED,
        parent=link_1,
        child=pin_2,
        origin=Origin(xyz=(LINK_1_LEN, 0.0, 0.0)),
    )
    model.articulation(
        "pin2_to_link2",
        ArticulationType.REVOLUTE,
        parent=pin_2,
        child=link_2,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.8, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "link2_to_pin3",
        ArticulationType.FIXED,
        parent=link_2,
        child=pin_3,
        origin=Origin(xyz=(LINK_2_LEN, 0.0, 0.0)),
    )
    model.articulation(
        "pin3_to_link3",
        ArticulationType.REVOLUTE,
        parent=pin_3,
        child=link_3,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.30, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_bracket = object_model.get_part("root_bracket")
    pin_1 = object_model.get_part("pin_1")
    link_1 = object_model.get_part("link_1")
    pin_2 = object_model.get_part("pin_2")
    link_2 = object_model.get_part("link_2")
    pin_3 = object_model.get_part("pin_3")
    link_3 = object_model.get_part("link_3")

    joint_1 = object_model.get_articulation("pin1_to_link1")
    joint_2 = object_model.get_articulation("pin2_to_link2")
    joint_3 = object_model.get_articulation("pin3_to_link3")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        root_bracket,
        pin_1,
        reason="Grounded clevis bracket captures the first hinge pin inside its bored cheeks.",
    )
    ctx.allow_overlap(
        pin_1,
        link_1,
        reason="Hinge pin passes through the first link's bored end plates as an intended journal fit.",
    )
    ctx.allow_overlap(
        link_1,
        pin_2,
        reason="Second hinge pin is intentionally seated through link_1 end bores.",
    )
    ctx.allow_overlap(
        pin_2,
        link_2,
        reason="Second hinge pin passes through link_2 clevis cheeks as an intended journal fit.",
    )
    ctx.allow_overlap(
        link_2,
        pin_3,
        reason="Third hinge pin is intentionally seated through link_2 end bores.",
    )
    ctx.allow_overlap(
        pin_3,
        link_3,
        reason="Third hinge pin passes through the terminal link's bored end plates as an intended journal fit.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "parallel_joint_axes",
        joint_1.axis == (0.0, -1.0, 0.0)
        and joint_2.axis == (0.0, -1.0, 0.0)
        and joint_3.axis == (0.0, -1.0, 0.0),
        details=f"axes: {joint_1.axis}, {joint_2.axis}, {joint_3.axis}",
    )

    with ctx.pose({joint_1: 0.0, joint_2: 0.0, joint_3: 0.0}):
        ctx.expect_contact(root_bracket, pin_1, name="root_bracket_carries_pin1")
        ctx.expect_contact(pin_1, link_1, name="pin1_supports_link1")
        ctx.expect_contact(link_1, pin_2, name="link1_carries_pin2")
        ctx.expect_contact(pin_2, link_2, name="pin2_supports_link2")
        ctx.expect_contact(link_2, pin_3, name="link2_carries_pin3")
        ctx.expect_contact(pin_3, link_3, name="pin3_supports_link3")

        link_3_aabb = ctx.part_world_aabb(link_3)
        link_2_aabb = ctx.part_world_aabb(link_2)
        has_tip_tab = (
            link_3_aabb is not None
            and link_2_aabb is not None
            and (link_3_aabb[1][0] - link_3_aabb[0][0]) > LINK_3_LEN + 0.020
        )
        ctx.check(
            "terminal_link_has_small_tip_tab",
            has_tip_tab,
            details=f"link_3_aabb={link_3_aabb}, link_2_aabb={link_2_aabb}",
        )

    with ctx.pose({joint_1: 0.55, joint_2: 0.40, joint_3: 0.30}):
        raised_link3_aabb = ctx.part_world_aabb(link_3)

    with ctx.pose({joint_1: 0.0, joint_2: 0.0, joint_3: 0.0}):
        rest_link3_aabb = ctx.part_world_aabb(link_3)

    lifts_up = (
        raised_link3_aabb is not None
        and rest_link3_aabb is not None
        and raised_link3_aabb[0][2] > rest_link3_aabb[0][2] + 0.080
    )
    ctx.check(
        "positive_joint_motion_lifts_tip",
        lifts_up,
        details=f"rest={rest_link3_aabb}, raised={raised_link3_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
