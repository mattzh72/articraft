from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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


PLATE_LEN = 0.220
PLATE_WID = 0.160
PLATE_THK = 0.018

SUPPORT_GAP = 0.060
SUPPORT_THK = 0.018
SUPPORT_BASE_EMBED = 0.001
SUPPORT_CENTER_Y = SUPPORT_GAP / 2.0 + SUPPORT_THK / 2.0
SUPPORT_PROFILE = (
    (-0.046, 0.000),
    (-0.046, 0.040),
    (-0.029, 0.072),
    (-0.012, 0.092),
    (0.020, 0.092),
    (0.040, 0.070),
    (0.040, 0.000),
)

AXIS_Z = 0.078
BORE_RADIUS = 0.0195
BOSS_RADIUS = 0.030
BOSS_LEN = 0.010

SHAFT_RADIUS = 0.018
SHAFT_LEN = SUPPORT_GAP + 2.0 * (SUPPORT_THK + BOSS_LEN)
FLANGE_RADIUS = 0.036
FLANGE_THK = 0.016
HUB_RADIUS = 0.024
HUB_LEN = 0.028
PILOT_RADIUS = 0.014
PILOT_LEN = 0.004
INDEX_PIN_RADIUS = 0.004
INDEX_PIN_LEN = 0.006
INDEX_PIN_OFFSET_X = 0.022
INDEX_PIN_OFFSET_Z = 0.0
INDEX_PIN_CENTER_Y = FLANGE_THK / 2.0 + INDEX_PIN_LEN / 2.0 - 0.001


def _y_cylinder(radius: float, length: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate(center_xyz)
    )


def _support_extent_y(side: float) -> tuple[float, float]:
    cheek_min = side * SUPPORT_CENTER_Y - SUPPORT_THK / 2.0
    cheek_max = side * SUPPORT_CENTER_Y + SUPPORT_THK / 2.0
    if side > 0.0:
        return cheek_min, cheek_max + BOSS_LEN
    return cheek_min - BOSS_LEN, cheek_max


def _build_base_plate() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(PLATE_LEN, PLATE_WID, PLATE_THK, centered=(True, True, False))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.070, -0.050),
                (-0.070, 0.050),
                (0.070, -0.050),
                (0.070, 0.050),
            ]
        )
        .hole(0.012)
    )


def _build_support(side: float) -> cq.Workplane:
    support = (
        cq.Workplane("XZ")
        .polyline(SUPPORT_PROFILE)
        .close()
        .extrude(SUPPORT_THK / 2.0, both=True)
        .translate((0.0, side * SUPPORT_CENTER_Y, PLATE_THK - SUPPORT_BASE_EMBED))
    )

    boss_outer_face_y = side * (SUPPORT_CENTER_Y + SUPPORT_THK / 2.0)
    boss_center_y = boss_outer_face_y + side * (BOSS_LEN / 2.0)
    boss = _y_cylinder(BOSS_RADIUS, BOSS_LEN, (0.0, boss_center_y, AXIS_Z))

    support_y_min, support_y_max = _support_extent_y(side)
    bore_center_y = 0.5 * (support_y_min + support_y_max)
    bore_len = support_y_max - support_y_min + 0.002
    bore = _y_cylinder(BORE_RADIUS, bore_len, (0.0, bore_center_y, AXIS_Z))

    return support.union(boss).cut(bore)


def _build_spindle_shaft() -> cq.Workplane:
    return _y_cylinder(SHAFT_RADIUS, SHAFT_LEN, (0.0, 0.0, 0.0))


def _build_flange_disk() -> cq.Workplane:
    hub = _y_cylinder(HUB_RADIUS, HUB_LEN, (0.0, 0.0, 0.0))
    flange = _y_cylinder(FLANGE_RADIUS, FLANGE_THK, (0.0, 0.0, 0.0))
    pilot = _y_cylinder(PILOT_RADIUS, PILOT_LEN, (0.0, FLANGE_THK / 2.0 + PILOT_LEN / 2.0, 0.0))

    flange_body = hub.union(flange).union(pilot)

    bolt_points = []
    bolt_circle_r = 0.022
    for angle_deg in (0.0, 120.0, 240.0):
        angle_rad = angle_deg * pi / 180.0
        bolt_points.append((bolt_circle_r * cos(angle_rad), bolt_circle_r * sin(angle_rad)))

    flange_body = (
        flange_body.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(bolt_points)
        .hole(0.008)
    )

    key_flat = (
        cq.Workplane("XZ")
        .rect(0.020, 0.012)
        .extrude(FLANGE_THK + 0.004, both=True)
        .translate((FLANGE_RADIUS - 0.006, 0.0, 0.0))
    )
    return flange_body.cut(key_flat)


def _build_index_pin() -> cq.Workplane:
    return _y_cylinder(
        INDEX_PIN_RADIUS,
        INDEX_PIN_LEN,
        (INDEX_PIN_OFFSET_X, INDEX_PIN_CENTER_Y, INDEX_PIN_OFFSET_Z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_cheek_roll_spindle_head")

    model.material("frame_paint", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("machined_steel", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("marker_black", rgba=(0.15, 0.16, 0.18, 1.0))

    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_build_base_plate(), "spindle_head_base_plate"),
        material="frame_paint",
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(_build_support(-1.0), "spindle_head_left_support"),
        material="frame_paint",
        name="left_support",
    )
    base.visual(
        mesh_from_cadquery(_build_support(1.0), "spindle_head_right_support"),
        material="frame_paint",
        name="right_support",
    )
    base.inertial = Inertial.from_geometry(
        Box((PLATE_LEN, PLATE_WID, AXIS_Z + 0.040)),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (AXIS_Z + 0.040))),
    )

    output = model.part("output_flange")
    output.visual(
        mesh_from_cadquery(_build_spindle_shaft(), "spindle_head_output_shaft"),
        material="machined_steel",
        name="shaft_journal",
    )
    output.visual(
        mesh_from_cadquery(_build_flange_disk(), "spindle_head_output_flange"),
        material="machined_steel",
        name="flange_disk",
    )
    output.visual(
        mesh_from_cadquery(_build_index_pin(), "spindle_head_index_pin"),
        material="marker_black",
        name="index_pin",
    )
    output.inertial = Inertial.from_geometry(
        Box((2.0 * FLANGE_RADIUS, SHAFT_LEN, 2.0 * FLANGE_RADIUS)),
        mass=1.1,
        origin=Origin(),
    )

    model.articulation(
        "base_to_output_flange",
        ArticulationType.REVOLUTE,
        parent=base,
        child=output,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=8.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    part_names = {part.name for part in object_model.parts}
    joint_names = {joint.name for joint in object_model.articulations}
    ctx.check(
        "required parts are present",
        {"base_frame", "output_flange"}.issubset(part_names),
        details=f"parts={sorted(part_names)}",
    )
    ctx.check(
        "required articulation is present",
        {"base_to_output_flange"}.issubset(joint_names),
        details=f"articulations={sorted(joint_names)}",
    )

    base = object_model.get_part("base_frame")
    output = object_model.get_part("output_flange")
    roll = object_model.get_articulation("base_to_output_flange")

    ctx.allow_isolated_part(
        output,
        reason="The spindle runs in relieved cheek bores with intentional bearing clearance, so the rotating part stays support-disconnected while remaining mounted by the roll articulation.",
    )

    axis_ok = all(abs(a - b) < 1e-9 for a, b in zip(roll.axis, (0.0, 1.0, 0.0)))
    limits = roll.motion_limits
    limits_ok = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -3.0
        and limits.upper >= 3.0
    )
    ctx.check(
        "roll joint is aligned to the spindle axis",
        axis_ok and limits_ok,
        details=f"axis={roll.axis}, limits={limits}",
    )

    base_aabb = ctx.part_world_aabb(base)
    grounded = base_aabb is not None and abs(base_aabb[0][2]) <= 1e-6
    ctx.check(
        "base plate is grounded at z=0",
        grounded,
        details=f"base_aabb={base_aabb}",
    )

    with ctx.pose({roll: 0.0}):
        ctx.expect_gap(
            output,
            base,
            axis="y",
            positive_elem="flange_disk",
            negative_elem="left_support",
            min_gap=0.010,
            name="flange clears the left cheek",
        )
        ctx.expect_gap(
            base,
            output,
            axis="y",
            positive_elem="right_support",
            negative_elem="flange_disk",
            min_gap=0.010,
            name="flange clears the right cheek",
        )

    left_aabb = ctx.part_element_world_aabb(base, elem="left_support")
    right_aabb = ctx.part_element_world_aabb(base, elem="right_support")
    flange_aabb = ctx.part_element_world_aabb(output, elem="flange_disk")
    centered_between_cheeks = False
    if left_aabb is not None and right_aabb is not None and flange_aabb is not None:
        left_gap = flange_aabb[0][1] - left_aabb[1][1]
        right_gap = right_aabb[0][1] - flange_aabb[1][1]
        centered_between_cheeks = abs(left_gap - right_gap) <= 0.002
    else:
        left_gap = None
        right_gap = None
    ctx.check(
        "flange sits centered between the split cheeks",
        centered_between_cheeks,
        details=f"left_gap={left_gap}, right_gap={right_gap}",
    )

    def _elem_center(part_obj, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    pin_rest = _elem_center(output, "index_pin")
    with ctx.pose({roll: pi / 2.0}):
        pin_rotated = _elem_center(output, "index_pin")

    pin_moves = (
        pin_rest is not None
        and pin_rotated is not None
        and (
            abs(pin_rotated[0] - pin_rest[0]) > 0.010
            or abs(pin_rotated[2] - pin_rest[2]) > 0.010
        )
    )
    ctx.check(
        "index pin moves when the flange rolls",
        pin_moves,
        details=f"pin_rest={pin_rest}, pin_rotated={pin_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
