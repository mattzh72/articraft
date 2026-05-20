from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_WIDTH = 6.8
BODY_DEPTH = 4.4
BODY_HEIGHT = 8.8
PARAPET_HEIGHT = 0.8
GATE_WIDTH = 3.2
GATE_SPRING_Z = 2.75
CLOCK_CENTER_Z = 6.45
FRONT_Y = -BODY_DEPTH * 0.5


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _gatehouse_profile(
    *,
    outer_width: float,
    outer_height: float,
    opening_width: float,
    spring_z: float,
    arch_segments: int = 18,
) -> list[tuple[float, float]]:
    half_outer = outer_width * 0.5
    half_open = opening_width * 0.5
    points = [
        (-half_outer, 0.0),
        (-half_open, 0.0),
        (-half_open, spring_z),
    ]
    for i in range(1, arch_segments):
        angle = pi - pi * i / arch_segments
        points.append((half_open * cos(angle), spring_z + half_open * sin(angle)))
    points.extend(
        [
            (half_open, spring_z),
            (half_open, 0.0),
            (half_outer, 0.0),
            (half_outer, outer_height),
            (-half_outer, outer_height),
        ]
    )
    return points


def _hand_profile(
    upper_tip_z: float,
    lower_tail_z: float,
    shaft_half_width: float,
    hub_half_width: float,
    tip_half_width: float,
    shoulder_z: float,
    neck_z: float,
) -> list[tuple[float, float]]:
    return [
        (-shaft_half_width * 0.55, lower_tail_z),
        (-shaft_half_width * 1.10, lower_tail_z * 0.45),
        (-hub_half_width, -0.03),
        (-hub_half_width, 0.03),
        (-shaft_half_width * 1.15, shoulder_z),
        (-shaft_half_width, neck_z),
        (-tip_half_width, upper_tip_z * 0.92),
        (0.0, upper_tip_z),
        (tip_half_width, upper_tip_z * 0.92),
        (shaft_half_width, neck_z),
        (shaft_half_width * 1.15, shoulder_z),
        (hub_half_width, 0.03),
        (hub_half_width, -0.03),
        (shaft_half_width * 1.10, lower_tail_z * 0.45),
        (shaft_half_width * 0.55, lower_tail_z),
    ]


def _axis_span(aabb, axis: str) -> float:
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    return aabb[1][axis_index] - aabb[0][axis_index]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="castle_gatehouse_clock_tower")

    stone = model.material("stone", rgba=(0.56, 0.57, 0.58, 1.0))
    trim_stone = model.material("trim_stone", rgba=(0.70, 0.69, 0.66, 1.0))
    dial_paint = model.material("dial_paint", rgba=(0.84, 0.81, 0.73, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.14, 0.14, 0.15, 1.0))

    tower_body = model.part("tower_body")
    tower_body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT + PARAPET_HEIGHT)),
        mass=6200.0,
        origin=Origin(xyz=(0.0, 0.0, (BODY_HEIGHT + PARAPET_HEIGHT) * 0.5)),
    )

    tower_profile = _gatehouse_profile(
        outer_width=BODY_WIDTH,
        outer_height=BODY_HEIGHT,
        opening_width=GATE_WIDTH,
        spring_z=GATE_SPRING_Z,
    )
    tower_mesh = ExtrudeGeometry(tower_profile, BODY_DEPTH, center=True).rotate_x(pi / 2.0)
    tower_body.visual(_save_mesh("tower_shell", tower_mesh), material=stone, name="tower_shell")

    arch_surround_profile = _gatehouse_profile(
        outer_width=4.55,
        outer_height=5.10,
        opening_width=GATE_WIDTH,
        spring_z=GATE_SPRING_Z,
    )
    arch_surround_mesh = ExtrudeGeometry(arch_surround_profile, 0.34, center=True).rotate_x(pi / 2.0)
    tower_body.visual(
        _save_mesh("arch_surround", arch_surround_mesh),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.165, 0.0)),
        material=trim_stone,
        name="arch_surround",
    )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            tower_body.visual(
                Box((0.44, 0.44, 8.20)),
                origin=Origin(xyz=(x_sign * 3.03, y_sign * 2.02, 4.10)),
                material=trim_stone,
            )

    tower_body.visual(
        Box((BODY_WIDTH + 0.12, 0.32, 0.20)),
        origin=Origin(xyz=(0.0, -2.08, 8.70)),
        material=trim_stone,
    )
    tower_body.visual(
        Box((BODY_WIDTH + 0.12, 0.32, 0.20)),
        origin=Origin(xyz=(0.0, 2.08, 8.70)),
        material=trim_stone,
    )
    tower_body.visual(
        Box((0.32, BODY_DEPTH + 0.12, 0.20)),
        origin=Origin(xyz=(-3.24, 0.0, 8.70)),
        material=trim_stone,
    )
    tower_body.visual(
        Box((0.32, BODY_DEPTH + 0.12, 0.20)),
        origin=Origin(xyz=(3.24, 0.0, 8.70)),
        material=trim_stone,
    )

    for x in (-2.78, -1.67, -0.56, 0.56, 1.67, 2.78):
        tower_body.visual(
            Box((0.78, 0.55, PARAPET_HEIGHT)),
            origin=Origin(xyz=(x, -1.93, BODY_HEIGHT + PARAPET_HEIGHT * 0.5 - 0.01)),
            material=stone,
        )
        tower_body.visual(
            Box((0.78, 0.55, PARAPET_HEIGHT)),
            origin=Origin(xyz=(x, 1.93, BODY_HEIGHT + PARAPET_HEIGHT * 0.5 - 0.01)),
            material=stone,
        )
    for y in (-1.63, 0.0, 1.63):
        tower_body.visual(
            Box((0.55, 0.74, PARAPET_HEIGHT)),
            origin=Origin(xyz=(-3.13, y, BODY_HEIGHT + PARAPET_HEIGHT * 0.5 - 0.01)),
            material=stone,
        )
        tower_body.visual(
            Box((0.55, 0.74, PARAPET_HEIGHT)),
            origin=Origin(xyz=(3.13, y, BODY_HEIGHT + PARAPET_HEIGHT * 0.5 - 0.01)),
            material=stone,
        )

    tower_body.visual(
        Cylinder(radius=1.02, length=0.010),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.005, CLOCK_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_stone,
        name="clock_ring",
    )
    tower_body.visual(
        Cylinder(radius=0.94, length=0.004),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.003, CLOCK_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dial_paint,
        name="clock_dial",
    )
    tower_body.visual(
        Cylinder(radius=0.055, length=0.008),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.004, CLOCK_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="clock_center_boss",
    )
    tower_body.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.004, CLOCK_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="clock_hour_arbor",
    )
    tower_body.visual(
        Box((0.12, 0.012, 0.24)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.006, CLOCK_CENTER_Z + 0.70)),
        material=dark_iron,
    )
    tower_body.visual(
        Box((0.12, 0.012, 0.24)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.006, CLOCK_CENTER_Z - 0.70)),
        material=dark_iron,
    )
    tower_body.visual(
        Box((0.24, 0.012, 0.12)),
        origin=Origin(xyz=(0.70, FRONT_Y + 0.006, CLOCK_CENTER_Z)),
        material=dark_iron,
    )
    tower_body.visual(
        Box((0.24, 0.012, 0.12)),
        origin=Origin(xyz=(-0.70, FRONT_Y + 0.006, CLOCK_CENTER_Z)),
        material=dark_iron,
    )

    hour_hand = model.part("hour_hand")
    hour_hand_mesh = ExtrudeGeometry(
        _hand_profile(
            upper_tip_z=0.63,
            lower_tail_z=-0.18,
            shaft_half_width=0.040,
            hub_half_width=0.105,
            tip_half_width=0.014,
            shoulder_z=0.14,
            neck_z=0.42,
        ),
        0.008,
        center=True,
    ).rotate_x(pi / 2.0)
    hour_hand.visual(
        _save_mesh("hour_blade", hour_hand_mesh),
        origin=Origin(xyz=(0.0, -0.011, 0.0)),
        material=dark_iron,
        name="hour_blade",
    )
    hour_hand.visual(
        Cylinder(radius=0.11, length=0.010),
        origin=Origin(xyz=(0.0, -0.011, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="hour_hub",
    )
    hour_hand.visual(
        Cylinder(radius=0.030, length=0.011),
        origin=Origin(xyz=(0.0, -0.0115, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="hour_forward_arbor",
    )
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.24, 0.012, 0.82)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.011, 0.21)),
    )

    minute_hand = model.part("minute_hand")
    minute_hand_mesh = ExtrudeGeometry(
        _hand_profile(
            upper_tip_z=0.86,
            lower_tail_z=-0.22,
            shaft_half_width=0.028,
            hub_half_width=0.075,
            tip_half_width=0.010,
            shoulder_z=0.10,
            neck_z=0.58,
        ),
        0.006,
        center=True,
    ).rotate_x(pi / 2.0)
    minute_hand.visual(
        _save_mesh("minute_blade", minute_hand_mesh),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=dark_iron,
        name="minute_blade",
    )
    minute_hand.visual(
        Cylinder(radius=0.080, length=0.006),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="minute_hub",
    )
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.18, 0.010, 1.08)),
        mass=14.0,
        origin=Origin(xyz=(0.0, -0.020, 0.27)),
    )

    clock_joint_origin = Origin(xyz=(0.0, FRONT_Y, CLOCK_CENTER_Z))
    model.articulation(
        "tower_to_hour_hand",
        ArticulationType.CONTINUOUS,
        parent=tower_body,
        child=hour_hand,
        origin=clock_joint_origin,
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.5),
    )
    model.articulation(
        "tower_to_minute_hand",
        ArticulationType.CONTINUOUS,
        parent=tower_body,
        child=minute_hand,
        origin=clock_joint_origin,
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
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

    tower_body = object_model.get_part("tower_body")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("tower_to_hour_hand")
    minute_joint = object_model.get_articulation("tower_to_minute_hand")

    same_axis = hour_joint.axis == minute_joint.axis == (0.0, 1.0, 0.0)
    same_origin = hour_joint.origin.xyz == minute_joint.origin.xyz
    continuous_types = (
        hour_joint.articulation_type == ArticulationType.CONTINUOUS
        and minute_joint.articulation_type == ArticulationType.CONTINUOUS
    )
    ctx.check(
        "clock hands are coaxial continuous articulations",
        same_axis and same_origin and continuous_types,
        details=(
            f"hour_type={hour_joint.articulation_type}, minute_type={minute_joint.articulation_type}, "
            f"hour_axis={hour_joint.axis}, minute_axis={minute_joint.axis}, "
            f"hour_origin={hour_joint.origin.xyz}, minute_origin={minute_joint.origin.xyz}"
        ),
    )

    with ctx.pose({hour_joint: 0.0, minute_joint: 0.0}):
        ctx.expect_gap(
            tower_body,
            hour_hand,
            axis="y",
            positive_elem="clock_dial",
            negative_elem="hour_blade",
            min_gap=0.001,
            max_gap=0.03,
            name="hour hand stands proud of the dial",
        )
        ctx.expect_contact(
            tower_body,
            hour_hand,
            elem_a="clock_hour_arbor",
            elem_b="hour_hub",
            name="hour hand mounts on the body arbor",
        )
        ctx.expect_contact(
            hour_hand,
            minute_hand,
            elem_a="hour_forward_arbor",
            elem_b="minute_hub",
            name="minute hand rides on the forward arbor ahead of the hour hand",
        )
        ctx.expect_overlap(
            minute_hand,
            tower_body,
            axes="xz",
            elem_a="minute_blade",
            elem_b="clock_dial",
            min_overlap=0.08,
            name="minute hand lies over the single clock face",
        )

    minute_rest_aabb = ctx.part_element_world_aabb(minute_hand, elem="minute_blade")
    with ctx.pose({minute_joint: pi / 2.0}):
        minute_quarter_turn_aabb = ctx.part_element_world_aabb(minute_hand, elem="minute_blade")

    minute_turn_ok = (
        minute_rest_aabb is not None
        and minute_quarter_turn_aabb is not None
        and _axis_span(minute_rest_aabb, "z") > _axis_span(minute_rest_aabb, "x") * 2.0
        and _axis_span(minute_quarter_turn_aabb, "x") > _axis_span(minute_quarter_turn_aabb, "z") * 2.0
    )
    ctx.check(
        "minute hand quarter turn changes from vertical to horizontal",
        minute_turn_ok,
        details=f"rest={minute_rest_aabb}, quarter_turn={minute_quarter_turn_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
