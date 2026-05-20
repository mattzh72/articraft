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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


COLUMN_TOP_Z = 2.82
FACE_CENTER_Z = 0.56
FRONT_HOUR_MOUNT_Y = 0.122
FRONT_MINUTE_MOUNT_Y = 0.134
REAR_HOUR_MOUNT_Y = -FRONT_HOUR_MOUNT_Y
REAR_MINUTE_MOUNT_Y = -FRONT_MINUTE_MOUNT_Y


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def _fluted_section(
    z: float,
    *,
    radius: float,
    groove_depth: float,
    flute_count: int = 16,
    samples: int = 96,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for idx in range(samples):
        angle = 2.0 * pi * idx / samples
        groove = 0.5 + 0.5 * cos(flute_count * angle)
        local_radius = radius - groove_depth * groove * groove
        points.append((local_radius * cos(angle), local_radius * sin(angle), z))
    return points


def _add_face_markers(
    housing,
    *,
    side_sign: float,
    face_y: float,
    center_z: float,
    radius: float,
    material,
) -> None:
    for idx in range(12):
        angle = idx * pi / 6.0
        mark_radius = radius * 0.84
        x = mark_radius * sin(angle)
        z = center_z + mark_radius * cos(angle)
        is_quarter = idx % 3 == 0
        housing.visual(
            Box((0.016 if is_quarter else 0.010, 0.004, 0.042 if is_quarter else 0.028)),
            origin=Origin(
                xyz=(x, face_y, z),
                rpy=(0.0, angle, 0.0),
            ),
            material=material,
            name=f"{'front' if side_sign > 0 else 'rear'}_marker_{idx}",
        )


def _add_hand_geometry(
    hand,
    *,
    side_sign: float,
    blade_length: float,
    blade_width: float,
    blade_thickness: float,
    hub_radius: float,
    hub_length: float,
    tip_length: float,
    tail_length: float,
    material,
) -> None:
    y_center = side_sign * hub_length * 0.58
    hand.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(
            xyz=(0.0, side_sign * hub_length * 0.5, 0.0),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=material,
        name="hub",
    )
    hand.visual(
        Box((blade_width, blade_thickness, blade_length)),
        origin=Origin(xyz=(0.0, y_center, blade_length * 0.5)),
        material=material,
        name="blade",
    )
    hand.visual(
        Box((blade_width * 1.85, blade_thickness, tip_length)),
        origin=Origin(xyz=(0.0, y_center, blade_length * 0.70)),
        material=material,
        name="spade_body",
    )
    hand.visual(
        Box((blade_width * 1.10, blade_thickness, blade_width * 1.10)),
        origin=Origin(
            xyz=(0.0, y_center, blade_length * 0.80),
            rpy=(0.0, pi * 0.25, 0.0),
        ),
        material=material,
        name="spade_tip",
    )
    hand.visual(
        Box((blade_width * 0.55, blade_thickness, tail_length)),
        origin=Origin(xyz=(0.0, y_center, -tail_length * 0.5)),
        material=material,
        name="counter_tail",
    )
    hand.visual(
        Sphere(radius=hub_radius * 0.42),
        origin=Origin(xyz=(0.0, y_center, -tail_length)),
        material=material,
        name="counter_ball",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heritage_platform_clock")

    cast_iron = model.material("cast_iron", rgba=(0.14, 0.17, 0.16, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.61, 0.54, 0.34, 1.0))
    dial_enamel = model.material("dial_enamel", rgba=(0.94, 0.93, 0.88, 1.0))
    hand_black = model.material("hand_black", rgba=(0.07, 0.07, 0.06, 1.0))

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.22, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cast_iron,
        name="base_foot",
    )
    column.visual(
        Cylinder(radius=0.16, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=cast_iron,
        name="base_plinth",
    )
    shaft_mesh = section_loft(
        [
            _fluted_section(0.16, radius=0.074, groove_depth=0.008),
            _fluted_section(2.56, radius=0.062, groove_depth=0.007),
        ]
    )
    column.visual(
        mesh_from_geometry(shaft_mesh, "fluted_column_shaft"),
        material=cast_iron,
        name="fluted_shaft",
    )
    column.visual(
        Cylinder(radius=0.094, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=cast_iron,
        name="shaft_base_ring",
    )
    column.visual(
        Cylinder(radius=0.086, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 2.59)),
        material=cast_iron,
        name="shaft_top_ring",
    )
    column.visual(
        Cylinder(radius=0.10, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.68)),
        material=cast_iron,
        name="neck_collar",
    )
    column.visual(
        Cylinder(radius=0.13, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.78)),
        material=aged_brass,
        name="capital_plate",
    )

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=0.055, length=0.13),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=cast_iron,
        name="support_stem",
    )
    housing.visual(
        Cylinder(radius=0.12, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=aged_brass,
        name="lower_collar",
    )
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.330, -0.090),
            (0.352, -0.100),
            (0.388, -0.122),
            (0.405, -0.130),
            (0.405, 0.130),
            (0.388, 0.122),
            (0.352, 0.100),
            (0.330, 0.090),
        ],
        [
            (0.288, -0.088),
            (0.304, -0.096),
            (0.340, -0.118),
            (0.352, -0.124),
            (0.352, 0.124),
            (0.340, 0.118),
            (0.304, 0.096),
            (0.288, 0.088),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    shell.rotate_x(-pi * 0.5).translate(0.0, 0.0, FACE_CENTER_Z)
    housing.visual(
        mesh_from_geometry(shell, "clock_housing_shell"),
        material=cast_iron,
        name="clock_shell",
    )
    housing.visual(
        Cylinder(radius=0.365, length=0.010),
        origin=Origin(
            xyz=(0.0, 0.108, FACE_CENTER_Z),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=dial_enamel,
        name="front_dial",
    )
    housing.visual(
        Cylinder(radius=0.365, length=0.010),
        origin=Origin(
            xyz=(0.0, -0.108, FACE_CENTER_Z),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=dial_enamel,
        name="rear_dial",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(
            xyz=(0.0, 0.117, FACE_CENTER_Z),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=aged_brass,
        name="front_center_boss",
    )
    housing.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(
            xyz=(0.0, -0.117, FACE_CENTER_Z),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=aged_brass,
        name="rear_center_boss",
    )
    housing.visual(
        Cylinder(radius=0.15, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.965)),
        material=aged_brass,
        name="upper_crown",
    )
    housing.visual(
        Sphere(radius=0.045),
        origin=Origin(xyz=(0.0, 0.0, 1.035)),
        material=aged_brass,
        name="finial",
    )

    _add_face_markers(
        housing,
        side_sign=1.0,
        face_y=0.112,
        center_z=FACE_CENTER_Z,
        radius=0.305,
        material=hand_black,
    )
    _add_face_markers(
        housing,
        side_sign=-1.0,
        face_y=-0.112,
        center_z=FACE_CENTER_Z,
        radius=0.305,
        material=hand_black,
    )

    front_hour_hand = model.part("front_hour_hand")
    _add_hand_geometry(
        front_hour_hand,
        side_sign=1.0,
        blade_length=0.185,
        blade_width=0.030,
        blade_thickness=0.005,
        hub_radius=0.023,
        hub_length=0.012,
        tip_length=0.055,
        tail_length=0.060,
        material=hand_black,
    )

    front_minute_hand = model.part("front_minute_hand")
    _add_hand_geometry(
        front_minute_hand,
        side_sign=1.0,
        blade_length=0.275,
        blade_width=0.022,
        blade_thickness=0.004,
        hub_radius=0.016,
        hub_length=0.010,
        tip_length=0.060,
        tail_length=0.080,
        material=hand_black,
    )

    rear_hour_hand = model.part("rear_hour_hand")
    _add_hand_geometry(
        rear_hour_hand,
        side_sign=-1.0,
        blade_length=0.185,
        blade_width=0.030,
        blade_thickness=0.005,
        hub_radius=0.023,
        hub_length=0.012,
        tip_length=0.055,
        tail_length=0.060,
        material=hand_black,
    )

    rear_minute_hand = model.part("rear_minute_hand")
    _add_hand_geometry(
        rear_minute_hand,
        side_sign=-1.0,
        blade_length=0.275,
        blade_width=0.022,
        blade_thickness=0.004,
        hub_radius=0.016,
        hub_length=0.010,
        tip_length=0.060,
        tail_length=0.080,
        material=hand_black,
    )

    model.articulation(
        "column_to_housing",
        ArticulationType.FIXED,
        parent=column,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z)),
    )
    model.articulation(
        "housing_to_front_hour",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=front_hour_hand,
        origin=Origin(xyz=(0.0, FRONT_HOUR_MOUNT_Y, FACE_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0),
    )
    model.articulation(
        "housing_to_front_minute",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=front_minute_hand,
        origin=Origin(xyz=(0.0, FRONT_MINUTE_MOUNT_Y, FACE_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5),
    )
    model.articulation(
        "housing_to_rear_hour",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rear_hour_hand,
        origin=Origin(xyz=(0.0, REAR_HOUR_MOUNT_Y, FACE_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0),
    )
    model.articulation(
        "housing_to_rear_minute",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rear_minute_hand,
        origin=Origin(xyz=(0.0, REAR_MINUTE_MOUNT_Y, FACE_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    column = object_model.get_part("column")
    housing = object_model.get_part("housing")
    front_hour_hand = object_model.get_part("front_hour_hand")
    front_minute_hand = object_model.get_part("front_minute_hand")
    rear_hour_hand = object_model.get_part("rear_hour_hand")
    rear_minute_hand = object_model.get_part("rear_minute_hand")

    front_hour_joint = object_model.get_articulation("housing_to_front_hour")
    front_minute_joint = object_model.get_articulation("housing_to_front_minute")
    rear_hour_joint = object_model.get_articulation("housing_to_rear_hour")
    rear_minute_joint = object_model.get_articulation("housing_to_rear_minute")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        housing,
        column,
        elem_a="support_stem",
        elem_b="capital_plate",
        name="housing stem seats on the column capital",
    )
    ctx.expect_contact(
        front_hour_hand,
        housing,
        elem_a="hub",
        elem_b="front_center_boss",
        name="front hour hand is mounted on the front center arbor",
    )
    ctx.expect_contact(
        front_minute_hand,
        front_hour_hand,
        elem_a="hub",
        elem_b="hub",
        name="front minute hand is stacked concentrically over the hour hand",
    )
    ctx.expect_contact(
        rear_hour_hand,
        housing,
        elem_a="hub",
        elem_b="rear_center_boss",
        name="rear hour hand is mounted on the rear center arbor",
    )
    ctx.expect_contact(
        rear_minute_hand,
        rear_hour_hand,
        elem_a="hub",
        elem_b="hub",
        name="rear minute hand is stacked concentrically over the rear hour hand",
    )
    ctx.expect_gap(
        front_minute_hand,
        front_hour_hand,
        axis="y",
        min_gap=0.005,
        positive_elem="blade",
        negative_elem="blade",
        name="front hand blades sit on distinct stacked planes",
    )
    ctx.expect_gap(
        rear_hour_hand,
        rear_minute_hand,
        axis="y",
        min_gap=0.005,
        positive_elem="blade",
        negative_elem="blade",
        name="rear hand blades sit on distinct stacked planes",
    )

    ctx.check(
        "clock hand axes follow the face normals",
        front_hour_joint.axis == (0.0, 1.0, 0.0)
        and front_minute_joint.axis == (0.0, 1.0, 0.0)
        and rear_hour_joint.axis == (0.0, -1.0, 0.0)
        and rear_minute_joint.axis == (0.0, -1.0, 0.0),
        details=(
            f"front_hour={front_hour_joint.axis}, front_minute={front_minute_joint.axis}, "
            f"rear_hour={rear_hour_joint.axis}, rear_minute={rear_minute_joint.axis}"
        ),
    )

    with ctx.pose({front_minute_joint: 0.0}):
        front_rest = _aabb_center(ctx.part_element_world_aabb(front_minute_hand, elem="blade"))
    with ctx.pose({front_minute_joint: pi * 0.5}):
        front_swept = _aabb_center(ctx.part_element_world_aabb(front_minute_hand, elem="blade"))

    ctx.check(
        "front minute hand sweeps clockwise from twelve toward three",
        front_rest is not None
        and front_swept is not None
        and abs(front_rest[0]) < 0.03
        and front_swept[0] > front_rest[0] + 0.10
        and front_swept[2] < front_rest[2] - 0.08,
        details=f"rest={front_rest}, swept={front_swept}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
