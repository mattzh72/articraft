from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="self_erecting_mini_tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(0.88, 0.76, 0.18, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.23, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    tire = model.material("tire", rgba=(0.12, 0.12, 0.13, 1.0))
    counterweight = model.material("counterweight", rgba=(0.48, 0.49, 0.50, 1.0))

    base_carriage = model.part("base_carriage")
    base_carriage.visual(
        Box((1.56, 0.54, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=dark_grey,
        name="deck",
    )
    base_carriage.visual(
        Box((1.82, 0.14, 0.14)),
        origin=Origin(xyz=(0.0, 0.27, 0.24)),
        material=dark_grey,
    )
    base_carriage.visual(
        Box((1.82, 0.14, 0.14)),
        origin=Origin(xyz=(0.0, -0.27, 0.24)),
        material=dark_grey,
    )
    base_carriage.visual(
        Box((0.68, 0.68, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=dark_grey,
    )
    for sx in (-0.20, 0.20):
        for sy in (-0.20, 0.20):
            base_carriage.visual(
                Box((0.08, 0.08, 0.10)),
                origin=Origin(xyz=(sx, sy, 0.58)),
                material=steel,
            )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            wheel_x = x_sign * 0.56
            wheel_y = y_sign * 0.40
            base_carriage.visual(
                Cylinder(radius=0.17, length=0.07),
                origin=Origin(xyz=(wheel_x, wheel_y, 0.17), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=tire,
            )
            base_carriage.visual(
                Cylinder(radius=0.05, length=0.13),
                origin=Origin(
                    xyz=(wheel_x, y_sign * 0.305, 0.17), rpy=(math.pi / 2.0, 0.0, 0.0)
                ),
                material=steel,
            )
            base_carriage.visual(
                Box((0.12, 0.12, 0.14)),
                origin=Origin(xyz=(wheel_x, y_sign * 0.24, 0.19)),
                material=dark_grey,
            )
            base_carriage.visual(
                Box((0.22, 0.22, 0.12)),
                origin=Origin(xyz=(x_sign * 0.69, y_sign * 0.34, 0.06)),
                material=steel,
            )
            base_carriage.visual(
                Cylinder(radius=0.05, length=0.06),
                origin=Origin(xyz=(x_sign * 0.69, y_sign * 0.34, 0.15)),
                material=steel,
            )

    _add_member(base_carriage, (-0.78, -0.12, 0.32), (-1.08, 0.0, 0.24), 0.025, steel)
    _add_member(base_carriage, (-0.78, 0.12, 0.32), (-1.08, 0.0, 0.24), 0.025, steel)
    base_carriage.visual(
        Cylinder(radius=0.06, length=0.05),
        origin=Origin(xyz=(-1.12, 0.0, 0.24), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
    )

    sleeve_bottom_z = 0.52
    sleeve_height = 1.18
    sleeve_top_z = sleeve_bottom_z + sleeve_height

    for sx in (-0.19, 0.19):
        for sy in (-0.19, 0.19):
            base_carriage.visual(
                Box((0.04, 0.04, sleeve_height)),
                origin=Origin(xyz=(sx, sy, sleeve_bottom_z + sleeve_height * 0.5)),
                material=crane_yellow,
            )

    for z in (0.56, 0.90, 1.24, 1.58):
        base_carriage.visual(
            Box((0.42, 0.02, 0.02)),
            origin=Origin(xyz=(0.0, 0.19, z)),
            material=steel,
        )
        base_carriage.visual(
            Box((0.42, 0.02, 0.02)),
            origin=Origin(xyz=(0.0, -0.19, z)),
            material=steel,
        )
        base_carriage.visual(
            Box((0.02, 0.42, 0.02)),
            origin=Origin(xyz=(0.19, 0.0, z)),
            material=steel,
        )
        base_carriage.visual(
            Box((0.02, 0.42, 0.02)),
            origin=Origin(xyz=(-0.19, 0.0, z)),
            material=steel,
        )

    base_carriage.visual(
        Box((0.02, 0.34, 1.02)),
        origin=Origin(xyz=(0.16, 0.0, 1.11)),
        material=steel,
        name="guide_rail_x_positive",
    )
    base_carriage.visual(
        Box((0.02, 0.34, 1.02)),
        origin=Origin(xyz=(-0.16, 0.0, 1.11)),
        material=steel,
    )
    base_carriage.visual(
        Box((0.34, 0.02, 1.02)),
        origin=Origin(xyz=(0.0, 0.16, 1.11)),
        material=steel,
    )
    base_carriage.visual(
        Box((0.34, 0.02, 1.02)),
        origin=Origin(xyz=(0.0, -0.16, 1.11)),
        material=steel,
    )
    base_carriage.inertial = Inertial.from_geometry(
        Box((2.24, 1.02, 1.78)),
        mass=24.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.89)),
    )

    telescoping_mast = model.part("telescoping_mast")
    inner_mast_height = 1.46
    for sx in (-0.13, 0.13):
        for sy in (-0.13, 0.13):
            telescoping_mast.visual(
                Box((0.03, 0.03, inner_mast_height)),
                origin=Origin(xyz=(sx, sy, inner_mast_height * 0.5)),
                material=crane_yellow,
            )

    for z in (0.08, 0.48, 0.88, 1.28):
        telescoping_mast.visual(
            Box((0.29, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, 0.13, z)),
            material=steel,
        )
        telescoping_mast.visual(
            Box((0.29, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, -0.13, z)),
            material=steel,
        )
        telescoping_mast.visual(
            Box((0.018, 0.29, 0.018)),
            origin=Origin(xyz=(0.13, 0.0, z)),
            material=steel,
        )
        telescoping_mast.visual(
            Box((0.018, 0.29, 0.018)),
            origin=Origin(xyz=(-0.13, 0.0, z)),
            material=steel,
        )

    telescoping_mast.visual(
        Box((0.01, 0.28, 0.75)),
        origin=Origin(xyz=(0.145, 0.0, 0.38)),
        material=steel,
        name="guide_shoe_x_positive",
    )
    telescoping_mast.visual(
        Box((0.01, 0.28, 0.75)),
        origin=Origin(xyz=(-0.145, 0.0, 0.38)),
        material=steel,
    )
    telescoping_mast.visual(
        Box((0.28, 0.01, 0.75)),
        origin=Origin(xyz=(0.0, 0.145, 0.38)),
        material=steel,
    )
    telescoping_mast.visual(
        Box((0.28, 0.01, 0.75)),
        origin=Origin(xyz=(0.0, -0.145, 0.38)),
        material=steel,
    )
    telescoping_mast.visual(
        Box((0.30, 0.30, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 1.445)),
        material=dark_grey,
    )
    telescoping_mast.visual(
        Box((0.22, 0.22, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.48)),
        material=steel,
        name="mast_head_plate",
    )
    telescoping_mast.inertial = Inertial.from_geometry(
        Box((0.36, 0.36, 1.52)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
    )

    jib_assembly = model.part("jib_assembly")
    jib_assembly.visual(
        Cylinder(radius=0.18, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=dark_grey,
        name="slew_ring",
    )
    jib_assembly.visual(
        Box((0.32, 0.24, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark_grey,
    )
    jib_assembly.visual(
        Box((2.18, 0.035, 0.05)),
        origin=Origin(xyz=(1.25, 0.09, 0.05)),
        material=steel,
        name="left_track",
    )
    jib_assembly.visual(
        Box((2.18, 0.035, 0.05)),
        origin=Origin(xyz=(1.25, -0.09, 0.05)),
        material=steel,
        name="right_track",
    )
    jib_assembly.visual(
        Box((0.74, 0.03, 0.045)),
        origin=Origin(xyz=(-0.50, 0.08, 0.05)),
        material=steel,
    )
    jib_assembly.visual(
        Box((0.74, 0.03, 0.045)),
        origin=Origin(xyz=(-0.50, -0.08, 0.05)),
        material=steel,
    )

    front_top_points = (
        (0.10, 0.0, 0.36),
        (0.62, 0.0, 0.34),
        (1.16, 0.0, 0.31),
        (1.74, 0.0, 0.27),
        (2.32, 0.0, 0.22),
    )
    left_bottom_points = tuple((x, 0.09, 0.05) for x, _, _ in front_top_points)
    right_bottom_points = tuple((x, -0.09, 0.05) for x, _, _ in front_top_points)

    for i in range(len(front_top_points) - 1):
        _add_member(
            jib_assembly,
            front_top_points[i],
            front_top_points[i + 1],
            0.011,
            crane_yellow,
        )
        _add_member(
            jib_assembly,
            left_bottom_points[i],
            left_bottom_points[i + 1],
            0.007,
            crane_yellow,
        )
        _add_member(
            jib_assembly,
            right_bottom_points[i],
            right_bottom_points[i + 1],
            0.007,
            crane_yellow,
        )

    for upper, left, right in zip(front_top_points, left_bottom_points, right_bottom_points):
        _add_member(jib_assembly, left, right, 0.005, steel)
        _add_member(jib_assembly, left, upper, 0.0055, crane_yellow)
        _add_member(jib_assembly, right, upper, 0.0055, crane_yellow)

    for i in range(len(front_top_points) - 1):
        if i % 2 == 0:
            _add_member(
                jib_assembly,
                left_bottom_points[i],
                front_top_points[i + 1],
                0.0045,
                steel,
            )
            _add_member(
                jib_assembly,
                right_bottom_points[i],
                front_top_points[i + 1],
                0.0045,
                steel,
            )
        else:
            _add_member(
                jib_assembly,
                front_top_points[i],
                left_bottom_points[i + 1],
                0.0045,
                steel,
            )
            _add_member(
                jib_assembly,
                front_top_points[i],
                right_bottom_points[i + 1],
                0.0045,
                steel,
            )

    counter_top = ((-0.08, 0.0, 0.33), (-0.42, 0.0, 0.28), (-0.78, 0.0, 0.18))
    counter_left = tuple((x, 0.08, 0.05) for x, _, _ in counter_top)
    counter_right = tuple((x, -0.08, 0.05) for x, _, _ in counter_top)
    for i in range(len(counter_top) - 1):
        _add_member(jib_assembly, counter_top[i], counter_top[i + 1], 0.010, crane_yellow)
        _add_member(jib_assembly, counter_left[i], counter_left[i + 1], 0.0065, crane_yellow)
        _add_member(jib_assembly, counter_right[i], counter_right[i + 1], 0.0065, crane_yellow)
    for upper, left, right in zip(counter_top, counter_left, counter_right):
        _add_member(jib_assembly, left, right, 0.005, steel)
        _add_member(jib_assembly, left, upper, 0.005, crane_yellow)
        _add_member(jib_assembly, right, upper, 0.005, crane_yellow)

    jib_assembly.visual(
        Box((0.30, 0.18, 0.18)),
        origin=Origin(xyz=(-0.62, 0.0, 0.16)),
        material=counterweight,
    )
    jib_assembly.visual(
        Box((0.18, 0.18, 0.16)),
        origin=Origin(xyz=(-0.82, 0.0, 0.15)),
        material=counterweight,
    )
    _add_member(jib_assembly, (0.00, -0.08, 0.14), (0.00, 0.08, 0.14), 0.03, steel)
    _add_member(jib_assembly, (-0.02, 0.0, 0.15), (0.62, 0.0, 0.34), 0.004, steel)
    _add_member(jib_assembly, (-0.02, 0.0, 0.15), (-0.42, 0.0, 0.28), 0.004, steel)
    jib_assembly.inertial = Inertial.from_geometry(
        Box((3.20, 0.40, 0.42)),
        mass=7.5,
        origin=Origin(xyz=(0.76, 0.0, 0.18)),
    )

    trolley = model.part("trolley")
    trolley.visual(
        Box((0.08, 0.03, 0.02)),
        origin=Origin(xyz=(0.0, 0.09, -0.01)),
        material=steel,
        name="left_roller_pad",
    )
    trolley.visual(
        Box((0.08, 0.03, 0.02)),
        origin=Origin(xyz=(0.0, -0.09, -0.01)),
        material=steel,
        name="right_roller_pad",
    )
    trolley.visual(
        Box((0.18, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.07)),
        material=dark_grey,
    )
    trolley.visual(
        Box((0.12, 0.12, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.15)),
        material=crane_yellow,
    )
    trolley.visual(
        Cylinder(radius=0.02, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, -0.115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    trolley.visual(
        Cylinder(radius=0.012, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, -0.245)),
        material=steel,
    )
    trolley.visual(
        Box((0.09, 0.09, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, -0.35)),
        material=crane_yellow,
    )
    trolley.inertial = Inertial.from_geometry(
        Box((0.20, 0.18, 0.46)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, -0.17)),
    )

    model.articulation(
        "mast_extension",
        ArticulationType.PRISMATIC,
        parent=base_carriage,
        child=telescoping_mast,
        origin=Origin(xyz=(0.0, 0.0, sleeve_bottom_z + 0.01)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.18, lower=0.0, upper=0.72),
    )
    model.articulation(
        "jib_slew",
        ArticulationType.REVOLUTE,
        parent=telescoping_mast,
        child=jib_assembly,
        origin=Origin(xyz=(0.0, 0.0, 1.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0, velocity=0.45, lower=-math.pi, upper=math.pi
        ),
    )
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent=jib_assembly,
        child=trolley,
        origin=Origin(xyz=(0.32, 0.0, 0.025)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.8, lower=0.0, upper=1.74),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_carriage = object_model.get_part("base_carriage")
    telescoping_mast = object_model.get_part("telescoping_mast")
    jib_assembly = object_model.get_part("jib_assembly")
    trolley = object_model.get_part("trolley")

    mast_extension = object_model.get_articulation("mast_extension")
    jib_slew = object_model.get_articulation("jib_slew")
    trolley_travel = object_model.get_articulation("trolley_travel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "part_presence",
        all(part is not None for part in (base_carriage, telescoping_mast, jib_assembly, trolley)),
        "Expected base carriage, telescoping mast, jib assembly, and trolley parts.",
    )
    ctx.check(
        "mast_extension_axis_is_vertical",
        mast_extension.articulation_type == ArticulationType.PRISMATIC
        and tuple(mast_extension.axis) == (0.0, 0.0, 1.0),
        f"mast_extension should be vertical prismatic, got type={mast_extension.articulation_type} axis={mast_extension.axis}",
    )
    ctx.check(
        "jib_slew_axis_is_vertical",
        jib_slew.articulation_type == ArticulationType.REVOLUTE
        and tuple(jib_slew.axis) == (0.0, 0.0, 1.0),
        f"jib_slew should be vertical revolute, got type={jib_slew.articulation_type} axis={jib_slew.axis}",
    )
    ctx.check(
        "trolley_travel_axis_is_along_jib",
        trolley_travel.articulation_type == ArticulationType.PRISMATIC
        and tuple(trolley_travel.axis) == (1.0, 0.0, 0.0),
        f"trolley_travel should be longitudinal prismatic, got type={trolley_travel.articulation_type} axis={trolley_travel.axis}",
    )

    with ctx.pose({mast_extension: 0.0, jib_slew: 0.0, trolley_travel: 0.0}):
        ctx.expect_contact(
            telescoping_mast,
            base_carriage,
            elem_a="guide_shoe_x_positive",
            elem_b="guide_rail_x_positive",
            name="mast_guides_contact_lowered",
        )
        ctx.expect_contact(
            jib_assembly,
            telescoping_mast,
            elem_a="slew_ring",
            elem_b="mast_head_plate",
            name="slew_ring_contact_at_rest",
        )
        ctx.expect_contact(
            trolley,
            jib_assembly,
            elem_a="left_roller_pad",
            elem_b="left_track",
            name="left_trolley_pad_contact_at_inner_end",
        )
        ctx.expect_contact(
            trolley,
            jib_assembly,
            elem_a="right_roller_pad",
            elem_b="right_track",
            name="right_trolley_pad_contact_at_inner_end",
        )
        ctx.expect_overlap(
            trolley,
            jib_assembly,
            axes="y",
            min_overlap=0.10,
            name="trolley_stays_between_jib_tracks_at_inner_end",
        )

    mast_limits = mast_extension.motion_limits
    if mast_limits is not None and mast_limits.lower is not None and mast_limits.upper is not None:
        with ctx.pose({mast_extension: mast_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="mast_extension_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="mast_extension_lower_no_floating")
        with ctx.pose({mast_extension: mast_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="mast_extension_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="mast_extension_upper_no_floating")
            ctx.expect_contact(
                telescoping_mast,
                base_carriage,
                elem_a="guide_shoe_x_positive",
                elem_b="guide_rail_x_positive",
                name="mast_guides_contact_fully_extended",
            )

    jib_limits = jib_slew.motion_limits
    if jib_limits is not None and jib_limits.lower is not None and jib_limits.upper is not None:
        with ctx.pose({mast_extension: 0.40, jib_slew: jib_limits.lower, trolley_travel: 0.50}):
            ctx.fail_if_parts_overlap_in_current_pose(name="jib_slew_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="jib_slew_lower_no_floating")
            ctx.expect_contact(
                jib_assembly,
                telescoping_mast,
                elem_a="slew_ring",
                elem_b="mast_head_plate",
                name="slew_ring_contact_at_negative_slew_limit",
            )
        with ctx.pose({mast_extension: 0.40, jib_slew: jib_limits.upper, trolley_travel: 1.10}):
            ctx.fail_if_parts_overlap_in_current_pose(name="jib_slew_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="jib_slew_upper_no_floating")
            ctx.expect_contact(
                jib_assembly,
                telescoping_mast,
                elem_a="slew_ring",
                elem_b="mast_head_plate",
                name="slew_ring_contact_at_positive_slew_limit",
            )

    trolley_limits = trolley_travel.motion_limits
    if trolley_limits is not None and trolley_limits.lower is not None and trolley_limits.upper is not None:
        with ctx.pose({mast_extension: 0.55, jib_slew: 0.90, trolley_travel: trolley_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="trolley_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="trolley_lower_no_floating")
            ctx.expect_contact(
                trolley,
                jib_assembly,
                elem_a="left_roller_pad",
                elem_b="left_track",
                name="left_trolley_pad_contact_at_lower_limit",
            )
            ctx.expect_contact(
                trolley,
                jib_assembly,
                elem_a="right_roller_pad",
                elem_b="right_track",
                name="right_trolley_pad_contact_at_lower_limit",
            )
        with ctx.pose({mast_extension: 0.55, jib_slew: -1.10, trolley_travel: trolley_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="trolley_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="trolley_upper_no_floating")
            ctx.expect_contact(
                trolley,
                jib_assembly,
                elem_a="left_roller_pad",
                elem_b="left_track",
                name="left_trolley_pad_contact_at_upper_limit",
            )
            ctx.expect_contact(
                trolley,
                jib_assembly,
                elem_a="right_roller_pad",
                elem_b="right_track",
                name="right_trolley_pad_contact_at_upper_limit",
            )
            ctx.expect_overlap(
                trolley,
                jib_assembly,
                axes="y",
                min_overlap=0.10,
                name="trolley_stays_between_jib_tracks_at_outer_end",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
