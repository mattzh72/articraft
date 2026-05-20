from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    mesh_from_geometry,
    wire_from_points,
)


LINK_LENGTH = 0.72
SIDE_X = 0.62
UPPER_FRONT_Y = 0.20
UPPER_REAR_Y = -0.20
UPPER_Z = 1.54
BENCH_WIDTH = 1.24
BENCH_PIVOT_SPAN = 0.40


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_link_visuals(part, *, steel) -> None:
    eye_radius = 0.028
    eye_length = 0.030
    part.visual(
        Cylinder(radius=eye_radius, length=eye_length),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="upper_eye",
    )
    part.visual(
        Box((0.030, 0.050, LINK_LENGTH - 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH * 0.5)),
        material=steel,
        name="link_bar",
    )
    part.visual(
        Cylinder(radius=eye_radius, length=eye_length),
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="lower_eye",
    )
    part.visual(
        Box((0.030, 0.060, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
        material=steel,
        name="upper_gusset",
    )
    part.visual(
        Box((0.030, 0.060, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH + 0.085)),
        material=steel,
        name="lower_gusset",
    )


def _add_bracket_pair(part, *, pivot_x: float, pivot_y: float, pivot_z: float, material, depth: float, height: float) -> None:
    plate_thickness = 0.008
    plate_gap = 0.030
    offset = plate_gap * 0.5 + plate_thickness * 0.5
    for dx in (-offset, offset):
        part.visual(
            Box((plate_thickness, depth, height)),
            origin=Origin(xyz=(pivot_x + dx, pivot_y, pivot_z - height * 0.5)),
            material=material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_glider_swing")

    frame_green = model.material("frame_green", rgba=(0.18, 0.34, 0.25, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.21, 0.22, 0.24, 1.0))
    bench_steel = model.material("bench_steel", rgba=(0.24, 0.26, 0.28, 1.0))
    cedar = model.material("cedar", rgba=(0.72, 0.53, 0.34, 1.0))
    hardware = model.material("hardware", rgba=(0.73, 0.74, 0.77, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.70, 1.35, 1.80)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
    )

    left_a_frame = wire_from_points(
        [
            (-0.74, 0.60, 0.02),
            (-0.74, 0.02, 1.68),
            (-0.74, -0.57, 0.02),
        ],
        radius=0.040,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.085,
        corner_segments=10,
    )
    right_a_frame = wire_from_points(
        [
            (0.74, 0.60, 0.02),
            (0.74, 0.02, 1.68),
            (0.74, -0.57, 0.02),
        ],
        radius=0.040,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.085,
        corner_segments=10,
    )
    frame.visual(_mesh("left_a_frame", left_a_frame), material=frame_green)
    frame.visual(_mesh("right_a_frame", right_a_frame), material=frame_green)
    frame.visual(
        Cylinder(radius=0.050, length=1.48),
        origin=Origin(xyz=(0.0, 0.02, 1.68), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_green,
        name="top_beam",
    )
    frame.visual(
        Cylinder(radius=0.032, length=1.48),
        origin=Origin(xyz=(0.0, 0.53, 0.08), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_green,
    )
    frame.visual(
        Cylinder(radius=0.032, length=1.48),
        origin=Origin(xyz=(0.0, -0.50, 0.08), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_green,
    )
    frame.visual(
        Box((0.07, 0.50, 0.06)),
        origin=Origin(xyz=(-SIDE_X, 0.0, 1.61)),
        material=dark_steel,
    )
    frame.visual(
        Box((0.07, 0.50, 0.06)),
        origin=Origin(xyz=(SIDE_X, 0.0, 1.61)),
        material=dark_steel,
    )
    frame.visual(Box((0.14, 0.14, 0.04)), origin=Origin(xyz=(-0.74, 0.60, 0.02)), material=dark_steel)
    frame.visual(Box((0.14, 0.14, 0.04)), origin=Origin(xyz=(0.74, 0.60, 0.02)), material=dark_steel)
    frame.visual(Box((0.14, 0.14, 0.04)), origin=Origin(xyz=(-0.74, -0.57, 0.02)), material=dark_steel)
    frame.visual(Box((0.14, 0.14, 0.04)), origin=Origin(xyz=(0.74, -0.57, 0.02)), material=dark_steel)

    for pivot_x in (-SIDE_X, SIDE_X):
        for pivot_y in (UPPER_FRONT_Y, UPPER_REAR_Y):
            _add_bracket_pair(
                frame,
                pivot_x=pivot_x,
                pivot_y=pivot_y,
                pivot_z=UPPER_Z,
                material=hardware,
                depth=0.090,
                height=0.100,
            )
            frame.visual(
                Box((0.046, 0.024, 0.108)),
                origin=Origin(
                    xyz=(
                        pivot_x,
                        pivot_y + (0.055 if pivot_y > 0.0 else -0.055),
                        UPPER_Z + 0.054,
                    )
                ),
                material=hardware,
            )

    left_front_link = model.part("left_front_link")
    left_front_link.inertial = Inertial.from_geometry(
        Box((0.05, 0.07, LINK_LENGTH)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH * 0.5)),
    )
    _add_link_visuals(left_front_link, steel=dark_steel)

    left_rear_link = model.part("left_rear_link")
    left_rear_link.inertial = Inertial.from_geometry(
        Box((0.05, 0.07, LINK_LENGTH)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH * 0.5)),
    )
    _add_link_visuals(left_rear_link, steel=dark_steel)

    right_front_link = model.part("right_front_link")
    right_front_link.inertial = Inertial.from_geometry(
        Box((0.05, 0.07, LINK_LENGTH)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH * 0.5)),
    )
    _add_link_visuals(right_front_link, steel=dark_steel)

    right_rear_link = model.part("right_rear_link")
    right_rear_link.inertial = Inertial.from_geometry(
        Box((0.05, 0.07, LINK_LENGTH)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH * 0.5)),
    )
    _add_link_visuals(right_rear_link, steel=dark_steel)

    bench = model.part("bench")
    bench.inertial = Inertial.from_geometry(
        Box((1.34, 0.64, 0.62)),
        mass=34.0,
        origin=Origin(xyz=(BENCH_WIDTH * 0.5, -0.23, -0.06)),
    )

    for pivot_x in (0.0, BENCH_WIDTH):
        for pivot_y in (0.0, -BENCH_PIVOT_SPAN):
            _add_bracket_pair(
                bench,
                pivot_x=pivot_x,
                pivot_y=pivot_y,
                pivot_z=0.0,
                material=bench_steel,
                depth=0.070,
                height=0.210,
            )
            bench.visual(
                Box((0.046, 0.050, 0.018)),
                origin=Origin(xyz=(pivot_x, pivot_y, -0.168)),
                material=bench_steel,
            )

    bench.visual(
        Box((0.090, 0.46, 0.22)),
        origin=Origin(xyz=(0.065, -0.20, -0.140)),
        material=bench_steel,
        name="left_end_frame",
    )
    bench.visual(
        Box((0.094, 0.46, 0.22)),
        origin=Origin(xyz=(BENCH_WIDTH - 0.065, -0.20, -0.140)),
        material=bench_steel,
        name="right_end_frame",
    )
    bench.visual(
        Box((1.08, 0.32, 0.030)),
        origin=Origin(xyz=(BENCH_WIDTH * 0.5, -0.21, -0.235)),
        material=bench_steel,
        name="underseat_pan",
    )
    back_tilt = 0.32

    bench.visual(
        Box((1.08, 0.060, 0.32)),
        origin=Origin(xyz=(BENCH_WIDTH * 0.5, -0.46, 0.00), rpy=(back_tilt, 0.0, 0.0)),
        material=bench_steel,
        name="back_spine",
    )

    bench.visual(
        Box((0.10, 0.34, 0.050)),
        origin=Origin(xyz=(0.05, -0.21, -0.26)),
        material=bench_steel,
    )
    bench.visual(
        Box((0.10, 0.34, 0.050)),
        origin=Origin(xyz=(BENCH_WIDTH - 0.05, -0.21, -0.26)),
        material=bench_steel,
    )
    bench.visual(
        Box((1.14, 0.050, 0.050)),
        origin=Origin(xyz=(BENCH_WIDTH * 0.5, -0.05, -0.26)),
        material=bench_steel,
        name="seat_front_rail",
    )
    bench.visual(
        Box((1.14, 0.050, 0.050)),
        origin=Origin(xyz=(BENCH_WIDTH * 0.5, -0.37, -0.26)),
        material=bench_steel,
        name="seat_rear_rail",
    )
    bench.visual(
        Box((1.10, 0.030, 0.140)),
        origin=Origin(xyz=(BENCH_WIDTH * 0.5, 0.005, -0.17)),
        material=bench_steel,
    )

    seat_slat_y = (-0.08, -0.16, -0.24, -0.32)
    for index, slat_y in enumerate(seat_slat_y, start=1):
        bench.visual(
            Box((1.08, 0.062, 0.022)),
            origin=Origin(xyz=(BENCH_WIDTH * 0.5, slat_y, -0.224)),
            material=cedar,
            name=f"seat_slat_{index}",
        )

    bench.visual(
        Box((0.06, 0.05, 0.35)),
        origin=Origin(xyz=(0.06, -0.44, 0.02), rpy=(back_tilt, 0.0, 0.0)),
        material=bench_steel,
    )
    bench.visual(
        Box((0.06, 0.05, 0.35)),
        origin=Origin(xyz=(BENCH_WIDTH - 0.06, -0.44, 0.02), rpy=(back_tilt, 0.0, 0.0)),
        material=bench_steel,
    )
    bench.visual(
        Box((1.10, 0.05, 0.04)),
        origin=Origin(xyz=(BENCH_WIDTH * 0.5, -0.43, -0.03), rpy=(back_tilt, 0.0, 0.0)),
        material=bench_steel,
    )
    bench.visual(
        Box((1.10, 0.05, 0.04)),
        origin=Origin(xyz=(BENCH_WIDTH * 0.5, -0.53, 0.18), rpy=(back_tilt, 0.0, 0.0)),
        material=bench_steel,
    )
    for index, (slat_y, slat_z) in enumerate(((-0.46, -0.01), (-0.49, 0.07), (-0.52, 0.15)), start=1):
        bench.visual(
            Box((1.06, 0.060, 0.020)),
            origin=Origin(xyz=(BENCH_WIDTH * 0.5, slat_y, slat_z), rpy=(back_tilt, 0.0, 0.0)),
            material=cedar,
            name=f"back_slat_{index}",
        )

    left_armrest = wire_from_points(
        [
            (0.06, -0.03, -0.13),
            (0.06, -0.10, -0.04),
            (0.06, -0.26, 0.03),
            (0.06, -0.42, 0.19),
        ],
        radius=0.015,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.045,
        corner_segments=8,
    )
    right_armrest = wire_from_points(
        [
            (BENCH_WIDTH - 0.06, -0.03, -0.13),
            (BENCH_WIDTH - 0.06, -0.10, -0.04),
            (BENCH_WIDTH - 0.06, -0.26, 0.03),
            (BENCH_WIDTH - 0.06, -0.42, 0.19),
        ],
        radius=0.015,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.045,
        corner_segments=8,
    )
    bench.visual(_mesh("left_armrest", left_armrest), material=bench_steel)
    bench.visual(_mesh("right_armrest", right_armrest), material=bench_steel)

    model.articulation(
        "left_front_upper",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_front_link,
        origin=Origin(xyz=(-SIDE_X, UPPER_FRONT_Y, UPPER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.8, lower=-0.40, upper=0.40),
    )
    model.articulation(
        "left_front_lower",
        ArticulationType.REVOLUTE,
        parent=left_front_link,
        child=bench,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.8, lower=-0.50, upper=0.50),
    )
    model.articulation(
        "left_rear_upper",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_rear_link,
        origin=Origin(xyz=(-SIDE_X, UPPER_REAR_Y, UPPER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.8, lower=-0.40, upper=0.40),
    )
    model.articulation(
        "right_front_upper",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_front_link,
        origin=Origin(xyz=(SIDE_X, UPPER_FRONT_Y, UPPER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.8, lower=-0.40, upper=0.40),
    )
    model.articulation(
        "right_rear_upper",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_rear_link,
        origin=Origin(xyz=(SIDE_X, UPPER_REAR_Y, UPPER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.8, lower=-0.40, upper=0.40),
    )

    return model
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    bench = object_model.get_part("bench")
    left_front_link = object_model.get_part("left_front_link")
    left_rear_link = object_model.get_part("left_rear_link")
    right_front_link = object_model.get_part("right_front_link")
    right_rear_link = object_model.get_part("right_rear_link")
    left_front_upper = object_model.get_articulation("left_front_upper")
    left_front_lower = object_model.get_articulation("left_front_lower")
    left_rear_upper = object_model.get_articulation("left_rear_upper")
    right_front_upper = object_model.get_articulation("right_front_upper")
    right_rear_upper = object_model.get_articulation("right_rear_upper")

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

    for joint_name in (
        "left_front_upper",
        "left_front_lower",
        "left_rear_upper",
        "right_front_upper",
        "right_rear_upper",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name}_axis_is_x",
            tuple(round(value, 6) for value in joint.axis) == (1.0, 0.0, 0.0),
            details=f"{joint_name} axis was {joint.axis!r}",
        )

    ctx.expect_contact(frame, left_front_link, name="frame_contacts_left_front_link")
    ctx.expect_contact(frame, left_rear_link, name="frame_contacts_left_rear_link")
    ctx.expect_contact(frame, right_front_link, name="frame_contacts_right_front_link")
    ctx.expect_contact(frame, right_rear_link, name="frame_contacts_right_rear_link")
    ctx.expect_contact(bench, left_front_link, name="bench_contacts_left_front_link")
    ctx.expect_contact(bench, left_rear_link, name="bench_contacts_left_rear_link")
    ctx.expect_contact(bench, right_front_link, name="bench_contacts_right_front_link")
    ctx.expect_contact(bench, right_rear_link, name="bench_contacts_right_rear_link")
    ctx.expect_gap(
        frame,
        bench,
        axis="z",
        positive_elem="top_beam",
        max_gap=0.90,
        min_gap=0.50,
        name="bench_hangs_well_below_top_beam",
    )

    rest_position = ctx.part_world_position(bench)
    glide_pose = {
        left_front_upper: 0.28,
        left_front_lower: -0.28,
        left_rear_upper: 0.28,
        right_front_upper: 0.28,
        right_rear_upper: 0.28,
    }
    with ctx.pose(glide_pose):
        ctx.expect_contact(bench, left_front_link, name="bench_contacts_left_front_link_in_glide")
        ctx.expect_contact(bench, left_rear_link, name="bench_contacts_left_rear_link_in_glide")
        ctx.expect_contact(bench, right_front_link, name="bench_contacts_right_front_link_in_glide")
        ctx.expect_contact(bench, right_rear_link, name="bench_contacts_right_rear_link_in_glide")

        glide_position = ctx.part_world_position(bench)
        ctx.check(
            "bench_glides_forward",
            rest_position is not None
            and glide_position is not None
            and glide_position[1] > rest_position[1] + 0.10,
            details=f"rest={rest_position!r}, glide={glide_position!r}",
        )

        front_rail = ctx.part_element_world_aabb(bench, elem="seat_front_rail")
        rear_rail = ctx.part_element_world_aabb(bench, elem="seat_rear_rail")
        front_center_z = None if front_rail is None else (front_rail[0][2] + front_rail[1][2]) * 0.5
        rear_center_z = None if rear_rail is None else (rear_rail[0][2] + rear_rail[1][2]) * 0.5
        ctx.check(
            "seat_stays_level_in_glide_pose",
            front_center_z is not None
            and rear_center_z is not None
            and abs(front_center_z - rear_center_z) <= 0.012,
            details=f"front_z={front_center_z!r}, rear_z={rear_center_z!r}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
