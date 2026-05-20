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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, segments: int = 24) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _transform_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [(c * x - s * y + dx, s * x + c * y + dy) for x, y in profile]


def _arm_section(width_x: float, depth_y: float, z: float, y: float = 0.0) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(width_x, depth_y, radius=min(width_x, depth_y) * 0.28, corner_segments=7)
    return [(x, py + y, z) for x, py in profile]


def _crank_arm_mesh(*, pedal_drop: float = 0.172) -> object:
    sections = [
        _arm_section(0.028, 0.040, -0.006, y=0.000),
        _arm_section(0.022, 0.035, -0.050, y=0.002),
        _arm_section(0.018, 0.028, -0.110, y=0.006),
        _arm_section(0.016, 0.024, -0.150, y=0.009),
        _arm_section(0.016, 0.026, -pedal_drop, y=0.010),
    ]
    return section_loft(sections)


def _chainring_outer_profile(tooth_count: int, root_radius: float, tooth_radius: float) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    step = math.pi / tooth_count
    for index in range(tooth_count * 2):
        angle = index * step
        radius = tooth_radius if index % 2 == 0 else root_radius
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ebike_torque_sensing_crankset")

    shell_black = model.material("shell_black", rgba=(0.10, 0.11, 0.12, 1.0))
    arm_alloy = model.material("arm_alloy", rgba=(0.30, 0.32, 0.35, 1.0))
    spindle_steel = model.material("spindle_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    chainring_black = model.material("chainring_black", rgba=(0.08, 0.08, 0.09, 1.0))

    bb_shell = model.part("bb_shell")
    shell_outer = [
        (0.038, -0.055),
        (0.038, -0.042),
        (0.0415, -0.030),
        (0.0420, -0.016),
        (0.0420, 0.016),
        (0.0415, 0.030),
        (0.038, 0.042),
        (0.038, 0.055),
    ]
    shell_inner = [(0.0195, -0.055), (0.0195, 0.055)]
    shell_mesh = _save_mesh(
        "bb_shell",
        LatheGeometry.from_shell_profiles(shell_outer, shell_inner, segments=72).rotate_y(math.pi / 2.0),
    )
    bb_shell.visual(shell_mesh, material=shell_black, name="bb_shell")
    bb_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.042, length=0.110),
        mass=1.4,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.015, length=0.166),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="spindle_axle",
    )
    spindle.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.061, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="drive_side_collar",
    )
    spindle.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(-0.061, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="non_drive_collar",
    )
    spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.015, length=0.166),
        mass=0.85,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_crank = model.part("right_crank")
    right_crank.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_alloy,
        name="right_spindle_boss",
    )
    right_crank.visual(
        _save_mesh("right_crank_arm", _crank_arm_mesh()),
        material=arm_alloy,
        name="right_arm_beam",
    )
    right_crank.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, -0.172), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_alloy,
        name="right_pedal_eye",
    )
    right_crank.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.012, 0.0, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_alloy,
        name="right_spider_mount",
    )
    spider_radius_mid = 0.048
    for arm_index, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        right_crank.visual(
            Box((0.008, 0.014, 0.056)),
            origin=Origin(
                xyz=(0.014, -math.sin(angle) * spider_radius_mid, math.cos(angle) * spider_radius_mid),
                rpy=(angle, 0.0, 0.0),
            ),
            material=arm_alloy,
            name=f"spider_arm_{arm_index}",
        )

    chainring_outer = _chainring_outer_profile(tooth_count=42, root_radius=0.089, tooth_radius=0.096)
    chainring_holes = [_circle_profile(0.034, segments=32)]
    window_profile = rounded_rect_profile(0.026, 0.050, radius=0.006, corner_segments=6)
    for angle in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
        chainring_holes.append(
            _transform_profile(
                window_profile,
                dx=0.058 * math.cos(angle),
                dy=0.058 * math.sin(angle),
                angle=angle,
            )
        )
    for angle in (
        math.pi / 4.0,
        3.0 * math.pi / 4.0,
        5.0 * math.pi / 4.0,
        7.0 * math.pi / 4.0,
    ):
        chainring_holes.append(_circle_profile(0.0035, segments=14))
        chainring_holes[-1] = _transform_profile(
            chainring_holes[-1],
            dx=0.032 * math.cos(angle),
            dy=0.032 * math.sin(angle),
        )
    chainring_mesh = _save_mesh(
        "chainring_plate",
        ExtrudeWithHolesGeometry(chainring_outer, chainring_holes, height=0.0042, center=True).rotate_y(math.pi / 2.0),
    )
    right_crank.visual(
        chainring_mesh,
        origin=Origin(xyz=(0.018, 0.0, -0.002)),
        material=chainring_black,
        name="chainring_plate",
    )
    right_crank.inertial = Inertial.from_geometry(
        Box((0.040, 0.220, 0.240)),
        mass=1.3,
        origin=Origin(xyz=(0.009, 0.0, -0.090)),
    )

    left_crank = model.part("left_crank")
    left_crank.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_alloy,
        name="left_spindle_boss",
    )
    left_crank.visual(
        _save_mesh("left_crank_arm", _crank_arm_mesh()),
        material=arm_alloy,
        name="left_arm_beam",
    )
    left_crank.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.010, -0.172), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_alloy,
        name="left_pedal_eye",
    )
    left_crank.inertial = Inertial.from_geometry(
        Box((0.032, 0.050, 0.220)),
        mass=0.72,
        origin=Origin(xyz=(0.004, 0.006, -0.090)),
    )

    right_pedal_spindle = model.part("right_pedal_spindle")
    right_pedal_spindle.visual(
        Cylinder(radius=0.007, length=0.086),
        origin=Origin(xyz=(0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="right_pedal_axle",
    )
    right_pedal_spindle.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="right_pedal_collar",
    )
    right_pedal_spindle.inertial = Inertial.from_geometry(
        Box((0.086, 0.020, 0.020)),
        mass=0.16,
        origin=Origin(xyz=(0.053, 0.0, 0.0)),
    )

    left_pedal_spindle = model.part("left_pedal_spindle")
    left_pedal_spindle.visual(
        Cylinder(radius=0.007, length=0.086),
        origin=Origin(xyz=(-0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="left_pedal_axle",
    )
    left_pedal_spindle.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(-0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=spindle_steel,
        name="left_pedal_collar",
    )
    left_pedal_spindle.inertial = Inertial.from_geometry(
        Box((0.086, 0.020, 0.020)),
        mass=0.16,
        origin=Origin(xyz=(-0.053, 0.0, 0.0)),
    )

    model.articulation(
        "bb_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=bb_shell,
        child=spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=20.0),
    )
    model.articulation(
        "spindle_to_right_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=right_crank,
        origin=Origin(xyz=(0.095, 0.0, 0.0)),
    )
    model.articulation(
        "spindle_to_left_crank",
        ArticulationType.FIXED,
        parent=spindle,
        child=left_crank,
        origin=Origin(xyz=(-0.095, 0.0, 0.0), rpy=(math.pi, 0.0, 0.0)),
    )
    model.articulation(
        "right_crank_to_pedal",
        ArticulationType.CONTINUOUS,
        parent=right_crank,
        child=right_pedal_spindle,
        origin=Origin(xyz=(0.0, 0.010, -0.172)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "left_crank_to_pedal",
        ArticulationType.CONTINUOUS,
        parent=left_crank,
        child=left_pedal_spindle,
        origin=Origin(xyz=(0.0, 0.010, -0.172)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
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

    bb_shell = object_model.get_part("bb_shell")
    spindle = object_model.get_part("spindle")
    right_crank = object_model.get_part("right_crank")
    left_crank = object_model.get_part("left_crank")
    right_pedal_spindle = object_model.get_part("right_pedal_spindle")
    left_pedal_spindle = object_model.get_part("left_pedal_spindle")

    spindle_joint = object_model.get_articulation("bb_to_spindle")
    right_pedal_joint = object_model.get_articulation("right_crank_to_pedal")
    left_pedal_joint = object_model.get_articulation("left_crank_to_pedal")

    ctx.check(
        "bottom bracket and pedal articulations are continuous about crank axes",
        spindle_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spindle_joint.axis) == (1.0, 0.0, 0.0)
        and right_pedal_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(right_pedal_joint.axis) == (1.0, 0.0, 0.0)
        and left_pedal_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_pedal_joint.axis) == (1.0, 0.0, 0.0),
        details=(
            f"spindle={spindle_joint.articulation_type}/{spindle_joint.axis}, "
            f"right_pedal={right_pedal_joint.articulation_type}/{right_pedal_joint.axis}, "
            f"left_pedal={left_pedal_joint.articulation_type}/{left_pedal_joint.axis}"
        ),
    )

    ctx.expect_gap(
        right_crank,
        bb_shell,
        axis="x",
        positive_elem="chainring_plate",
        negative_elem="bb_shell",
        min_gap=0.050,
        name="chainring clears the bottom bracket shell on the drive side",
    )
    ctx.expect_gap(
        spindle,
        bb_shell,
        axis="x",
        positive_elem="drive_side_collar",
        negative_elem="bb_shell",
        max_gap=0.0005,
        max_penetration=0.0,
        name="drive-side spindle collar seats against the shell face",
    )
    ctx.expect_gap(
        bb_shell,
        spindle,
        axis="x",
        positive_elem="bb_shell",
        negative_elem="non_drive_collar",
        max_gap=0.0005,
        max_penetration=0.0,
        name="non-drive spindle collar seats against the shell face",
    )
    ctx.expect_gap(
        right_pedal_spindle,
        right_crank,
        axis="x",
        positive_elem="right_pedal_collar",
        negative_elem="right_pedal_eye",
        max_gap=0.0005,
        max_penetration=0.0,
        name="right pedal spindle shoulder seats against the crank eye",
    )
    ctx.expect_gap(
        left_crank,
        left_pedal_spindle,
        axis="x",
        positive_elem="left_pedal_eye",
        negative_elem="left_pedal_collar",
        max_gap=0.0005,
        max_penetration=0.0,
        name="left pedal spindle shoulder seats against the crank eye",
    )

    right_rest = ctx.part_world_position(right_pedal_spindle)
    left_rest = ctx.part_world_position(left_pedal_spindle)
    ctx.check(
        "crank arms start 180 degrees opposed",
        right_rest is not None
        and left_rest is not None
        and right_rest[2] < -0.15
        and left_rest[2] > 0.15,
        details=f"right_rest={right_rest}, left_rest={left_rest}",
    )

    with ctx.pose({spindle_joint: math.pi / 2.0}):
        turned_right = ctx.part_world_position(right_pedal_spindle)
        turned_left = ctx.part_world_position(left_pedal_spindle)
        ctx.check(
            "bottom bracket rotation swings both pedal axes around the shell",
            right_rest is not None
            and left_rest is not None
            and turned_right is not None
            and turned_left is not None
            and turned_right[1] > right_rest[1] + 0.15
            and turned_left[1] < left_rest[1] - 0.15
            and abs(turned_right[2]) < 0.04
            and abs(turned_left[2]) < 0.04,
            details=(
                f"right_rest={right_rest}, turned_right={turned_right}, "
                f"left_rest={left_rest}, turned_left={turned_left}"
            ),
        )

        ctx.expect_overlap(
            spindle,
            bb_shell,
            axes="yz",
            elem_a="spindle_axle",
            elem_b="bb_shell",
            min_overlap=0.025,
            name="spindle stays concentric within the shell while rotating",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
