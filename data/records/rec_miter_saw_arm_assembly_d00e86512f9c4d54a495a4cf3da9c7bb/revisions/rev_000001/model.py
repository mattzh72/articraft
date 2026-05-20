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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _sector_profile(
    *,
    center_y: float,
    center_z: float,
    radius: float,
    start_angle: float,
    end_angle: float,
    segments: int = 24,
) -> list[tuple[float, float]]:
    points = [(center_y, center_z)]
    for index in range(segments + 1):
        t = index / segments
        angle = start_angle + (end_angle - start_angle) * t
        points.append(
            (
                center_y + radius * math.cos(angle),
                center_z + radius * math.sin(angle),
            )
        )
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compound_miter_saw")

    cast_aluminum = model.material("cast_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    tool_gray = model.material("tool_gray", rgba=(0.33, 0.35, 0.38, 1.0))
    fence_silver = model.material("fence_silver", rgba=(0.82, 0.83, 0.84, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.10, 0.11, 0.12, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    guard_orange = model.material("guard_orange", rgba=(0.86, 0.41, 0.16, 1.0))

    base_casting_mesh = _mesh(
        "miter_saw_base_casting",
        ExtrudeGeometry.from_z0(rounded_rect_profile(0.60, 0.42, 0.045), 0.026),
    )
    arm_beam_mesh = _mesh(
        "miter_saw_arm_beam",
        sweep_profile_along_spline(
            [
                (0.0, -0.005, 0.015),
                (0.0, 0.075, 0.060),
                (0.0, 0.155, 0.048),
                (0.0, 0.235, 0.005),
            ],
            profile=rounded_rect_profile(0.058, 0.040, 0.010),
            samples_per_segment=14,
            cap_profile=True,
        ),
    )
    handle_mesh = _mesh(
        "miter_saw_handle",
        tube_from_spline_points(
            [
                (0.0, 0.070, 0.078),
                (0.0, 0.105, 0.132),
                (0.0, 0.165, 0.122),
            ],
            radius=0.009,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    upper_guard_mesh = _mesh(
        "miter_saw_upper_guard",
        ExtrudeGeometry.centered(
            [
                (-0.010, 0.175),
                *[
                    (
                        -0.010 + 0.155 * math.sin(angle),
                        0.175 + 0.155 * math.cos(angle),
                    )
                    for angle in [
                        -0.12 * math.pi
                        + (1.30 * math.pi) * (index / 28.0)
                        for index in range(29)
                    ]
                ],
            ],
            0.038,
            cap=True,
            closed=True,
        ),
    )

    base = model.part("base")
    base.visual(base_casting_mesh, material=cast_aluminum, name="base_casting")
    base.visual(
        Box((0.44, 0.18, 0.018)),
        origin=Origin(xyz=(0.0, -0.090, 0.034)),
        material=cast_aluminum,
        name="rear_scale_deck",
    )
    base.visual(
        Cylinder(radius=0.095, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=tool_gray,
        name="bearing_pedestal",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.60, 0.42, 0.08)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    table_assembly = model.part("table_assembly")
    table_assembly.visual(
        Cylinder(radius=0.100, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=tool_gray,
        name="turntable_hub",
    )
    table_assembly.visual(
        Cylinder(radius=0.145, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=cast_aluminum,
        name="turntable_platter",
    )
    table_assembly.visual(
        Box((0.34, 0.06, 0.018)),
        origin=Origin(xyz=(0.0, -0.110, 0.024)),
        material=cast_aluminum,
        name="fence_support",
    )
    table_assembly.visual(
        Box((0.24, 0.028, 0.055)),
        origin=Origin(xyz=(-0.190, -0.145, 0.052)),
        material=fence_silver,
        name="fence_left",
    )
    table_assembly.visual(
        Box((0.24, 0.028, 0.055)),
        origin=Origin(xyz=(0.190, -0.145, 0.052)),
        material=fence_silver,
        name="fence_right",
    )
    table_assembly.visual(
        Box((0.10, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, -0.125, 0.067)),
        material=tool_gray,
        name="yoke_pedestal",
    )
    table_assembly.visual(
        Box((0.028, 0.040, 0.19)),
        origin=Origin(xyz=(-0.065, -0.155, 0.170)),
        material=tool_gray,
        name="yoke_left_upright",
    )
    table_assembly.visual(
        Box((0.028, 0.040, 0.19)),
        origin=Origin(xyz=(0.065, -0.155, 0.170)),
        material=tool_gray,
        name="yoke_right_upright",
    )
    table_assembly.visual(
        Box((0.17, 0.030, 0.08)),
        origin=Origin(xyz=(0.0, -0.170, 0.190)),
        material=tool_gray,
        name="yoke_bridge",
    )
    table_assembly.visual(
        Box((0.16, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, -0.150, 0.272)),
        material=tool_gray,
        name="top_tie_beam",
    )
    table_assembly.visual(
        Box((0.018, 0.050, 0.040)),
        origin=Origin(xyz=(-0.065, -0.125, 0.260)),
        material=tool_gray,
        name="hinge_ear_left",
    )
    table_assembly.visual(
        Box((0.018, 0.050, 0.040)),
        origin=Origin(xyz=(0.065, -0.125, 0.260)),
        material=tool_gray,
        name="hinge_ear_right",
    )
    table_assembly.inertial = Inertial.from_geometry(
        Box((0.54, 0.24, 0.30)),
        mass=8.5,
        origin=Origin(xyz=(0.0, -0.090, 0.120)),
    )

    cutting_arm = model.part("cutting_arm")
    cutting_arm.visual(
        Cylinder(radius=0.0135, length=0.102),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tool_gray,
        name="trunnion_shaft",
    )
    cutting_arm.visual(arm_beam_mesh, material=tool_gray, name="arm_beam")
    cutting_arm.visual(
        Box((0.10, 0.11, 0.08)),
        origin=Origin(xyz=(0.0, 0.135, 0.035)),
        material=tool_gray,
        name="gearbox_housing",
    )
    cutting_arm.visual(
        Box((0.08, 0.09, 0.06)),
        origin=Origin(xyz=(-0.055, 0.135, 0.045)),
        material=tool_gray,
        name="belt_housing",
    )
    cutting_arm.visual(
        upper_guard_mesh,
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=guard_orange,
        name="upper_guard",
    )
    cutting_arm.visual(
        Cylinder(radius=0.127, length=0.003),
        origin=Origin(
            xyz=(0.0, 0.175, -0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=blade_steel,
        name="blade_disk",
    )
    cutting_arm.visual(
        Cylinder(radius=0.023, length=0.026),
        origin=Origin(
            xyz=(0.0, 0.175, -0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=tool_gray,
        name="blade_hub",
    )
    cutting_arm.visual(
        Cylinder(radius=0.055, length=0.16),
        origin=Origin(
            xyz=(0.105, 0.175, -0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=tool_gray,
        name="motor_housing",
    )
    cutting_arm.visual(
        Cylinder(radius=0.034, length=0.040),
        origin=Origin(
            xyz=(0.205, 0.175, -0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber_black,
        name="motor_end_cap",
    )
    cutting_arm.visual(handle_mesh, material=rubber_black, name="handle_tube")
    cutting_arm.visual(
        Box((0.030, 0.035, 0.090)),
        origin=Origin(xyz=(0.0, 0.155, 0.075)),
        material=rubber_black,
        name="trigger_grip",
    )
    cutting_arm.visual(
        Box((0.050, 0.030, 0.025)),
        origin=Origin(xyz=(0.0, 0.188, 0.114)),
        material=rubber_black,
        name="handle_cap",
    )
    cutting_arm.inertial = Inertial.from_geometry(
        Box((0.34, 0.32, 0.28)),
        mass=7.5,
        origin=Origin(xyz=(0.06, 0.150, 0.040)),
    )

    model.articulation(
        "base_to_table",
        ArticulationType.REVOLUTE,
        parent=base,
        child=table_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=2.0,
            lower=-0.90,
            upper=0.90,
        ),
    )
    model.articulation(
        "table_to_arm",
        ArticulationType.REVOLUTE,
        parent=table_assembly,
        child=cutting_arm,
        origin=Origin(xyz=(0.0, -0.125, 0.260)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.8,
            lower=-0.60,
            upper=0.25,
        ),
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

    base = object_model.get_part("base")
    table_assembly = object_model.get_part("table_assembly")
    cutting_arm = object_model.get_part("cutting_arm")
    table_joint = object_model.get_articulation("base_to_table")
    arm_joint = object_model.get_articulation("table_to_arm")

    bearing_pedestal = base.get_visual("bearing_pedestal")
    turntable_hub = table_assembly.get_visual("turntable_hub")
    turntable_platter = table_assembly.get_visual("turntable_platter")
    blade_disk = cutting_arm.get_visual("blade_disk")

    def _aabb_center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    with ctx.pose({table_joint: 0.0, arm_joint: 0.0}):
        ctx.expect_overlap(
            table_assembly,
            base,
            axes="xy",
            elem_a=turntable_hub,
            elem_b=bearing_pedestal,
            min_overlap=0.18,
            name="turntable hub stays centered over the base bearing",
        )
        ctx.expect_contact(
            table_assembly,
            base,
            elem_a=turntable_hub,
            elem_b=bearing_pedestal,
            contact_tol=0.0005,
            name="turntable hub bears on the base pedestal",
        )
        ctx.expect_gap(
            cutting_arm,
            table_assembly,
            axis="z",
            positive_elem=blade_disk,
            negative_elem=turntable_platter,
            min_gap=0.070,
            name="parked blade clears the turntable surface",
        )

    rest_arm_pos = ctx.part_world_position(cutting_arm)
    with ctx.pose({table_joint: 0.60, arm_joint: 0.0}):
        rotated_arm_pos = ctx.part_world_position(cutting_arm)

    radial_rest = (
        math.hypot(rest_arm_pos[0], rest_arm_pos[1]) if rest_arm_pos is not None else None
    )
    radial_rotated = (
        math.hypot(rotated_arm_pos[0], rotated_arm_pos[1])
        if rotated_arm_pos is not None
        else None
    )
    ctx.check(
        "table rotation swings the saw head around the vertical axis",
        rest_arm_pos is not None
        and rotated_arm_pos is not None
        and abs(rotated_arm_pos[0] - rest_arm_pos[0]) > 0.060
        and abs(rotated_arm_pos[2] - rest_arm_pos[2]) < 0.002
        and radial_rest is not None
        and radial_rotated is not None
        and abs(radial_rotated - radial_rest) < 0.003,
        details=f"rest={rest_arm_pos}, rotated={rotated_arm_pos}",
    )

    with ctx.pose({table_joint: 0.0, arm_joint: 0.0}):
        parked_blade_aabb = ctx.part_element_world_aabb(cutting_arm, elem=blade_disk)
    arm_lower = arm_joint.motion_limits.lower if arm_joint.motion_limits is not None else None
    with ctx.pose({table_joint: 0.0, arm_joint: arm_lower if arm_lower is not None else -0.60}):
        lowered_blade_aabb = ctx.part_element_world_aabb(cutting_arm, elem=blade_disk)
        ctx.expect_gap(
            cutting_arm,
            table_assembly,
            axis="z",
            positive_elem=blade_disk,
            negative_elem=turntable_platter,
            min_gap=0.002,
            name="lowered blade still stays above the table insert surface",
        )

    parked_blade_z = _aabb_center_z(parked_blade_aabb)
    lowered_blade_z = _aabb_center_z(lowered_blade_aabb)
    ctx.check(
        "hinged arm lowers the blade toward the work surface",
        parked_blade_z is not None
        and lowered_blade_z is not None
        and lowered_blade_z < parked_blade_z - 0.080,
        details=f"parked_z={parked_blade_z}, lowered_z={lowered_blade_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
