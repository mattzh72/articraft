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
)


def _build_upper_guard(name: str):
    start_angle = math.radians(35.0)
    end_angle = math.radians(235.0)
    outer_radius = 0.128
    inner_radius = 0.109
    samples = 24

    outer = [
        (
            outer_radius * math.cos(start_angle + (end_angle - start_angle) * idx / samples),
            outer_radius * math.sin(start_angle + (end_angle - start_angle) * idx / samples),
        )
        for idx in range(samples + 1)
    ]
    inner = [
        (
            inner_radius * math.cos(end_angle - (end_angle - start_angle) * idx / samples),
            inner_radius * math.sin(end_angle - (end_angle - start_angle) * idx / samples),
        )
        for idx in range(samples + 1)
    ]

    guard = ExtrudeGeometry.centered(outer + inner, 0.06, cap=True, closed=True)
    guard.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(guard, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rail_slide_miter_saw")

    base_dark = model.material("base_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    table_metal = model.material("table_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.58, 0.60, 0.64, 1.0))
    guard_gray = model.material("guard_gray", rgba=(0.42, 0.46, 0.52, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.84, 0.86, 0.88, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    handle_black = model.material("handle_black", rgba=(0.09, 0.09, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.72, 0.52, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=base_dark,
        name="elem_base_plinth",
    )
    base.visual(
        Cylinder(radius=0.13, length=0.05),
        origin=Origin(xyz=(0.07, 0.0, 0.085)),
        material=base_dark,
        name="elem_table_pedestal",
    )
    base.visual(
        Box((0.04, 0.64, 0.10)),
        origin=Origin(xyz=(-0.14, 0.0, 0.11)),
        material=housing_gray,
        name="elem_fence",
    )
    base.visual(
        Box((0.10, 0.14, 0.34)),
        origin=Origin(xyz=(-0.21, 0.0, 0.23)),
        material=guard_gray,
        name="elem_rear_column",
    )
    base.visual(
        Box((0.13, 0.24, 0.07)),
        origin=Origin(xyz=(-0.145, 0.0, 0.405)),
        material=guard_gray,
        name="elem_column_head",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.36),
        origin=Origin(xyz=(0.05, -0.075, 0.39), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="elem_left_tube",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.36),
        origin=Origin(xyz=(0.05, 0.075, 0.39), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="elem_right_tube",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.72, 0.52, 0.50)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
    )

    table = model.part("turntable")
    table.visual(
        Cylinder(radius=0.16, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=table_metal,
        name="elem_turntable_disk",
    )
    table.visual(
        Box((0.19, 0.08, 0.024)),
        origin=Origin(xyz=(0.13, 0.0, 0.022)),
        material=table_metal,
        name="elem_turntable_front",
    )
    table.visual(
        Box((0.26, 0.05, 0.006)),
        origin=Origin(xyz=(0.02, 0.0, 0.043)),
        material=blade_metal,
        name="elem_kerf_plate",
    )
    table.inertial = Inertial.from_geometry(
        Box((0.36, 0.34, 0.05)),
        mass=3.2,
        origin=Origin(xyz=(0.04, 0.0, 0.022)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.12, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, -0.075, -0.029)),
        material=guard_gray,
        name="elem_left_sleeve",
    )
    carriage.visual(
        Box((0.12, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, 0.075, -0.029)),
        material=guard_gray,
        name="elem_right_sleeve",
    )
    carriage.visual(
        Box((0.04, 0.23, 0.05)),
        origin=Origin(xyz=(-0.005, 0.0, -0.044)),
        material=guard_gray,
        name="elem_carriage_lower_bridge",
    )
    carriage.visual(
        Box((0.04, 0.09, 0.17)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=guard_gray,
        name="elem_carriage_tower",
    )
    carriage.visual(
        Box((0.03, 0.022, 0.09)),
        origin=Origin(xyz=(0.015, -0.056, 0.115)),
        material=guard_gray,
        name="elem_left_cheek",
    )
    carriage.visual(
        Box((0.03, 0.022, 0.09)),
        origin=Origin(xyz=(0.015, 0.056, 0.115)),
        material=guard_gray,
        name="elem_right_cheek",
    )
    carriage.visual(
        Box((0.03, 0.14, 0.022)),
        origin=Origin(xyz=(0.01, 0.0, 0.155)),
        material=guard_gray,
        name="elem_yoke_bridge",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.15, 0.25, 0.20)),
        mass=4.0,
        origin=Origin(xyz=(0.01, 0.0, 0.06)),
    )

    upper_guard_mesh = _build_upper_guard("upper_guard_shell")

    arm = model.part("arm")
    arm.visual(
        Box((0.04, 0.09, 0.07)),
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
        material=guard_gray,
        name="elem_pivot_block",
    )
    arm.visual(
        Box((0.18, 0.10, 0.07)),
        origin=Origin(xyz=(0.12, 0.0, 0.05)),
        material=housing_gray,
        name="elem_arm_beam",
    )
    arm.visual(
        upper_guard_mesh,
        origin=Origin(xyz=(0.225, 0.0, -0.018)),
        material=guard_gray,
        name="elem_upper_guard",
    )
    arm.visual(
        Cylinder(radius=0.105, length=0.004),
        origin=Origin(xyz=(0.225, 0.0, -0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blade_metal,
        name="elem_blade",
    )
    arm.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(xyz=(0.21, 0.09, 0.06), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_gray,
        name="elem_motor",
    )
    arm.visual(
        Box((0.018, 0.018, 0.055)),
        origin=Origin(xyz=(0.105, 0.0, 0.102)),
        material=handle_black,
        name="elem_handle_stem_left",
    )
    arm.visual(
        Box((0.018, 0.018, 0.055)),
        origin=Origin(xyz=(0.135, 0.0, 0.102)),
        material=handle_black,
        name="elem_handle_stem_right",
    )
    arm.visual(
        Cylinder(radius=0.015, length=0.12),
        origin=Origin(xyz=(0.12, 0.0, 0.127), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="elem_handle_grip",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.36, 0.20, 0.28)),
        mass=6.2,
        origin=Origin(xyz=(0.18, 0.0, 0.03)),
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=table,
        origin=Origin(xyz=(0.07, 0.0, 0.11)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.6,
            lower=-0.87,
            upper=0.87,
        ),
    )
    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(-0.015, 0.0, 0.39)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=0.20,
        ),
    )
    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(0.03, 0.0, 0.115)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.4,
            lower=-1.05,
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
    table = object_model.get_part("turntable")
    carriage = object_model.get_part("carriage")
    arm = object_model.get_part("arm")

    table_joint = object_model.get_articulation("base_to_turntable")
    slide_joint = object_model.get_articulation("base_to_carriage")
    tilt_joint = object_model.get_articulation("carriage_to_arm")

    slide_upper = (
        slide_joint.motion_limits.upper
        if slide_joint.motion_limits is not None and slide_joint.motion_limits.upper is not None
        else 0.20
    )
    tilt_lower = (
        tilt_joint.motion_limits.lower
        if tilt_joint.motion_limits is not None and tilt_joint.motion_limits.lower is not None
        else -1.0
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[idx] + maxs[idx]) * 0.5 for idx in range(3))

    with ctx.pose({table_joint: 0.0, slide_joint: 0.0, tilt_joint: 0.0}):
        ctx.expect_gap(
            table,
            base,
            axis="z",
            positive_elem="elem_turntable_disk",
            negative_elem="elem_table_pedestal",
            max_gap=0.002,
            max_penetration=1e-6,
            name="turntable seats on the pedestal",
        )
        ctx.expect_gap(
            table,
            base,
            axis="x",
            positive_elem="elem_turntable_disk",
            negative_elem="elem_fence",
            min_gap=0.02,
            name="turntable clears the fence",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="elem_left_sleeve",
            elem_b="elem_left_tube",
            min_overlap=0.10,
            name="left sleeve stays engaged on the left guide tube at rest",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="elem_right_sleeve",
            elem_b="elem_right_tube",
            min_overlap=0.10,
            name="right sleeve stays engaged on the right guide tube at rest",
        )
        ctx.expect_gap(
            arm,
            carriage,
            axis="x",
            positive_elem="elem_pivot_block",
            negative_elem="elem_left_cheek",
            max_gap=0.001,
            max_penetration=0.0,
            name="arm pivot block is supported by the yoke cheeks",
        )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide_joint: slide_upper}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="elem_left_sleeve",
            elem_b="elem_left_tube",
            min_overlap=0.10,
            name="left sleeve retains insertion at full slide",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="elem_right_sleeve",
            elem_b="elem_right_tube",
            min_overlap=0.10,
            name="right sleeve retains insertion at full slide",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage slides forward on the rail tubes",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.15
        and abs(extended_carriage_pos[1] - rest_carriage_pos[1]) < 1e-6
        and abs(extended_carriage_pos[2] - rest_carriage_pos[2]) < 1e-6,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    front_rest = None
    front_rotated = None
    with ctx.pose({table_joint: 0.0}):
        front_rest = _aabb_center(ctx.part_element_world_aabb(table, elem="elem_turntable_front"))
    with ctx.pose({table_joint: 0.55}):
        front_rotated = _aabb_center(ctx.part_element_world_aabb(table, elem="elem_turntable_front"))

    ctx.check(
        "turntable rotates about a vertical axis",
        front_rest is not None
        and front_rotated is not None
        and abs(front_rotated[2] - front_rest[2]) < 1e-6
        and abs(front_rotated[1] - front_rest[1]) > 0.05,
        details=f"rest_front={front_rest}, rotated_front={front_rotated}",
    )

    blade_rest = None
    blade_lowered = None
    with ctx.pose({slide_joint: 0.04, tilt_joint: 0.0}):
        blade_rest = ctx.part_element_world_aabb(arm, elem="elem_blade")
    with ctx.pose({slide_joint: 0.04, tilt_joint: tilt_lower}):
        blade_lowered = ctx.part_element_world_aabb(arm, elem="elem_blade")
        ctx.expect_gap(
            arm,
            table,
            axis="z",
            positive_elem="elem_blade",
            negative_elem="elem_turntable_disk",
            min_gap=0.02,
            max_gap=0.07,
            name="lowered blade hovers just above the turntable",
        )

    ctx.check(
        "blade lowers toward the work surface",
        blade_rest is not None
        and blade_lowered is not None
        and blade_lowered[0][2] < blade_rest[0][2] - 0.12,
        details=f"rest_blade={blade_rest}, lowered_blade={blade_lowered}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
