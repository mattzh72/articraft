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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BASE_HEIGHT = 0.038
COLUMN_HEIGHT = 1.130
SWIVEL_HEIGHT = BASE_HEIGHT + COLUMN_HEIGHT


def _build_weighted_base_mesh():
    return LatheGeometry(
        [
            (0.0, 0.0),
            (0.106, 0.0),
            (0.132, 0.004),
            (0.145, 0.014),
            (0.143, 0.026),
            (0.118, 0.034),
            (0.050, BASE_HEIGHT),
            (0.0, BASE_HEIGHT),
        ],
        segments=72,
    )


def _build_shade_shell_mesh():
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.042, 0.000),
            (0.052, 0.016),
            (0.056, 0.050),
            (0.056, 0.170),
            (0.050, 0.200),
        ],
        [
            (0.000, 0.010),
            (0.037, 0.026),
            (0.046, 0.050),
            (0.046, 0.200),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    shell.rotate_y(math.pi / 2.0)
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_task_lamp")

    base_charcoal = model.material("base_charcoal", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.32, 1.0))
    shade_cream = model.material("shade_cream", rgba=(0.88, 0.87, 0.82, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_geometry(_build_weighted_base_mesh(), "task_lamp_weighted_base"),
        material=base_charcoal,
        name="weighted_base",
    )
    stand.visual(
        Cylinder(radius=0.027, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + 0.007)),
        material=dark_steel,
        name="base_collar",
    )
    stand.visual(
        Cylinder(radius=0.016, length=COLUMN_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + (COLUMN_HEIGHT * 0.5))),
        material=satin_steel,
        name="column",
    )
    stand.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_HEIGHT - 0.006)),
        material=dark_steel,
        name="top_collar",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.29, 0.29, SWIVEL_HEIGHT)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_HEIGHT * 0.5)),
    )

    swivel = model.part("swivel_head")
    swivel.visual(
        Cylinder(radius=0.040, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_steel,
        name="swivel_disk",
    )
    swivel.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=satin_steel,
        name="swivel_stem",
    )
    swivel.visual(
        Box((0.030, 0.028, 0.018)),
        origin=Origin(xyz=(0.012, 0.0, 0.030)),
        material=satin_steel,
        name="shoulder_bridge",
    )
    swivel.visual(
        Box((0.040, 0.006, 0.028)),
        origin=Origin(xyz=(0.018, 0.014, 0.040)),
        material=dark_steel,
        name="left_cheek",
    )
    swivel.visual(
        Box((0.040, 0.006, 0.028)),
        origin=Origin(xyz=(0.018, -0.014, 0.040)),
        material=dark_steel,
        name="right_cheek",
    )
    swivel.inertial = Inertial.from_geometry(
        Box((0.080, 0.040, 0.060)),
        mass=0.6,
        origin=Origin(xyz=(0.020, 0.0, 0.030)),
    )

    arm = model.part("arm_link")
    arm.visual(
        Cylinder(radius=0.0105, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="shoulder_barrel",
    )
    arm.visual(
        Box((0.024, 0.016, 0.016)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=dark_steel,
        name="shoulder_neck",
    )
    arm.visual(
        Cylinder(radius=0.011, length=0.386),
        origin=Origin(xyz=(0.217, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="arm_tube",
    )
    arm.visual(
        Box((0.028, 0.018, 0.016)),
        origin=Origin(xyz=(0.414, 0.0, 0.0)),
        material=dark_steel,
        name="tip_bridge",
    )
    arm.visual(
        Box((0.024, 0.005, 0.030)),
        origin=Origin(xyz=(0.418, 0.0115, 0.0)),
        material=dark_steel,
        name="tip_left_cheek",
    )
    arm.visual(
        Box((0.024, 0.005, 0.030)),
        origin=Origin(xyz=(0.418, -0.0115, 0.0)),
        material=dark_steel,
        name="tip_right_cheek",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.440, 0.050, 0.050)),
        mass=1.2,
        origin=Origin(xyz=(0.220, 0.0, 0.0)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="shade_trunnion",
    )
    shade.visual(
        Box((0.036, 0.016, 0.048)),
        origin=Origin(xyz=(0.018, 0.0, -0.006)),
        material=dark_steel,
        name="shade_mount",
    )
    shade.visual(
        mesh_from_geometry(_build_shade_shell_mesh(), "task_lamp_shade_shell"),
        origin=Origin(xyz=(0.004, 0.0, -0.038)),
        material=shade_cream,
        name="shade_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.220, 0.130, 0.120)),
        mass=0.8,
        origin=Origin(xyz=(0.100, 0.0, -0.038)),
    )

    model.articulation(
        "stand_to_swivel",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-math.radians(110.0),
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "swivel_to_arm",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=arm,
        origin=Origin(xyz=(0.038, 0.0, 0.040)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=-math.radians(40.0),
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=shade,
        origin=Origin(xyz=(0.439, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=-math.radians(65.0),
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    swivel = object_model.get_part("swivel_head")
    arm = object_model.get_part("arm_link")
    shade = object_model.get_part("shade")
    swivel_joint = object_model.get_articulation("stand_to_swivel")
    shoulder_joint = object_model.get_articulation("swivel_to_arm")
    shade_joint = object_model.get_articulation("arm_to_shade")

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

    stand_aabb = ctx.part_world_aabb(stand)
    stand_height = None
    if stand_aabb is not None:
        stand_height = stand_aabb[1][2] - stand_aabb[0][2]
    ctx.check(
        "stand has floor-lamp height",
        stand_height is not None and 1.15 <= stand_height <= 1.25,
        details=f"stand_height={stand_height}",
    )

    ctx.expect_contact(
        swivel,
        stand,
        elem_a="swivel_disk",
        elem_b="top_collar",
        name="swivel disk sits on the column collar",
    )
    ctx.expect_contact(
        arm,
        swivel,
        name="arm shoulder is carried by the swivel yoke",
    )
    ctx.expect_gap(
        arm,
        shade,
        axis="y",
        positive_elem="tip_left_cheek",
        negative_elem="shade_trunnion",
        max_gap=0.00005,
        max_penetration=0.0,
        name="left yoke cheek captures the shade trunnion",
    )
    ctx.expect_gap(
        shade,
        arm,
        axis="y",
        positive_elem="shade_trunnion",
        negative_elem="tip_right_cheek",
        max_gap=0.00005,
        max_penetration=0.0,
        name="right yoke cheek captures the shade trunnion",
    )
    ctx.expect_origin_gap(
        shade,
        stand,
        axis="x",
        min_gap=0.35,
        name="shade projects forward from the stand",
    )

    ctx.check(
        "swivel axis is vertical",
        swivel_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={swivel_joint.axis}",
    )
    ctx.check(
        "shoulder axis lifts the arm upward",
        shoulder_joint.axis == (0.0, -1.0, 0.0),
        details=f"axis={shoulder_joint.axis}",
    )
    ctx.check(
        "shade tilt axis lifts the nose upward",
        shade_joint.axis == (0.0, -1.0, 0.0),
        details=f"axis={shade_joint.axis}",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    with ctx.pose({swivel_joint: math.radians(60.0)}):
        turned_shade_pos = ctx.part_world_position(shade)
    ctx.check(
        "swivel turns the lamp head around the column",
        rest_shade_pos is not None
        and turned_shade_pos is not None
        and turned_shade_pos[1] > rest_shade_pos[1] + 0.25,
        details=f"rest={rest_shade_pos}, turned={turned_shade_pos}",
    )

    with ctx.pose({shoulder_joint: math.radians(60.0)}):
        raised_shade_pos = ctx.part_world_position(shade)
    ctx.check(
        "shoulder raises the arm",
        rest_shade_pos is not None
        and raised_shade_pos is not None
        and raised_shade_pos[2] > rest_shade_pos[2] + 0.25,
        details=f"rest={rest_shade_pos}, raised={raised_shade_pos}",
    )

    rest_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({shade_joint: math.radians(45.0)}):
        tilted_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    ctx.check(
        "shade tilt raises the front of the cylindrical shell",
        rest_shell_aabb is not None
        and tilted_shell_aabb is not None
        and tilted_shell_aabb[1][2] > rest_shell_aabb[1][2] + 0.05,
        details=f"rest={rest_shell_aabb}, tilted={tilted_shell_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
