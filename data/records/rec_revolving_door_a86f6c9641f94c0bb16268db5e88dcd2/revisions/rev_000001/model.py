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


def _world_aabb_center(aabb):
    if aabb is None:
        return None
    minimum, maximum = aabb
    return (
        0.5 * (minimum[0] + maximum[0]),
        0.5 * (minimum[1] + maximum[1]),
        0.5 * (minimum[2] + maximum[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_four_wing_revolving_door")

    housing_steel = model.material("housing_steel", rgba=(0.26, 0.28, 0.30, 1.0))
    panel_steel = model.material("panel_steel", rgba=(0.44, 0.46, 0.49, 1.0))
    trim_steel = model.material("trim_steel", rgba=(0.18, 0.19, 0.20, 1.0))

    outer_radius = 1.18
    threshold_thickness = 0.08
    canopy_thickness = 0.18
    overall_height = 2.55
    shell_height = overall_height - threshold_thickness - canopy_thickness + 0.02
    shell_center_z = 0.5 * (0.07 + 2.38)

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=outer_radius, length=threshold_thickness),
        origin=Origin(xyz=(0.0, 0.0, threshold_thickness * 0.5)),
        material=trim_steel,
        name="threshold",
    )
    housing.visual(
        Cylinder(radius=outer_radius, length=canopy_thickness),
        origin=Origin(xyz=(0.0, 0.0, overall_height - canopy_thickness * 0.5)),
        material=housing_steel,
        name="canopy",
    )
    housing.visual(
        Cylinder(radius=0.17, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=trim_steel,
        name="bottom_bearing",
    )

    drum_radius = 1.06
    drum_angles = (-0.95, -0.55, 0.0, 0.55, 0.95)
    for side_index, base_angle in enumerate((0.0, math.pi)):
        for segment_index, angle_offset in enumerate(drum_angles):
            angle = base_angle + angle_offset
            housing.visual(
                Box((0.08, 0.34, shell_height)),
                origin=Origin(
                    xyz=(drum_radius * math.cos(angle), drum_radius * math.sin(angle), shell_center_z),
                    rpy=(0.0, 0.0, angle),
                ),
                material=housing_steel,
                name=f"drum_segment_{side_index}_{segment_index}",
            )

    jamb_positions = (
        (0.66, 0.92),
        (-0.66, 0.92),
        (0.66, -0.92),
        (-0.66, -0.92),
    )
    for index, (x_pos, y_pos) in enumerate(jamb_positions):
        housing.visual(
            Box((0.14, 0.18, shell_height)),
            origin=Origin(xyz=(x_pos, y_pos, shell_center_z)),
            material=trim_steel,
            name=f"jamb_{index}",
        )

    housing.inertial = Inertial.from_geometry(
        Box((outer_radius * 2.0, outer_radius * 2.0, overall_height)),
        mass=850.0,
        origin=Origin(xyz=(0.0, 0.0, overall_height * 0.5)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.15, length=2.22),
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
        material=trim_steel,
        name="center_post",
    )
    rotor.visual(
        Cylinder(radius=0.25, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=trim_steel,
        name="bottom_hub",
    )
    rotor.visual(
        Cylinder(radius=0.25, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.26)),
        material=trim_steel,
        name="top_hub",
    )

    wing_length = 0.74
    wing_thickness = 0.06
    wing_height = 2.10
    wing_center_offset = 0.50
    wing_center_z = 1.20

    rotor.visual(
        Box((wing_length, wing_thickness, wing_height)),
        origin=Origin(xyz=(wing_center_offset, 0.0, wing_center_z)),
        material=panel_steel,
        name="wing_pos_x",
    )
    rotor.visual(
        Box((wing_length, wing_thickness, wing_height)),
        origin=Origin(xyz=(-wing_center_offset, 0.0, wing_center_z)),
        material=panel_steel,
        name="wing_neg_x",
    )
    rotor.visual(
        Box((wing_thickness, wing_length, wing_height)),
        origin=Origin(xyz=(0.0, wing_center_offset, wing_center_z)),
        material=panel_steel,
        name="wing_pos_y",
    )
    rotor.visual(
        Box((wing_thickness, wing_length, wing_height)),
        origin=Origin(xyz=(0.0, -wing_center_offset, wing_center_z)),
        material=panel_steel,
        name="wing_neg_y",
    )
    rotor.inertial = Inertial.from_geometry(
        Box((1.74, 1.74, 2.24)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
    )

    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=300.0, velocity=1.5),
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

    housing = object_model.get_part("housing")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("housing_to_rotor")
    rotor.get_visual("center_post")
    rotor.get_visual("wing_pos_x")
    rotor.get_visual("wing_neg_x")
    rotor.get_visual("wing_pos_y")
    rotor.get_visual("wing_neg_y")
    housing.get_visual("threshold")
    housing.get_visual("canopy")
    housing.get_visual("bottom_bearing")

    ctx.check(
        "rotor uses a vertical continuous articulation",
        spin.joint_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 4) for v in spin.axis) == (0.0, 0.0, 1.0)
        and spin.motion_limits is not None
        and spin.motion_limits.lower is None
        and spin.motion_limits.upper is None,
        details=f"type={spin.joint_type}, axis={spin.axis}, limits={spin.motion_limits}",
    )

    ctx.expect_within(
        rotor,
        housing,
        axes="xy",
        margin=0.0,
        name="rotor stays within housing footprint at rest",
    )
    ctx.expect_contact(
        rotor,
        housing,
        elem_a="bottom_hub",
        elem_b="bottom_bearing",
        name="bottom hub sits on floor bearing",
    )
    ctx.expect_gap(
        housing,
        rotor,
        axis="z",
        positive_elem="canopy",
        negative_elem="top_hub",
        min_gap=0.05,
        max_gap=0.12,
        name="top hub clears canopy",
    )

    rest_center = _world_aabb_center(ctx.part_element_world_aabb(rotor, elem="wing_pos_x"))
    with ctx.pose({spin: math.pi / 2.0}):
        quarter_turn_center = _world_aabb_center(ctx.part_element_world_aabb(rotor, elem="wing_pos_x"))
        ctx.expect_within(
            rotor,
            housing,
            axes="xy",
            margin=0.0,
            name="rotor stays within housing footprint at quarter turn",
        )

    ctx.check(
        "positive x wing rotates into the front opening on positive turn",
        rest_center is not None
        and quarter_turn_center is not None
        and rest_center[0] > 0.40
        and abs(rest_center[1]) < 0.10
        and quarter_turn_center[1] > 0.40
        and abs(quarter_turn_center[0]) < 0.10,
        details=f"rest_center={rest_center}, quarter_turn_center={quarter_turn_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
