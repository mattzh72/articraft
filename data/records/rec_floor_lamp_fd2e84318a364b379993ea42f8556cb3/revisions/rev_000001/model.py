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


def _build_reading_shade_shell():
    outer_profile = [
        (0.034, 0.000),
        (0.038, 0.018),
        (0.043, 0.060),
        (0.049, 0.120),
        (0.056, 0.182),
    ]
    inner_profile = [
        (0.000, 0.004),
        (0.029, 0.008),
        (0.034, 0.018),
        (0.039, 0.060),
        (0.045, 0.120),
        (0.052, 0.176),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
        ),
        "reading_shade_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_lamp_with_reading_boom")

    powder_black = model.material("powder_black", rgba=(0.13, 0.14, 0.15, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.65, 0.68, 1.0))
    warm_white = model.material("warm_white", rgba=(0.92, 0.91, 0.87, 1.0))
    charcoal = model.material("charcoal", rgba=(0.21, 0.22, 0.24, 1.0))

    base_radius = 0.17
    base_thickness = 0.04
    plinth_height = 0.03
    column_height = 1.34
    shoulder_z = base_thickness + plinth_height + column_height + 0.04
    boom_length = 0.48
    shade_pitch = -math.radians(35.0)

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=base_radius, length=base_thickness),
        origin=Origin(xyz=(0.0, 0.0, base_thickness * 0.5)),
        material=powder_black,
        name="weighted_base",
    )
    stand.visual(
        Cylinder(radius=0.05, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=powder_black,
        name="base_plinth",
    )
    stand.visual(
        Cylinder(radius=0.014, length=1.348),
        origin=Origin(xyz=(0.0, 0.0, 0.740)),
        material=satin_steel,
        name="column",
    )
    stand.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, shoulder_z - 0.026)),
        material=charcoal,
        name="shoulder_collar",
    )
    stand.visual(
        Box((0.026, 0.010, 0.058)),
        origin=Origin(xyz=(0.0, 0.027, shoulder_z + 0.001)),
        material=charcoal,
        name="left_yoke",
    )
    stand.visual(
        Box((0.026, 0.010, 0.058)),
        origin=Origin(xyz=(0.0, -0.027, shoulder_z + 0.001)),
        material=charcoal,
        name="right_yoke",
    )
    stand.visual(
        Box((0.022, 0.064, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, shoulder_z + 0.028)),
        material=charcoal,
        name="yoke_bridge",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, shoulder_z + 0.04)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, (shoulder_z + 0.04) * 0.5)),
    )

    boom = model.part("boom_arm")
    boom.visual(
        Cylinder(radius=0.012, length=0.044),
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=charcoal,
        name="shoulder_barrel",
    )
    boom.visual(
        Box((0.044, 0.034, 0.020)),
        origin=Origin(xyz=(0.022, 0.0, 0.017)),
        material=charcoal,
        name="shoulder_knuckle",
    )
    boom.visual(
        Cylinder(radius=0.012, length=boom_length),
        origin=Origin(xyz=(boom_length * 0.5, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=satin_steel,
        name="arm_tube",
    )
    boom.visual(
        Box((0.032, 0.054, 0.028)),
        origin=Origin(xyz=(boom_length - 0.016, 0.0, 0.0)),
        material=charcoal,
        name="tip_block",
    )
    boom.visual(
        Box((0.032, 0.014, 0.050)),
        origin=Origin(xyz=(boom_length + 0.010, 0.026, 0.0)),
        material=charcoal,
        name="left_fork",
    )
    boom.visual(
        Box((0.032, 0.014, 0.050)),
        origin=Origin(xyz=(boom_length + 0.010, -0.026, 0.0)),
        material=charcoal,
        name="right_fork",
    )
    boom.visual(
        Box((0.028, 0.066, 0.012)),
        origin=Origin(xyz=(boom_length + 0.004, 0.0, 0.021)),
        material=charcoal,
        name="fork_bridge",
    )
    boom.inertial = Inertial.from_geometry(
        Box((boom_length + 0.06, 0.08, 0.08)),
        mass=1.2,
        origin=Origin(xyz=(((boom_length + 0.06) * 0.5), 0.0, 0.0)),
    )

    shade = model.part("reading_shade")
    shade.visual(
        Cylinder(radius=0.0095, length=0.038),
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=charcoal,
        name="tilt_barrel",
    )
    shade.visual(
        Box((0.034, 0.028, 0.024)),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, shade_pitch, 0.0)),
        material=charcoal,
        name="shade_knuckle",
    )
    shade.visual(
        Cylinder(radius=0.017, length=0.026),
        origin=Origin(
            xyz=(0.052, 0.0, 0.0),
            rpy=(0.0, shade_pitch + (math.pi * 0.5), 0.0),
        ),
        material=charcoal,
        name="socket_housing",
    )
    shade.visual(
        _build_reading_shade_shell(),
        origin=Origin(
            xyz=(0.060, 0.0, 0.0),
            rpy=(0.0, shade_pitch + (math.pi * 0.5), 0.0),
        ),
        material=warm_white,
        name="shade_shell",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.24, 0.14, 0.14)),
        mass=0.8,
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, shoulder_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=math.radians(-20.0),
            upper=math.radians(68.0),
        ),
    )
    model.articulation(
        "shade_tilt_joint",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=shade,
        origin=Origin(xyz=(boom_length + 0.015, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=math.radians(-42.0),
            upper=math.radians(34.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    boom = object_model.get_part("boom_arm")
    shade = object_model.get_part("reading_shade")
    shoulder = object_model.get_articulation("shoulder_joint")
    shade_tilt = object_model.get_articulation("shade_tilt_joint")

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
        boom,
        stand,
        elem_a="shoulder_barrel",
        elem_b="left_yoke",
        name="boom barrel contacts left shoulder yoke",
    )
    ctx.expect_contact(
        boom,
        stand,
        elem_a="shoulder_barrel",
        elem_b="right_yoke",
        name="boom barrel contacts right shoulder yoke",
    )
    ctx.expect_contact(
        shade,
        boom,
        elem_a="tilt_barrel",
        elem_b="left_fork",
        name="shade tilt barrel contacts left fork",
    )
    ctx.expect_contact(
        shade,
        boom,
        elem_a="tilt_barrel",
        elem_b="right_fork",
        name="shade tilt barrel contacts right fork",
    )
    ctx.expect_origin_gap(
        shade,
        stand,
        axis="z",
        min_gap=1.30,
        name="shade sits well above the floor base",
    )

    rest_shade_pos = ctx.part_world_position(shade)
    shoulder_upper = shoulder.motion_limits.upper if shoulder.motion_limits is not None else None
    with ctx.pose({shoulder: shoulder_upper}):
        raised_shade_pos = ctx.part_world_position(shade)

    ctx.check(
        "shoulder raises the reading head",
        rest_shade_pos is not None
        and raised_shade_pos is not None
        and shoulder_upper is not None
        and raised_shade_pos[2] > rest_shade_pos[2] + 0.20,
        details=f"rest={rest_shade_pos}, raised={raised_shade_pos}, upper={shoulder_upper}",
    )

    def element_center_z(part_obj, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    shade_lower = shade_tilt.motion_limits.lower if shade_tilt.motion_limits is not None else None
    shade_upper = shade_tilt.motion_limits.upper if shade_tilt.motion_limits is not None else None
    with ctx.pose({shoulder: 0.0, shade_tilt: shade_lower}):
        shade_center_at_lower_limit = element_center_z(shade, "shade_shell")
    with ctx.pose({shoulder: 0.0, shade_tilt: shade_upper}):
        shade_center_at_upper_limit = element_center_z(shade, "shade_shell")

    ctx.check(
        "shade tilt sweeps the reading head upward",
        shade_center_at_lower_limit is not None
        and shade_center_at_upper_limit is not None
        and shade_lower is not None
        and shade_upper is not None
        and shade_center_at_upper_limit > shade_center_at_lower_limit + 0.04,
        details=(
            f"lower_pose_center_z={shade_center_at_lower_limit}, "
            f"upper_pose_center_z={shade_center_at_upper_limit}, "
            f"limits=({shade_lower}, {shade_upper})"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
