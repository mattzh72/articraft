from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi

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


def _origin_x(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0))


def _origin_y(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sine_vise")

    ground_steel = model.material("ground_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    blued_steel = model.material("blued_steel", rgba=(0.40, 0.43, 0.48, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.14, 0.15, 0.16, 1.0))

    base_length = 0.190
    base_width = 0.082
    base_height = 0.024

    hinge_axis_x = -0.083
    hinge_axis_z = base_height + 0.005

    jaw_body_length = 0.154
    jaw_body_height = 0.042
    jaw_body_width = 0.078
    jaw_body_x_center = 0.085
    jaw_body_z_center = 0.017
    jaw_top_z = jaw_body_z_center + jaw_body_height / 2.0
    rail_height = 0.006
    rail_top_z = jaw_top_z + rail_height

    base = model.part("base")
    base.visual(
        Box((base_length, base_width, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
        material=ground_steel,
        name="main_block",
    )
    base.visual(
        Box((0.014, 0.020, 0.010)),
        origin=Origin(xyz=(hinge_axis_x, -0.027, hinge_axis_z)),
        material=ground_steel,
        name="left_hinge_lug",
    )
    base.visual(
        Box((0.014, 0.020, 0.010)),
        origin=Origin(xyz=(hinge_axis_x, 0.027, hinge_axis_z)),
        material=ground_steel,
        name="right_hinge_lug",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=_origin_y(hinge_axis_x, -0.027, hinge_axis_z),
        material=ground_steel,
        name="left_hinge_knuckle",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=_origin_y(hinge_axis_x, 0.027, hinge_axis_z),
        material=ground_steel,
        name="right_hinge_knuckle",
    )
    base.visual(
        Cylinder(radius=0.0035, length=0.076),
        origin=_origin_y(hinge_axis_x, 0.0, hinge_axis_z),
        material=black_oxide,
        name="hinge_pin",
    )
    base.visual(
        Box((0.018, 0.026, 0.024)),
        origin=Origin(xyz=(0.086, 0.0, 0.012)),
        material=blued_steel,
        name="rear_screw_housing",
    )
    base.visual(
        Cylinder(radius=0.005, length=0.022),
        origin=_origin_x(0.095, 0.0, 0.016),
        material=black_oxide,
        name="screw_shaft",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=_origin_x(0.101, 0.0, 0.016),
        material=black_oxide,
        name="adjustment_wheel",
    )

    jack_center = (0.058, 0.0, 0.023)
    strut_start = (0.083, 0.0, 0.017)
    strut_dx = jack_center[0] - strut_start[0]
    strut_dz = jack_center[2] - strut_start[2]
    strut_length = (strut_dx**2 + strut_dz**2) ** 0.5
    strut_pitch = atan2(strut_dx, strut_dz)

    base.visual(
        Cylinder(radius=0.004, length=strut_length),
        origin=Origin(
            xyz=(
                (strut_start[0] + jack_center[0]) / 2.0,
                0.0,
                (strut_start[2] + jack_center[2]) / 2.0,
            ),
            rpy=(0.0, strut_pitch, 0.0),
        ),
        material=black_oxide,
        name="jack_strut",
    )
    base.visual(
        Cylinder(radius=0.003, length=0.020),
        origin=_origin_y(*jack_center),
        material=ground_steel,
        name="jack_pad",
    )
    base.inertial = Inertial.from_geometry(
        Box((base_length, base_width, base_height)),
        mass=5.6,
        origin=Origin(xyz=(0.0, 0.0, base_height / 2.0)),
    )

    jaw = model.part("tilting_jaw")
    jaw.visual(
        Box((jaw_body_length, jaw_body_width, jaw_body_height)),
        origin=Origin(xyz=(jaw_body_x_center, 0.0, jaw_body_z_center)),
        material=blued_steel,
        name="jaw_body",
    )
    jaw.visual(
        Cylinder(radius=0.0055, length=0.032),
        origin=_origin_y(0.0, 0.0, 0.0),
        material=black_oxide,
        name="center_hinge_knuckle",
    )
    jaw.visual(
        Box((0.014, 0.028, 0.020)),
        origin=Origin(xyz=(0.005, 0.0, 0.010)),
        material=blued_steel,
        name="hinge_web",
    )
    jaw.visual(
        Box((0.010, jaw_body_width, 0.020)),
        origin=Origin(xyz=(0.017, 0.0, 0.048)),
        material=ground_steel,
        name="fixed_jaw_face",
    )
    jaw.visual(
        Box((0.118, 0.010, rail_height)),
        origin=Origin(xyz=(0.087, -0.029, jaw_top_z + rail_height / 2.0)),
        material=ground_steel,
        name="left_guide_rail",
    )
    jaw.visual(
        Box((0.118, 0.010, rail_height)),
        origin=Origin(xyz=(0.087, 0.029, jaw_top_z + rail_height / 2.0)),
        material=ground_steel,
        name="right_guide_rail",
    )
    jaw.visual(
        Box((0.028, 0.022, 0.004)),
        origin=Origin(xyz=(0.140, 0.0, -0.001)),
        material=ground_steel,
        name="rear_jack_pad",
    )
    jaw.inertial = Inertial.from_geometry(
        Box((jaw_body_length, jaw_body_width, jaw_body_height)),
        mass=3.8,
        origin=Origin(xyz=(jaw_body_x_center, 0.0, jaw_body_z_center)),
    )

    clamp = model.part("clamp_plate")
    clamp.visual(
        Box((0.045, jaw_body_width, 0.007)),
        origin=Origin(xyz=(0.0225, 0.0, 0.0035)),
        material=ground_steel,
        name="slider_base",
    )
    clamp.visual(
        Box((0.008, jaw_body_width, 0.026)),
        origin=Origin(xyz=(0.004, 0.0, 0.020)),
        material=ground_steel,
        name="jaw_face",
    )
    clamp.visual(
        Box((0.026, 0.050, 0.010)),
        origin=Origin(xyz=(0.019, 0.0, 0.031)),
        material=blued_steel,
        name="top_plate",
    )
    clamp.inertial = Inertial.from_geometry(
        Box((0.045, jaw_body_width, 0.033)),
        mass=0.9,
        origin=Origin(xyz=(0.0225, 0.0, 0.0165)),
    )

    tilt_limits = MotionLimits(
        effort=250.0,
        velocity=0.7,
        lower=0.0,
        upper=0.50,
    )
    slide_limits = MotionLimits(
        effort=120.0,
        velocity=0.08,
        lower=0.0,
        upper=0.060,
    )

    model.articulation(
        "base_to_tilting_jaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=jaw,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=tilt_limits,
    )
    model.articulation(
        "jaw_to_clamp_plate",
        ArticulationType.PRISMATIC,
        parent=jaw,
        child=clamp,
        origin=Origin(xyz=(0.030, 0.0, rail_top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=slide_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    jaw = object_model.get_part("tilting_jaw")
    clamp = object_model.get_part("clamp_plate")
    tilt_joint = object_model.get_articulation("base_to_tilting_jaw")
    slide_joint = object_model.get_articulation("jaw_to_clamp_plate")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        base,
        jaw,
        elem_a="hinge_pin",
        elem_b="center_hinge_knuckle",
        reason="The hinge pin is intentionally modeled as passing through the tilting jaw barrel.",
    )

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
        base,
        jaw,
        elem_a="jack_pad",
        elem_b="rear_jack_pad",
        name="rear jackscrew pad supports the tilting jaw at rest",
    )
    ctx.expect_contact(
        clamp,
        jaw,
        elem_a="slider_base",
        elem_b="left_guide_rail",
        name="clamp plate rides on the jaw guide rail",
    )
    ctx.expect_gap(
        clamp,
        jaw,
        axis="x",
        positive_elem="jaw_face",
        negative_elem="fixed_jaw_face",
        min_gap=0.004,
        max_gap=0.008,
        name="clamp starts slightly open from the fixed jaw",
    )

    jaw_rest_aabb = ctx.part_element_world_aabb(jaw, elem="rear_jack_pad")
    with ctx.pose({tilt_joint: tilt_joint.motion_limits.upper}):
        jaw_open_aabb = ctx.part_element_world_aabb(jaw, elem="rear_jack_pad")
    ctx.check(
        "tilting jaw rear rises when the sine vise opens",
        jaw_rest_aabb is not None
        and jaw_open_aabb is not None
        and jaw_open_aabb[0][2] > jaw_rest_aabb[0][2] + 0.025,
        details=f"rest={jaw_rest_aabb}, open={jaw_open_aabb}",
    )

    clamp_rest_pos = ctx.part_world_position(clamp)
    with ctx.pose({slide_joint: slide_joint.motion_limits.upper}):
        clamp_open_pos = ctx.part_world_position(clamp)
    ctx.check(
        "clamp plate retracts rearward along the jaw ways",
        clamp_rest_pos is not None
        and clamp_open_pos is not None
        and clamp_open_pos[0] > clamp_rest_pos[0] + 0.050,
        details=f"rest={clamp_rest_pos}, open={clamp_open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
