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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _wheel_shell_geometry(radius: float, width: float):
    half_width = width * 0.5
    profile = [
        (0.0, -half_width),
        (0.030, -half_width),
        (0.040, -half_width * 0.96),
        (0.060, -half_width * 0.88),
        (0.082, -half_width * 0.78),
        (0.094, -half_width * 0.52),
        (radius * 0.985, -half_width * 0.18),
        (radius, 0.0),
        (radius * 0.985, half_width * 0.18),
        (0.094, half_width * 0.52),
        (0.082, half_width * 0.78),
        (0.060, half_width * 0.88),
        (0.040, half_width * 0.96),
        (0.030, half_width),
        (0.0, half_width),
    ]
    return LatheGeometry(profile, segments=56).rotate_y(math.pi / 2.0)


def _add_wheel_visuals(
    part,
    *,
    mesh_name: str,
    tire_material,
    hub_material,
    accent_material,
    inner_face_sign: float,
    radius: float = 0.100,
    width: float = 0.055,
) -> None:
    spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    half_width = width * 0.5
    inner_face_center = inner_face_sign * (half_width - 0.005)
    outer_face_center = -inner_face_center

    part.visual(
        _save_mesh(mesh_name, _wheel_shell_geometry(radius, width)),
        material=tire_material,
        name="wheel_shell",
    )
    part.visual(
        Cylinder(radius=0.038, length=0.010),
        origin=Origin(xyz=(inner_face_center, 0.0, 0.0), rpy=spin_origin.rpy),
        material=hub_material,
        name="inner_hub_face",
    )
    part.visual(
        Cylinder(radius=0.031, length=0.037),
        origin=spin_origin,
        material=hub_material,
        name="hub_barrel",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(outer_face_center, 0.0, 0.0), rpy=spin_origin.rpy),
        material=hub_material,
        name="outer_cap",
    )
    part.visual(
        Box((0.006, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, radius * 0.90)),
        material=accent_material,
        name="rotation_index",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_wheelie_bin")

    body_plastic = model.material("body_plastic", rgba=(0.33, 0.39, 0.44, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.37, 0.44, 0.49, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_hub = model.material("dark_hub", rgba=(0.18, 0.19, 0.21, 1.0))
    steel = model.material("steel", rgba=(0.34, 0.35, 0.38, 1.0))
    datum_accent = model.material("datum_accent", rgba=(0.82, 0.63, 0.16, 1.0))

    body = model.part("bin_body")
    body.inertial = Inertial.from_geometry(
        Box((0.60, 0.80, 1.02)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
    )

    body.visual(
        Box((0.476, 0.596, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.127)),
        material=body_plastic,
        name="floor",
    )
    body.visual(
        Box((0.536, 0.018, 0.820)),
        origin=Origin(xyz=(0.0, 0.332, 0.530), rpy=(-0.091, 0.0, 0.0)),
        material=body_plastic,
        name="front_wall",
    )
    body.visual(
        Box((0.536, 0.018, 0.820)),
        origin=Origin(xyz=(0.0, -0.332, 0.530), rpy=(0.091, 0.0, 0.0)),
        material=body_plastic,
        name="rear_wall",
    )
    body.visual(
        Box((0.018, 0.650, 0.820)),
        origin=Origin(xyz=(0.257, 0.0, 0.530), rpy=(0.0, 0.067, 0.0)),
        material=body_plastic,
        name="left_wall",
    )
    body.visual(
        Box((0.018, 0.650, 0.820)),
        origin=Origin(xyz=(-0.257, 0.0, 0.530), rpy=(0.0, -0.067, 0.0)),
        material=body_plastic,
        name="right_wall",
    )
    body.visual(
        Box((0.120, 0.050, 0.190)),
        origin=Origin(xyz=(0.0, 0.279, 0.095)),
        material=body_plastic,
        name="front_foot_web",
    )
    body.visual(
        Box((0.240, 0.050, 0.035)),
        origin=Origin(xyz=(0.0, 0.287, 0.0175)),
        material=body_plastic,
        name="front_foot_rail",
    )
    body.visual(
        Box((0.040, 0.660, 0.028)),
        origin=Origin(xyz=(0.280, 0.0, 0.950)),
        material=body_plastic,
        name="left_top_rim",
    )
    body.visual(
        Box((0.040, 0.660, 0.028)),
        origin=Origin(xyz=(-0.280, 0.0, 0.950)),
        material=body_plastic,
        name="right_top_rim",
    )
    body.visual(
        Box((0.560, 0.040, 0.028)),
        origin=Origin(xyz=(0.0, 0.350, 0.950)),
        material=body_plastic,
        name="front_top_rim",
    )
    body.visual(
        Box((0.180, 0.024, 0.008)),
        origin=Origin(xyz=(0.0, 0.352, 0.963)),
        material=datum_accent,
        name="front_alignment_rail",
    )
    body.visual(
        Box((0.008, 0.026, 0.004)),
        origin=Origin(xyz=(0.0, 0.352, 0.967)),
        material=steel,
        name="body_index_center",
    )
    body.visual(
        Box((0.008, 0.022, 0.004)),
        origin=Origin(xyz=(0.050, 0.352, 0.967)),
        material=steel,
        name="body_index_right",
    )
    body.visual(
        Box((0.008, 0.022, 0.004)),
        origin=Origin(xyz=(-0.050, 0.352, 0.967)),
        material=steel,
        name="body_index_left",
    )
    body.visual(
        Box((0.150, 0.006, 0.070)),
        origin=Origin(xyz=(0.0, 0.358, 0.635), rpy=(-0.091, 0.0, 0.0)),
        material=steel,
        name="front_datum_plaque",
    )
    body.visual(
        Box((0.150, 0.022, 0.080)),
        origin=Origin(xyz=(0.0, 0.352, 0.635), rpy=(-0.091, 0.0, 0.0)),
        material=body_plastic,
        name="front_datum_boss",
    )
    body.visual(
        Box((0.014, 0.150, 0.090)),
        origin=Origin(xyz=(0.279, 0.050, 0.620), rpy=(0.0, 0.067, 0.0)),
        material=steel,
        name="left_side_datum_pad",
    )
    body.visual(
        Box((0.014, 0.150, 0.090)),
        origin=Origin(xyz=(-0.279, 0.050, 0.620), rpy=(0.0, -0.067, 0.0)),
        material=steel,
        name="right_side_datum_pad",
    )
    body.visual(
        Box((0.055, 0.060, 0.020)),
        origin=Origin(xyz=(0.115, -0.405, 0.163)),
        material=body_plastic,
        name="left_axle_saddle",
    )
    body.visual(
        Box((0.055, 0.060, 0.020)),
        origin=Origin(xyz=(-0.115, -0.405, 0.163)),
        material=body_plastic,
        name="right_axle_saddle",
    )
    body.visual(
        Box((0.034, 0.160, 0.185)),
        origin=Origin(xyz=(0.115, -0.365, 0.245)),
        material=body_plastic,
        name="left_axle_bracket",
    )
    body.visual(
        Box((0.034, 0.160, 0.185)),
        origin=Origin(xyz=(-0.115, -0.365, 0.245)),
        material=body_plastic,
        name="right_axle_bracket",
    )
    body.visual(
        Box((0.560, 0.090, 0.036)),
        origin=Origin(xyz=(0.0, -0.375, 0.955)),
        material=body_plastic,
        name="rear_hinge_bridge",
    )
    body.visual(
        Box((0.060, 0.080, 0.052)),
        origin=Origin(xyz=(0.270, -0.380, 0.947)),
        material=body_plastic,
        name="left_hinge_connector",
    )
    body.visual(
        Box((0.060, 0.080, 0.052)),
        origin=Origin(xyz=(-0.270, -0.380, 0.947)),
        material=body_plastic,
        name="right_hinge_connector",
    )
    body.visual(
        Box((0.016, 0.004, 0.026)),
        origin=Origin(xyz=(0.270, -0.416, 0.960)),
        material=datum_accent,
        name="left_hinge_scale",
    )
    body.visual(
        Box((0.016, 0.004, 0.026)),
        origin=Origin(xyz=(-0.270, -0.416, 0.960)),
        material=datum_accent,
        name="right_hinge_scale",
    )

    axle = model.part("rear_axle")
    axle.inertial = Inertial.from_geometry(
        Box((0.58, 0.08, 0.10)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )
    axle.visual(
        Cylinder(radius=0.011, length=0.556),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_tube",
    )
    axle.visual(
        Box((0.048, 0.058, 0.050)),
        origin=Origin(xyz=(0.115, 0.0, 0.028)),
        material=steel,
        name="left_adjust_block",
    )
    axle.visual(
        Box((0.048, 0.058, 0.050)),
        origin=Origin(xyz=(-0.115, 0.0, 0.028)),
        material=steel,
        name="right_adjust_block",
    )
    axle.visual(
        Box((0.040, 0.004, 0.030)),
        origin=Origin(xyz=(0.115, 0.031, 0.028)),
        material=datum_accent,
        name="left_adjust_scale",
    )
    axle.visual(
        Box((0.040, 0.004, 0.030)),
        origin=Origin(xyz=(-0.115, 0.031, 0.028)),
        material=datum_accent,
        name="right_adjust_scale",
    )
    axle.visual(
        Cylinder(radius=0.035, length=0.010),
        origin=Origin(xyz=(0.2825, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_hub_flange",
    )
    axle.visual(
        Cylinder(radius=0.035, length=0.010),
        origin=Origin(xyz=(-0.2825, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_hub_flange",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.60, 0.76, 0.06)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.37, -0.004)),
    )
    lid.visual(
        Box((0.608, 0.754, 0.014)),
        origin=Origin(xyz=(0.0, 0.377, 0.005)),
        material=lid_plastic,
        name="top_panel",
    )
    lid.visual(
        Box((0.012, 0.708, 0.024)),
        origin=Origin(xyz=(0.304, 0.372, -0.014)),
        material=lid_plastic,
        name="left_skirt",
    )
    lid.visual(
        Box((0.012, 0.708, 0.024)),
        origin=Origin(xyz=(-0.304, 0.372, -0.014)),
        material=lid_plastic,
        name="right_skirt",
    )
    lid.visual(
        Box((0.560, 0.014, 0.024)),
        origin=Origin(xyz=(0.0, 0.761, -0.014)),
        material=lid_plastic,
        name="front_skirt",
    )
    lid.visual(
        Box((0.500, 0.044, 0.016)),
        origin=Origin(xyz=(0.0, 0.012, 0.009)),
        material=lid_plastic,
        name="hinge_leaf",
    )
    lid.visual(
        Box((0.180, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.726, -0.001)),
        material=datum_accent,
        name="front_datum_pad",
    )
    lid.visual(
        Box((0.190, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, 0.690, 0.011)),
        material=steel,
        name="top_datum_strip",
    )
    lid.visual(
        Box((0.008, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, 0.690, 0.014)),
        material=datum_accent,
        name="lid_index_center",
    )
    lid.visual(
        Box((0.008, 0.020, 0.004)),
        origin=Origin(xyz=(0.050, 0.690, 0.014)),
        material=datum_accent,
        name="lid_index_right",
    )
    lid.visual(
        Box((0.008, 0.020, 0.004)),
        origin=Origin(xyz=(-0.050, 0.690, 0.014)),
        material=datum_accent,
        name="lid_index_left",
    )
    lid.visual(
        Box((0.070, 0.030, 0.006)),
        origin=Origin(xyz=(0.205, 0.080, 0.010)),
        material=steel,
        name="left_adjust_pad",
    )
    lid.visual(
        Box((0.070, 0.030, 0.006)),
        origin=Origin(xyz=(-0.205, 0.080, 0.010)),
        material=steel,
        name="right_adjust_pad",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.055),
        mass=1.7,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        left_wheel,
        mesh_name="left_wheel_shell",
        tire_material=wheel_rubber,
        hub_material=dark_hub,
        accent_material=datum_accent,
        inner_face_sign=-1.0,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.055),
        mass=1.7,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        right_wheel,
        mesh_name="right_wheel_shell",
        tire_material=wheel_rubber,
        hub_material=dark_hub,
        accent_material=datum_accent,
        inner_face_sign=1.0,
    )

    model.articulation(
        "body_to_axle",
        ArticulationType.FIXED,
        parent=body,
        child=axle,
        origin=Origin(xyz=(0.0, -0.405, 0.100)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.372, 0.972)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )
    model.articulation(
        "axle_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=left_wheel,
        origin=Origin(xyz=(0.315, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )
    model.articulation(
        "axle_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=right_wheel,
        origin=Origin(xyz=(-0.315, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    body = object_model.get_part("bin_body")
    axle = object_model.get_part("rear_axle")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    lid_hinge = object_model.get_articulation("body_to_lid")
    left_spin = object_model.get_articulation("axle_to_left_wheel")
    right_spin = object_model.get_articulation("axle_to_right_wheel")

    ctx.check(
        "lid_hinge_axis_and_limits",
        lid_hinge.axis == (1.0, 0.0, 0.0)
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper > math.radians(90.0),
        f"Unexpected lid hinge setup: axis={lid_hinge.axis}, limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "wheel_spin_axes_are_coaxial_with_axle",
        left_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_spin.articulation_type == ArticulationType.CONTINUOUS
        and left_spin.axis == (1.0, 0.0, 0.0)
        and right_spin.axis == (1.0, 0.0, 0.0),
        (
            "Wheel spin articulations should be continuous and aligned with the rear axle: "
            f"left={left_spin.axis}, right={right_spin.axis}"
        ),
    )

    ctx.expect_contact(
        axle,
        body,
        elem_a="left_adjust_block",
        elem_b="left_axle_saddle",
        name="left_axle_block_seated",
    )
    ctx.expect_contact(
        axle,
        body,
        elem_a="right_adjust_block",
        elem_b="right_axle_saddle",
        name="right_axle_block_seated",
    )
    ctx.expect_contact(
        left_wheel,
        axle,
        elem_a="inner_hub_face",
        elem_b="left_hub_flange",
        name="left_wheel_supported_by_axle",
    )
    ctx.expect_contact(
        right_wheel,
        axle,
        elem_a="inner_hub_face",
        elem_b="right_hub_flange",
        name="right_wheel_supported_by_axle",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_contact(
            lid,
            body,
            elem_a="front_datum_pad",
            elem_b="front_alignment_rail",
            name="lid_front_datum_contacts_body_rail",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            elem_a="front_datum_pad",
            elem_b="front_alignment_rail",
            min_overlap=0.150,
            name="lid_front_datum_registers_in_x",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="top_panel",
            negative_elem="left_top_rim",
            max_gap=0.007,
            max_penetration=0.0,
            name="lid_height_above_left_rim_is_controlled",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="top_panel",
            min_overlap=0.50,
            name="lid_panel_covers_body_opening",
        )

    with ctx.pose({lid_hinge: math.radians(80.0)}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_datum_pad",
            negative_elem="front_alignment_rail",
            min_gap=0.18,
            name="lid_opens_clear_of_front_alignment_rail",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
