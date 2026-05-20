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


def _y_axis_cylinder(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.13, 0.14, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_smoke = model.material("dark_smoke", rgba=(0.30, 0.33, 0.36, 1.0))
    accent_red = model.material("accent_red", rgba=(0.78, 0.16, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.090, 0.120, 0.260)),
        origin=Origin(xyz=(0.025, 0.0, 0.171)),
        material=graphite,
        name="main_shell",
    )
    body.visual(
        Box((0.068, 0.050, 0.030)),
        origin=Origin(xyz=(0.000, 0.0, 0.026)),
        material=graphite,
        name="lower_mount_block",
    )
    body.visual(
        Box((0.062, 0.048, 0.055)),
        origin=Origin(xyz=(0.000, 0.0, 0.065)),
        material=graphite,
        name="hinge_spine",
    )
    body.visual(
        Box((0.080, 0.040, 0.040)),
        origin=Origin(xyz=(0.020, 0.0, 0.362)),
        material=graphite,
        name="handle_bridge",
    )
    body.visual(
        Box((0.036, 0.034, 0.210)),
        origin=Origin(xyz=(-0.030, 0.0, 0.257)),
        material=graphite,
        name="rear_grip",
    )
    body.visual(
        Cylinder(radius=0.042, length=0.140),
        origin=Origin(xyz=(0.066, 0.0, 0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_smoke,
        name="dust_cup",
    )
    body.visual(
        Box((0.050, 0.080, 0.055)),
        origin=Origin(xyz=(0.105, 0.0, 0.165)),
        material=graphite,
        name="front_inlet_nose",
    )
    body.visual(
        Box((0.018, 0.012, 0.034)),
        origin=Origin(xyz=(0.000, 0.028, 0.000)),
        material=charcoal,
        name="left_clevis",
    )
    body.visual(
        Box((0.018, 0.012, 0.034)),
        origin=Origin(xyz=(0.000, -0.028, 0.000)),
        material=charcoal,
        name="right_clevis",
    )
    _y_axis_cylinder(
        body,
        name="left_pin_cap",
        radius=0.011,
        length=0.004,
        xyz=(0.000, 0.036, 0.000),
        material=charcoal,
    )
    _y_axis_cylinder(
        body,
        name="right_pin_cap",
        radius=0.011,
        length=0.004,
        xyz=(0.000, -0.036, 0.000),
        material=charcoal,
    )
    body.visual(
        Box((0.018, 0.018, 0.010)),
        origin=Origin(xyz=(0.098, 0.0, 0.241)),
        material=accent_red,
        name="bin_release_button",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.160, 0.120, 0.400)),
        mass=1.9,
        origin=Origin(xyz=(0.030, 0.0, 0.190)),
    )

    wand = model.part("wand")
    _y_axis_cylinder(
        wand,
        name="top_knuckle",
        radius=0.0105,
        length=0.044,
        xyz=(0.000, 0.0, 0.000),
        material=satin_silver,
    )
    wand.visual(
        Box((0.038, 0.036, 0.090)),
        origin=Origin(xyz=(0.000, 0.0, -0.050)),
        material=satin_silver,
        name="upper_clamp_block",
    )
    wand.visual(
        Box((0.050, 0.034, 0.016)),
        origin=Origin(xyz=(0.000, 0.0, -0.088)),
        material=graphite,
        name="fold_lock_collar",
    )
    wand.visual(
        Box((0.042, 0.028, 0.540)),
        origin=Origin(xyz=(0.000, 0.0, -0.366)),
        material=satin_silver,
        name="wand_tube",
    )
    wand.visual(
        Box((0.012, 0.012, 0.450)),
        origin=Origin(xyz=(-0.018, 0.0, -0.345)),
        material=graphite,
        name="wire_channel",
    )
    wand.visual(
        Box((0.048, 0.040, 0.026)),
        origin=Origin(xyz=(0.000, 0.0, -0.675)),
        material=graphite,
        name="lower_yoke_block",
    )
    wand.visual(
        Box((0.022, 0.022, 0.050)),
        origin=Origin(xyz=(0.000, 0.0, -0.656)),
        material=graphite,
        name="lower_bridge",
    )
    wand.visual(
        Box((0.014, 0.012, 0.034)),
        origin=Origin(xyz=(0.000, 0.025, -0.705)),
        material=graphite,
        name="lower_left_ear",
    )
    wand.visual(
        Box((0.014, 0.012, 0.034)),
        origin=Origin(xyz=(0.000, -0.025, -0.705)),
        material=graphite,
        name="lower_right_ear",
    )
    _y_axis_cylinder(
        wand,
        name="lower_left_pin_cap",
        radius=0.0095,
        length=0.004,
        xyz=(0.000, 0.033, -0.705),
        material=charcoal,
    )
    _y_axis_cylinder(
        wand,
        name="lower_right_pin_cap",
        radius=0.0095,
        length=0.004,
        xyz=(0.000, -0.033, -0.705),
        material=charcoal,
    )
    wand.visual(
        Box((0.016, 0.010, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, -0.088)),
        material=accent_red,
        name="fold_latch_button",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.055, 0.045, 0.740)),
        mass=0.55,
        origin=Origin(xyz=(0.000, 0.0, -0.370)),
    )

    floor_head = model.part("floor_head")
    _y_axis_cylinder(
        floor_head,
        name="neck_knuckle",
        radius=0.0095,
        length=0.038,
        xyz=(0.000, 0.0, 0.000),
        material=charcoal,
    )
    floor_head.visual(
        Box((0.028, 0.030, 0.052)),
        origin=Origin(xyz=(0.000, 0.0, -0.031)),
        material=graphite,
        name="neck_stem",
    )
    floor_head.visual(
        Box((0.055, 0.060, 0.040)),
        origin=Origin(xyz=(0.018, 0.0, -0.059)),
        material=graphite,
        name="neck_tower",
    )
    floor_head.visual(
        Box((0.270, 0.105, 0.028)),
        origin=Origin(xyz=(0.030, 0.0, -0.090)),
        material=charcoal,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.085, 0.105, 0.014)),
        origin=Origin(xyz=(0.123, 0.0, -0.078)),
        material=graphite,
        name="front_lip",
    )
    floor_head.visual(
        Box((0.090, 0.105, 0.024)),
        origin=Origin(xyz=(-0.070, 0.0, -0.086)),
        material=graphite,
        name="rear_roller_housing",
    )
    floor_head.visual(
        Box((0.070, 0.055, 0.030)),
        origin=Origin(xyz=(0.010, 0.0, -0.072)),
        material=graphite,
        name="suction_throat",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.280, 0.110, 0.110)),
        mass=0.75,
        origin=Origin(xyz=(0.030, 0.0, -0.060)),
    )

    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=3.0,
            lower=-0.10,
            upper=1.25,
        ),
    )
    model.articulation(
        "wand_to_head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.000, 0.0, -0.705)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=4.0,
            lower=-0.55,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("body_to_wand_fold")
    nozzle_joint = object_model.get_articulation("wand_to_head_pitch")

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
        wand,
        body,
        elem_a="top_knuckle",
        elem_b="left_clevis",
        name="fold_joint_left_bracket_contact",
    )
    ctx.expect_contact(
        wand,
        body,
        elem_a="top_knuckle",
        elem_b="right_clevis",
        name="fold_joint_right_bracket_contact",
    )
    ctx.expect_contact(
        floor_head,
        wand,
        elem_a="neck_knuckle",
        elem_b="lower_left_ear",
        name="nozzle_joint_left_bracket_contact",
    )
    ctx.expect_contact(
        floor_head,
        wand,
        elem_a="neck_knuckle",
        elem_b="lower_right_ear",
        name="nozzle_joint_right_bracket_contact",
    )

    with ctx.pose({fold_joint: 0.0, nozzle_joint: 0.0}):
        ctx.expect_origin_gap(
            body,
            floor_head,
            axis="z",
            min_gap=0.68,
            max_gap=0.74,
            name="upright_stack_height_is_realistic",
        )
        ctx.expect_overlap(
            wand,
            floor_head,
            axes="xy",
            min_overlap=0.025,
            name="wand_feeds_directly_into_head_neck",
        )

    with ctx.pose({fold_joint: 1.0, nozzle_joint: 0.25}):
        ctx.expect_origin_gap(
            floor_head,
            body,
            axis="x",
            min_gap=0.50,
            name="fold_joint_swings_head_forward_for_storage",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
