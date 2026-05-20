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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hidden_shackle_security_padlock")

    body_finish = model.material("body_finish", rgba=(0.22, 0.23, 0.25, 1.0))
    faceplate_finish = model.material("faceplate_finish", rgba=(0.31, 0.32, 0.34, 1.0))
    shackle_steel = model.material("shackle_steel", rgba=(0.77, 0.79, 0.82, 1.0))
    brass = model.material("brass", rgba=(0.79, 0.64, 0.24, 1.0))
    keyway_dark = model.material("keyway_dark", rgba=(0.16, 0.13, 0.09, 1.0))

    body_width = 0.120
    body_depth = 0.046
    base_height = 0.056
    shield_height = 0.034
    body_front_y = 0.023

    pivot_x = -0.036
    pivot_z = base_height
    leg_spacing = 0.072

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, base_height + shield_height)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, (base_height + shield_height) * 0.5)),
    )

    base_profile = rounded_rect_profile(body_width, body_depth, 0.008, corner_segments=8)
    body.visual(
        _mesh("padlock_body_base", ExtrudeGeometry.from_z0(base_profile, base_height)),
        material=body_finish,
        name="body_base",
    )
    body.visual(
        Box((0.108, 0.010, shield_height)),
        origin=Origin(xyz=(0.0, -0.0185, base_height + shield_height * 0.5)),
        material=body_finish,
        name="rear_shield",
    )
    body.visual(
        Box((0.008, 0.026, shield_height)),
        origin=Origin(xyz=(-0.056, 0.004, base_height + shield_height * 0.5)),
        material=body_finish,
        name="left_shield",
    )
    body.visual(
        Box((0.016, 0.026, shield_height)),
        origin=Origin(xyz=(0.052, 0.004, base_height + shield_height * 0.5)),
        material=body_finish,
        name="right_shield",
    )
    body.visual(
        Box((0.062, 0.007, 0.038)),
        origin=Origin(xyz=(0.0, 0.0215, 0.030)),
        material=faceplate_finish,
        name="lock_face",
    )
    body.visual(
        Box((0.010, 0.016, 0.010)),
        origin=Origin(xyz=(-0.055, 0.010, 0.084)),
        material=faceplate_finish,
        name="top_lip",
    )
    body.visual(
        Box((0.010, 0.016, 0.010)),
        origin=Origin(xyz=(0.055, 0.010, 0.084)),
        material=faceplate_finish,
        name="right_top_cap",
    )

    shackle = model.part("shackle")
    shackle.inertial = Inertial.from_geometry(
        Box((0.088, 0.018, 0.074)),
        mass=0.34,
        origin=Origin(xyz=(0.036, 0.0, 0.037)),
    )

    shackle_loop = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.042),
            (0.010, 0.0, 0.055),
            (0.026, 0.0, 0.062),
            (0.046, 0.0, 0.062),
            (0.062, 0.0, 0.055),
            (leg_spacing, 0.0, 0.042),
            (leg_spacing, 0.0, 0.0),
        ],
        radius=0.0075,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    shackle.visual(
        _mesh("padlock_shackle_loop", shackle_loop),
        material=shackle_steel,
        name="loop",
    )
    shackle.visual(
        Cylinder(radius=0.0125, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=shackle_steel,
        name="pivot_collar",
    )
    shackle.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(leg_spacing, 0.0, 0.002)),
        material=shackle_steel,
        name="free_tip",
    )

    key_cylinder = model.part("key_cylinder")
    key_cylinder.inertial = Inertial.from_geometry(
        Box((0.024, 0.014, 0.024)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
    )
    key_cylinder.visual(
        Cylinder(radius=0.012, length=0.002),
        origin=Origin(xyz=(0.0, 0.001, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="plug_collar",
    )
    key_cylinder.visual(
        Cylinder(radius=0.0095, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="plug_body",
    )
    key_cylinder.visual(
        Box((0.003, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, 0.009, -0.002)),
        material=keyway_dark,
        name="keyway_slot",
    )
    key_cylinder.visual(
        Box((0.010, 0.0015, 0.0025)),
        origin=Origin(xyz=(0.0, 0.010, -0.0065)),
        material=keyway_dark,
        name="keyway_ward",
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "body_to_key_cylinder",
        ArticulationType.REVOLUTE,
        parent=body,
        child=key_cylinder,
        origin=Origin(xyz=(0.0, 0.025, 0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=-0.60, upper=0.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    key_cylinder = object_model.get_part("key_cylinder")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    key_joint = object_model.get_articulation("body_to_key_cylinder")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=12)

    ctx.check(
        "shackle_rotates_about_captive_leg_axis",
        tuple(shackle_joint.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical captive-leg axis, got {shackle_joint.axis}",
    )
    ctx.check(
        "key_cylinder_rotates_in_lock_face",
        tuple(key_joint.axis) == (0.0, 1.0, 0.0),
        details=f"expected front-face plug axis, got {key_joint.axis}",
    )

    ctx.expect_contact(
        shackle,
        body,
        elem_a="pivot_collar",
        elem_b="body_base",
        name="shackle_pivot_collar_seated",
    )
    ctx.expect_contact(
        key_cylinder,
        body,
        elem_a="plug_collar",
        elem_b="lock_face",
        name="key_plug_seated_in_faceplate",
    )
    ctx.expect_within(
        key_cylinder,
        body,
        axes="xz",
        margin=0.008,
        name="key_cylinder_centered_on_front_face",
    )

    with ctx.pose({shackle_joint: 1.05}):
        ctx.expect_contact(
            shackle,
            body,
            elem_a="pivot_collar",
            elem_b="body_base",
            name="shackle_stays_captive_when_open",
        )
        ctx.expect_gap(
            shackle,
            body,
            axis="y",
            positive_elem="free_tip",
            min_gap=0.020,
            name="free_leg_clears_body_front_when_open",
        )

    with ctx.pose({key_joint: 0.45}):
        ctx.expect_contact(
            key_cylinder,
            body,
            elem_a="plug_collar",
            elem_b="lock_face",
            name="key_plug_stays_seated_while_rotating",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
