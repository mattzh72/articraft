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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bar_mount_faucet")

    chrome = model.material("chrome", rgba=(0.83, 0.84, 0.86, 1.0))
    satin = model.material("satin_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    dark = model.material("dark_rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")

    body.visual(
        Cylinder(radius=0.011, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=satin,
        name="mounting_shank",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=chrome,
        name="base_flange",
    )
    body.visual(
        Cylinder(radius=0.019, length=0.222),
        origin=Origin(xyz=(0.0, 0.0, 0.119)),
        material=chrome,
        name="column_shell",
    )
    body.visual(
        Sphere(radius=0.019),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=chrome,
        name="top_dome",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.224), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="spout_root",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.112),
        origin=Origin(xyz=(0.056, 0.0, 0.224), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="spout_shell",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.0, 0.029, 0.170), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_mount",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="handle_hub",
    )
    handle.visual(
        Box((0.010, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.020, 0.010)),
        material=satin,
        name="handle_web",
    )

    blade_geom = ExtrudeGeometry.centered(
        rounded_rect_profile(0.016, 0.068, 0.0035),
        0.004,
    )
    blade_geom.rotate_x(-pi / 2.0)
    handle.visual(
        mesh_from_geometry(blade_geom, "handle_blade"),
        origin=Origin(xyz=(0.0, 0.031, 0.026)),
        material=satin,
        name="handle_blade",
    )

    aerator = model.part("aerator")
    aerator.visual(
        Cylinder(radius=0.0135, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="aerator_collar",
    )
    aerator.visual(
        Cylinder(radius=0.0125, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="aerator_shell",
    )
    aerator.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark,
        name="aerator_outlet",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.050, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.25,
            upper=0.75,
        ),
    )
    model.articulation(
        "body_to_aerator",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=aerator,
        origin=Origin(xyz=(0.112, 0.0, 0.224)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    aerator = object_model.get_part("aerator")
    handle_joint = object_model.get_articulation("body_to_handle")
    aerator_joint = object_model.get_articulation("body_to_aerator")

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

    ctx.check(
        "handle joint is revolute",
        handle_joint.articulation_type == ArticulationType.REVOLUTE,
        f"expected REVOLUTE, got {handle_joint.articulation_type}",
    )
    ctx.check(
        "handle joint axis runs along spout",
        tuple(handle_joint.axis) == (1.0, 0.0, 0.0),
        f"expected axis (1, 0, 0), got {handle_joint.axis}",
    )
    ctx.check(
        "aerator joint rotates around spout axis",
        aerator_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(aerator_joint.axis) == (1.0, 0.0, 0.0),
        (
            "expected continuous aerator joint around x axis, "
            f"got type={aerator_joint.articulation_type}, axis={aerator_joint.axis}"
        ),
    )

    ctx.expect_origin_gap(
        handle,
        body,
        axis="y",
        min_gap=0.0495,
        max_gap=0.0505,
        name="handle sits off the right side of the body",
    )
    ctx.expect_origin_gap(
        aerator,
        body,
        axis="x",
        min_gap=0.1115,
        max_gap=0.1125,
        name="aerator sits at the spout tip",
    )
    ctx.expect_contact(
        handle,
        body,
        elem_a="handle_hub",
        elem_b="handle_mount",
        name="handle hub contacts its side mount",
    )
    ctx.expect_contact(
        aerator,
        body,
        elem_a="aerator_collar",
        elem_b="spout_shell",
        name="aerator collar seats against spout tip",
    )
    ctx.expect_overlap(
        aerator,
        body,
        axes="yz",
        elem_a="aerator_shell",
        elem_b="spout_shell",
        min_overlap=0.020,
        name="aerator remains coaxial with spout",
    )

    with ctx.pose({handle_joint: 0.60, aerator_joint: 1.20}):
        ctx.expect_gap(
            handle,
            body,
            axis="y",
            positive_elem="handle_blade",
            negative_elem="column_shell",
            min_gap=0.020,
            name="opened blade handle clears the faucet column",
        )
        ctx.expect_contact(
            aerator,
            body,
            elem_a="aerator_collar",
            elem_b="spout_shell",
            name="aerator stays seated while rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
