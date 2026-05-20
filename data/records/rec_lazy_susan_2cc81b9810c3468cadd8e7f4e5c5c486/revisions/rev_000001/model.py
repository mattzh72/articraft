from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lazy_susan_serving_tray")

    oak = model.material("oak", rgba=(0.73, 0.58, 0.39, 1.0))
    walnut = model.material("walnut", rgba=(0.34, 0.22, 0.14, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.18, 0.20, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.71, 0.73, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    tray_outer_profile = [
        (0.0, 0.006),
        (0.145, 0.006),
        (0.180, 0.0065),
        (0.196, 0.0085),
        (0.206, 0.014),
        (0.210, 0.022),
        (0.210, 0.030),
    ]
    tray_inner_profile = [
        (0.0, 0.014),
        (0.150, 0.014),
        (0.182, 0.0145),
        (0.196, 0.017),
        (0.202, 0.023),
    ]
    tray_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            tray_outer_profile,
            tray_inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "upper_tray_shell",
    )
    base_disc_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.004),
                (0.108, 0.004),
                (0.122, 0.0045),
                (0.148, 0.0075),
                (0.166, 0.0125),
                (0.170, 0.0175),
                (0.164, 0.0210),
                (0.148, 0.0230),
                (0.0, 0.0230),
            ],
            segments=72,
        ),
        "base_disc_shell",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.112, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber,
        name="base_foot",
    )
    base.visual(
        base_disc_mesh,
        material=walnut,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=0.062, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=dark_metal,
        name="lower_bearing_race",
    )
    base.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=satin_steel,
        name="bearing_spindle",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.170, length=0.032),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    tray = model.part("tray")
    tray.visual(
        tray_shell_mesh,
        material=oak,
        name="tray_shell",
    )
    tray.visual(
        Cylinder(radius=0.057, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_metal,
        name="upper_bearing_race",
    )
    tray.visual(
        Cylinder(radius=0.034, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=satin_steel,
        name="hub_post",
    )
    tray.inertial = Inertial.from_geometry(
        Cylinder(radius=0.210, length=0.030),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    model.articulation(
        "tray_rotation",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tray = object_model.get_part("tray")
    rotation = object_model.get_articulation("tray_rotation")

    base_disc = base.get_visual("base_disc")
    spindle = base.get_visual("bearing_spindle")
    tray_shell = tray.get_visual("tray_shell")
    hub_post = tray.get_visual("hub_post")

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
        "tray_rotation_is_continuous",
        rotation.articulation_type == ArticulationType.CONTINUOUS,
        details=f"expected continuous rotation, got {rotation.articulation_type}",
    )
    ctx.check(
        "tray_rotation_axis_vertical",
        rotation.axis == (0.0, 0.0, 1.0),
        details=f"expected vertical axis, got {rotation.axis}",
    )
    limits = rotation.motion_limits
    ctx.check(
        "tray_rotation_limits_unbounded",
        limits is not None and limits.lower is None and limits.upper is None,
        details="continuous tray rotation should not define lower/upper bounds",
    )

    ctx.expect_origin_distance(
        tray,
        base,
        axes="xy",
        max_dist=0.001,
        name="tray_concentric_with_base",
    )
    ctx.expect_contact(
        tray,
        base,
        elem_a=hub_post,
        elem_b=spindle,
        name="hub_supported_by_spindle",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        min_gap=0.010,
        max_gap=0.016,
        positive_elem=tray_shell,
        negative_elem=base_disc,
        name="visible_shadow_gap_between_discs",
    )
    ctx.expect_overlap(
        tray,
        base,
        axes="xy",
        min_overlap=0.30,
        elem_a=tray_shell,
        elem_b=base_disc,
        name="tray_and_base_plan_overlap",
    )

    for angle, label in (
        (0.0, "rest"),
        (math.pi / 2.0, "quarter_turn"),
        (math.pi, "half_turn"),
    ):
        with ctx.pose({rotation: angle}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_no_floating")
            ctx.expect_contact(
                tray,
                base,
                elem_a=hub_post,
                elem_b=spindle,
                name=f"{label}_hub_contact",
            )
            ctx.expect_gap(
                tray,
                base,
                axis="z",
                min_gap=0.010,
                max_gap=0.016,
                positive_elem=tray_shell,
                negative_elem=base_disc,
                name=f"{label}_shadow_gap",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
