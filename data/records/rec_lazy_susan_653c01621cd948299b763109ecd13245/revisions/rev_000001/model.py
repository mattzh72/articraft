from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="round_dining_lazy_susan")

    tray_wood = model.material("tray_wood", rgba=(0.55, 0.38, 0.24, 1.0))
    base_wood = model.material("base_wood", rgba=(0.31, 0.21, 0.14, 1.0))
    bearing_metal = model.material("bearing_metal", rgba=(0.56, 0.58, 0.60, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    base_profile = [
        (0.0, 0.0),
        (0.110, 0.0),
        (0.164, 0.001),
        (0.202, 0.004),
        (0.215, 0.009),
        (0.208, 0.015),
        (0.182, 0.018),
        (0.0, 0.018),
    ]
    base_plinth_mesh = save_mesh("lazy_susan_base_plinth", LatheGeometry(base_profile, segments=72))

    tray_outer_profile = [
        (0.0, 0.0),
        (0.080, 0.0),
        (0.180, 0.001),
        (0.252, 0.003),
        (0.284, 0.006),
        (0.295, 0.014),
        (0.301, 0.028),
        (0.304, 0.030),
    ]
    tray_inner_profile = [
        (0.0, 0.011),
        (0.105, 0.011),
        (0.225, 0.012),
        (0.279, 0.014),
        (0.290, 0.018),
        (0.294, 0.026),
    ]
    tray_shell_mesh = save_mesh(
        "lazy_susan_tray_shell",
        LatheGeometry.from_shell_profiles(
            tray_outer_profile,
            tray_inner_profile,
            segments=88,
        ),
    )

    retainer_outer_profile = [
        (0.108, -0.016),
        (0.120, -0.016),
        (0.120, -0.002),
        (0.108, -0.002),
    ]
    retainer_inner_profile = [
        (0.111, -0.0145),
        (0.116, -0.0145),
        (0.116, -0.0035),
        (0.111, -0.0035),
    ]
    retainer_mesh = save_mesh(
        "lazy_susan_retainer_skirt",
        LatheGeometry.from_shell_profiles(
            retainer_outer_profile,
            retainer_inner_profile,
            segments=72,
        ),
    )

    base_ring = model.part("base_ring")
    base_ring.visual(base_plinth_mesh, material=base_wood, name="base_plinth")
    base_ring.visual(
        Cylinder(radius=0.188, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=base_wood,
        name="base_trim",
    )
    base_ring.visual(
        Cylinder(radius=0.103, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=bearing_metal,
        name="bearing_pedestal",
    )
    base_ring.visual(
        Cylinder(radius=0.070, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0015)),
        material=matte_black,
        name="anti_skid_pad",
    )
    base_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.215, length=0.032),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    rotating_tray = model.part("rotating_tray")
    rotating_tray.visual(tray_shell_mesh, material=tray_wood, name="tray_shell")
    rotating_tray.visual(
        Cylinder(radius=0.110, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=bearing_metal,
        name="bearing_pad",
    )
    rotating_tray.visual(retainer_mesh, material=matte_black, name="retainer_skirt")
    rotating_tray.inertial = Inertial.from_geometry(
        Cylinder(radius=0.304, length=0.046),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "tray_spin",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=rotating_tray,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    base_ring = object_model.get_part("base_ring")
    rotating_tray = object_model.get_part("rotating_tray")
    tray_spin = object_model.get_articulation("tray_spin")

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
        "tray_spin_is_continuous",
        tray_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"articulation_type={tray_spin.articulation_type}",
    )
    ctx.check(
        "tray_spin_axis_vertical",
        tuple(tray_spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={tray_spin.axis}",
    )

    ctx.expect_contact(
        rotating_tray,
        base_ring,
        elem_a="bearing_pad",
        elem_b="bearing_pedestal",
        name="tray_bearing_pad_seats_on_pedestal",
    )
    ctx.expect_gap(
        rotating_tray,
        base_ring,
        axis="z",
        positive_elem="tray_shell",
        negative_elem="base_trim",
        min_gap=0.014,
        max_gap=0.018,
        name="tray_shell_clears_visible_base_ring",
    )
    ctx.expect_within(
        base_ring,
        rotating_tray,
        axes="xy",
        name="base_ring_stays_within_tray_footprint",
    )
    ctx.expect_overlap(
        rotating_tray,
        base_ring,
        axes="xy",
        min_overlap=0.40,
        name="tray_and_base_share_substantial_centered_overlap",
    )

    with ctx.pose({tray_spin: 2.35}):
        ctx.expect_contact(
            rotating_tray,
            base_ring,
            elem_a="bearing_pad",
            elem_b="bearing_pedestal",
            name="tray_remains_seated_when_rotated",
        )
        ctx.expect_gap(
            rotating_tray,
            base_ring,
            axis="z",
            positive_elem="tray_shell",
            negative_elem="base_trim",
            min_gap=0.014,
            max_gap=0.018,
            name="tray_shell_keeps_same_standoff_after_rotation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
