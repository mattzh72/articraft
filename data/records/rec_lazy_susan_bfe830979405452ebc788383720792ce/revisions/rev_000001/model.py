from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="monitor_stand_lazy_susan")

    top_finish = model.material("top_finish", rgba=(0.17, 0.18, 0.19, 1.0))
    base_finish = model.material("base_finish", rgba=(0.13, 0.13, 0.14, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.39, 0.40, 0.42, 1.0))

    top_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.56, 0.26, 0.020, corner_segments=10),
            0.030,
        ),
        "top_platform_plate",
    )

    base_assembly = model.part("base_assembly")
    base_assembly.visual(
        Cylinder(radius=0.17, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=base_finish,
        name="base_disc",
    )
    base_assembly.visual(
        Cylinder(radius=0.13, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=bearing_steel,
        name="bearing_ring",
    )
    base_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.17, length=0.024),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    top_platform = model.part("top_platform")
    top_platform.visual(
        Cylinder(radius=0.095, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=bearing_steel,
        name="rotary_carrier",
    )
    top_platform.visual(
        Box((0.34, 0.16, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=top_finish,
        name="underside_stiffener",
    )
    top_platform.visual(
        top_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=top_finish,
        name="top_plate",
    )
    top_platform.inertial = Inertial.from_geometry(
        Box((0.56, 0.26, 0.050)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    model.articulation(
        "turntable_spin",
        ArticulationType.CONTINUOUS,
        parent=base_assembly,
        child=top_platform,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_assembly = object_model.get_part("base_assembly")
    top_platform = object_model.get_part("top_platform")
    turntable_spin = object_model.get_articulation("turntable_spin")

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
        "turntable articulation is continuous",
        turntable_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"expected continuous articulation, got {turntable_spin.articulation_type}",
    )
    ctx.check(
        "turntable axis is vertical",
        tuple(turntable_spin.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical axis, got {turntable_spin.axis}",
    )

    ctx.expect_contact(
        top_platform,
        base_assembly,
        elem_a="rotary_carrier",
        elem_b="bearing_ring",
        name="carrier contacts bearing ring",
    )
    ctx.expect_origin_distance(
        top_platform,
        base_assembly,
        axes="xy",
        max_dist=0.001,
        name="platform stays centered over base",
    )
    ctx.expect_gap(
        top_platform,
        base_assembly,
        axis="z",
        min_gap=0.028,
        max_gap=0.040,
        positive_elem="top_plate",
        negative_elem="base_disc",
        name="top plate sits above base disc",
    )
    ctx.expect_overlap(
        top_platform,
        base_assembly,
        axes="xy",
        min_overlap=0.18,
        elem_a="rotary_carrier",
        elem_b="bearing_ring",
        name="bearing faces share broad footprint",
    )

    with ctx.pose({turntable_spin: 1.1}):
        ctx.expect_contact(
            top_platform,
            base_assembly,
            elem_a="rotary_carrier",
            elem_b="bearing_ring",
            name="carrier stays seated while rotated",
        )
        ctx.expect_origin_distance(
            top_platform,
            base_assembly,
            axes="xy",
            max_dist=0.001,
            name="platform remains centered while rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
