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
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _annulus_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    name: str,
    segments: int = 48,
):
    outer = _circle_profile(outer_radius, segments=segments)
    inner = _circle_profile(inner_radius, segments=segments)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, [inner], height, center=True),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_calibration_syringe")

    glass = model.material("glass", rgba=(0.82, 0.90, 0.98, 0.30))
    stainless = model.material("stainless", rgba=(0.75, 0.77, 0.80, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.14, 0.15, 0.16, 1.0))
    ink = model.material("ink", rgba=(0.08, 0.09, 0.10, 1.0))
    accent = model.material("accent", rgba=(0.26, 0.36, 0.58, 1.0))

    outer_radius = 0.009
    inner_radius = 0.007
    tube_z0 = 0.026
    tube_z1 = 0.142
    guide_z0 = tube_z1
    guide_z1 = 0.154
    slide_upper = 0.081

    barrel = model.part("barrel")
    barrel.inertial = Inertial.from_geometry(
        Box((0.060, 0.050, guide_z1)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, guide_z1 * 0.5)),
    )

    barrel.visual(
        _annulus_mesh(
            outer_radius=outer_radius,
            inner_radius=inner_radius,
            height=tube_z1 - tube_z0,
            name="barrel_tube_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, (tube_z0 + tube_z1) * 0.5)),
        material=glass,
        name="barrel_tube",
    )
    barrel.visual(
        _annulus_mesh(
            outer_radius=0.012,
            inner_radius=0.0042,
            height=guide_z1 - guide_z0,
            name="rear_guide_bushing",
        ),
        origin=Origin(xyz=(0.0, 0.0, (guide_z0 + guide_z1) * 0.5)),
        material=stainless,
        name="guide_bushing",
    )
    barrel.visual(
        _annulus_mesh(
            outer_radius=0.0074,
            inner_radius=0.0032,
            height=0.004,
            name="dose_shoulder_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=stainless,
        name="dose_shoulder",
    )
    barrel.visual(
        _annulus_mesh(
            outer_radius=0.011,
            inner_radius=0.0034,
            height=0.016,
            name="hub_body_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=stainless,
        name="hub_body",
    )
    barrel.visual(
        Box((0.006, 0.012, 0.010)),
        origin=Origin(xyz=(0.008, 0.0, 0.018)),
        material=stainless,
        name="datum_flat_right",
    )
    barrel.visual(
        Box((0.006, 0.012, 0.010)),
        origin=Origin(xyz=(-0.008, 0.0, 0.018)),
        material=stainless,
        name="datum_flat_left",
    )
    barrel.visual(
        Cylinder(radius=0.0034, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=stainless,
        name="tip_base",
    )
    barrel.visual(
        Cylinder(radius=0.0022, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=stainless,
        name="nozzle_tip",
    )
    barrel.visual(
        Box((0.012, 0.020, 0.003)),
        origin=Origin(xyz=(0.0, 0.018, 0.145)),
        material=stainless,
        name="left_finger_flange",
    )
    barrel.visual(
        Box((0.012, 0.020, 0.003)),
        origin=Origin(xyz=(0.0, -0.018, 0.145)),
        material=stainless,
        name="right_finger_flange",
    )
    barrel.visual(
        Box((0.0010, 0.0025, 0.088)),
        origin=Origin(xyz=(0.0086, 0.0, 0.086)),
        material=ink,
        name="index_rail",
    )

    z_marks = [0.040 + 0.007 * i for i in range(13)]
    for i, z in enumerate(z_marks):
        major = i % 5 == 0
        half_major = i % 5 == 2
        mark_width_y = 0.0062 if major else 0.0046 if half_major else 0.0033
        mark_height_z = 0.0010 if major else 0.0008
        barrel.visual(
            Box((0.0010, mark_width_y, mark_height_z)),
            origin=Origin(xyz=(0.0086, 0.0, z)),
            material=ink,
            name=f"mark_{i:02d}",
        )

    plunger = model.part("plunger")
    plunger.inertial = Inertial.from_geometry(
        Box((0.032, 0.018, 0.190)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
    )
    plunger.visual(
        Cylinder(radius=0.0026, length=0.124),
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
        material=stainless,
        name="rod",
    )
    plunger.visual(
        Cylinder(radius=0.00675, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=dark_polymer,
        name="seal",
    )
    plunger.visual(
        Cylinder(radius=0.0057, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.083)),
        material=stainless,
        name="stop_collar",
    )
    plunger.visual(
        Cylinder(radius=0.0060, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=accent,
        name="adjuster",
    )
    plunger.visual(
        Box((0.004, 0.007, 0.006)),
        origin=Origin(xyz=(0.0062, 0.0, 0.012)),
        material=dark_polymer,
        name="adjust_index",
    )
    plunger.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=stainless,
        name="thumb_stem",
    )
    plunger.visual(
        Box((0.030, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=dark_polymer,
        name="thumb_pad",
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, guide_z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=0.25,
            lower=0.0,
            upper=slide_upper,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("plunger_slide")

    guide_bushing = barrel.get_visual("guide_bushing")
    dose_shoulder = barrel.get_visual("dose_shoulder")
    thumb_pad = plunger.get_visual("thumb_pad")
    stop_collar = plunger.get_visual("stop_collar")
    seal = plunger.get_visual("seal")
    adjuster = plunger.get_visual("adjuster")

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

    limits = slide.motion_limits
    axis_ok = tuple(float(v) for v in slide.axis) == (0.0, 0.0, 1.0)
    limits_ok = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower - 0.0) < 1e-9
        and abs(limits.upper - 0.081) < 1e-9
    )
    ctx.check(
        "plunger_axis_is_coaxial_linear_z",
        axis_ok,
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "plunger_limits_match_stop_geometry",
        limits_ok,
        details=f"limits={limits}",
    )
    ctx.expect_origin_distance(
        plunger,
        barrel,
        axes="xy",
        max_dist=1e-6,
        name="plunger_stays_coaxial_to_barrel",
    )

    closed_position = ctx.part_world_position(plunger)
    with ctx.pose({slide: 0.081}):
        open_position = ctx.part_world_position(plunger)
        ctx.expect_origin_distance(
            plunger,
            barrel,
            axes="xy",
            max_dist=1e-6,
            name="plunger_remains_coaxial_when_retracted",
        )
        ctx.expect_gap(
            barrel,
            plunger,
            axis="z",
            positive_elem=guide_bushing,
            negative_elem=stop_collar,
            max_gap=0.001,
            max_penetration=0.0,
            name="rear_stop_collar_seats_under_guide_bushing",
        )
        ctx.expect_contact(
            plunger,
            barrel,
            elem_a=stop_collar,
            elem_b=guide_bushing,
            contact_tol=0.001,
            name="rear_stop_geometry_is_real_contact",
        )
        ctx.expect_gap(
            plunger,
            barrel,
            axis="z",
            positive_elem=thumb_pad,
            negative_elem=guide_bushing,
            min_gap=0.070,
            max_gap=0.115,
            name="thumb_pad_clears_barrel_at_full_retraction",
        )

    outward_ok = (
        closed_position is not None
        and open_position is not None
        and open_position[2] > closed_position[2] + 0.070
    )
    ctx.check(
        "plunger_motion_retracts_outward",
        outward_ok,
        details=f"closed={closed_position}, open={open_position}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            plunger,
            barrel,
            elem_a=seal,
            elem_b=dose_shoulder,
            contact_tol=1e-5,
            name="seal_stops_at_front_dose_shoulder",
        )
        ctx.expect_gap(
            plunger,
            barrel,
            axis="z",
            positive_elem=adjuster,
            negative_elem=guide_bushing,
            min_gap=0.001,
            max_gap=0.018,
            name="adjuster_sits_just_above_guide_in_closed_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
