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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_box_mesh(
    *,
    width: float,
    depth: float,
    height: float,
    radius: float,
    name: str,
):
    geometry = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        depth,
        cap=True,
        center=True,
        closed=True,
    )
    geometry.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnetic_base_worksite_floodlight")

    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    pivot_black = model.material("pivot_black", rgba=(0.12, 0.12, 0.13, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.78, 0.12, 1.0))
    lens_smoke = model.material("lens_smoke", rgba=(0.70, 0.76, 0.80, 0.55))
    led_phosphor = model.material("led_phosphor", rgba=(0.98, 0.95, 0.72, 1.0))

    base_plate_geometry = ExtrudeGeometry(
        rounded_rect_profile(0.190, 0.115, 0.014, corner_segments=8),
        0.009,
        cap=True,
        center=True,
        closed=True,
    )
    base_plate_mesh = mesh_from_geometry(base_plate_geometry, "base_plate_top")
    head_housing_mesh = _rounded_box_mesh(
        width=0.150,
        depth=0.042,
        height=0.094,
        radius=0.010,
        name="head_housing",
    )
    bezel_mesh = _rounded_box_mesh(
        width=0.148,
        depth=0.006,
        height=0.092,
        radius=0.008,
        name="head_bezel",
    )

    base_support = model.part("base_support")
    base_support.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=dark_steel,
        name="base_plate_top",
    )
    base_support.visual(
        Box((0.176, 0.101, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=satin_black,
        name="magnet_pad",
    )
    base_support.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="post_collar",
    )
    base_support.visual(
        Cylinder(radius=0.011, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
        material=dark_steel,
        name="vertical_post",
    )
    base_support.visual(
        Box((0.170, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.003, 0.086)),
        material=pivot_black,
        name="yoke_bridge",
    )
    base_support.visual(
        Box((0.006, 0.020, 0.024)),
        origin=Origin(xyz=(-0.088, 0.008, 0.104)),
        material=pivot_black,
        name="left_yoke_ear",
    )
    base_support.visual(
        Box((0.006, 0.020, 0.024)),
        origin=Origin(xyz=(0.088, 0.008, 0.104)),
        material=pivot_black,
        name="right_yoke_ear",
    )
    base_support.inertial = Inertial.from_geometry(
        Box((0.190, 0.115, 0.116)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
    )

    head_assembly = model.part("head_assembly")
    head_assembly.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(-0.081, 0.008, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pivot_black,
        name="left_trunnion_boss",
    )
    head_assembly.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.081, 0.008, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pivot_black,
        name="right_trunnion_boss",
    )
    head_assembly.visual(
        Box((0.164, 0.008, 0.090)),
        origin=Origin(xyz=(0.0, 0.015, -0.014)),
        material=dark_steel,
        name="rear_mount_plate",
    )
    head_assembly.visual(
        head_housing_mesh,
        origin=Origin(xyz=(0.0, 0.034, -0.014)),
        material=safety_yellow,
        name="head_housing",
    )
    head_assembly.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, 0.058, -0.014)),
        material=satin_black,
        name="front_bezel",
    )
    head_assembly.visual(
        Box((0.136, 0.002, 0.082)),
        origin=Origin(xyz=(0.0, 0.055, -0.014)),
        material=lens_smoke,
        name="front_lens",
    )
    head_assembly.visual(
        Box((0.116, 0.010, 0.062)),
        origin=Origin(xyz=(0.0, 0.049, -0.014)),
        material=led_phosphor,
        name="led_panel",
    )
    for index, x_pos in enumerate((-0.054, -0.018, 0.018, 0.054)):
        head_assembly.visual(
            Box((0.008, 0.010, 0.078)),
            origin=Origin(xyz=(x_pos, 0.016, -0.014)),
            material=dark_steel,
            name=f"heat_sink_fin_{index}",
        )
    head_assembly.visual(
        Box((0.144, 0.004, 0.084)),
        origin=Origin(xyz=(0.0, 0.013, -0.014)),
        material=dark_steel,
        name="rear_heat_sink_plate",
    )
    head_assembly.inertial = Inertial.from_geometry(
        Box((0.166, 0.070, 0.094)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.034, -0.014)),
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=base_support,
        child=head_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-0.85,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_support = object_model.get_part("base_support")
    head_assembly = object_model.get_part("head_assembly")
    head_tilt = object_model.get_articulation("head_tilt")

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

    limits = head_tilt.motion_limits
    ctx.check(
        "tilt_axis_is_horizontal",
        tuple(round(value, 6) for value in head_tilt.axis) == (1.0, 0.0, 0.0),
        details=f"axis={head_tilt.axis}",
    )
    ctx.check(
        "tilt_limits_match_worklight_range",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -0.75
        and limits.upper >= 1.0,
        details=f"limits={limits}",
    )
    ctx.expect_origin_gap(
        head_assembly,
        base_support,
        axis="z",
        min_gap=0.100,
        max_gap=0.110,
        name="pivot_sits_on_short_post",
    )
    ctx.expect_contact(
        base_support,
        head_assembly,
        elem_a="left_yoke_ear",
        elem_b="left_trunnion_boss",
        name="left_yoke_supports_left_trunnion",
    )
    ctx.expect_contact(
        base_support,
        head_assembly,
        elem_a="right_yoke_ear",
        elem_b="right_trunnion_boss",
        name="right_yoke_supports_right_trunnion",
    )
    ctx.expect_gap(
        head_assembly,
        base_support,
        axis="z",
        min_gap=0.010,
        positive_elem="head_housing",
        negative_elem="base_plate_top",
        name="head_clears_base_at_rest",
    )
    ctx.expect_overlap(
        head_assembly,
        base_support,
        axes="x",
        min_overlap=0.120,
        elem_a="head_housing",
        elem_b="base_plate_top",
        name="head_tracks_over_wide_base",
    )

    with ctx.pose({head_tilt: -0.75}):
        ctx.expect_gap(
            head_assembly,
            base_support,
            axis="z",
            min_gap=0.008,
            positive_elem="head_housing",
            negative_elem="base_plate_top",
            name="head_clears_base_when_tilted_down",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
