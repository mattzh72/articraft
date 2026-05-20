from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_slab_mesh(
    width: float,
    depth: float,
    height: float,
    corner_radius: float,
    z_center: float,
    name: str,
):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, depth, corner_radius),
        height,
        center=True,
        cap=True,
        closed=True,
    )
    geom.translate(0.0, 0.0, z_center)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_turntable")

    plinth_paint = model.material("plinth_paint", rgba=(0.17, 0.18, 0.19, 1.0))
    plinth_polymer = model.material("plinth_polymer", rgba=(0.10, 0.11, 0.12, 1.0))
    platter_metal = model.material("platter_metal", rgba=(0.30, 0.31, 0.33, 1.0))
    tonearm_metal = model.material("tonearm_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    elastomer = model.material("elastomer", rgba=(0.05, 0.05, 0.05, 1.0))
    spindle_metal = model.material("spindle_metal", rgba=(0.84, 0.85, 0.87, 1.0))

    plinth = model.part("plinth")
    plinth.visual(
        _rounded_slab_mesh(
            width=0.458,
            depth=0.358,
            height=0.032,
            corner_radius=0.020,
            z_center=0.022,
            name="plinth_lower_shell",
        ),
        material=plinth_polymer,
        name="lower_shell",
    )
    plinth.visual(
        _rounded_slab_mesh(
            width=0.442,
            depth=0.342,
            height=0.018,
            corner_radius=0.015,
            z_center=0.047,
            name="plinth_upper_shell",
        ),
        material=plinth_paint,
        name="upper_shell",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.188, -0.138),
            (-0.188, 0.138),
            (0.188, -0.138),
            (0.188, 0.138),
        ),
        start=1,
    ):
        plinth.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, 0.003)),
            material=elastomer,
            name=f"foot_{index}",
        )

    platter_axis = (-0.040, 0.000, 0.062)
    tonearm_axis = (0.148, -0.095, 0.088)

    plinth.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(platter_axis[0], platter_axis[1], 0.059)),
        material=plinth_paint,
        name="bearing_base",
    )
    plinth.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(platter_axis[0], platter_axis[1], 0.059)),
        material=spindle_metal,
        name="bearing_cap",
    )
    plinth.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(tonearm_axis[0], tonearm_axis[1], 0.065)),
        material=plinth_paint,
        name="arm_pivot_base",
    )
    plinth.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(tonearm_axis[0], tonearm_axis[1], 0.081)),
        material=spindle_metal,
        name="arm_pivot_cap",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=platter_metal,
        name="hub_boss",
    )
    platter.visual(
        Cylinder(radius=0.155, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=platter_metal,
        name="platter_skirt",
    )
    platter.visual(
        Cylinder(radius=0.149, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=platter_metal,
        name="platter_crown",
    )
    platter.visual(
        Cylinder(radius=0.146, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=elastomer,
        name="record_mat",
    )
    platter.visual(
        Cylinder(radius=0.003, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=spindle_metal,
        name="spindle_pin",
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=tonearm_metal,
        name="pivot_collar",
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.216),
        origin=Origin(xyz=(0.108, 0.0, 0.012), rpy=(0.0, pi / 2.0, 0.0)),
        material=tonearm_metal,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.0035, length=0.048),
        origin=Origin(xyz=(-0.024, 0.0, 0.011), rpy=(0.0, pi / 2.0, 0.0)),
        material=tonearm_metal,
        name="counter_stem",
    )
    tonearm.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(-0.046, 0.0, 0.011), rpy=(0.0, pi / 2.0, 0.0)),
        material=platter_metal,
        name="counterweight",
    )
    tonearm.visual(
        _rounded_slab_mesh(
            width=0.020,
            depth=0.018,
            height=0.012,
            corner_radius=0.003,
            z_center=0.010,
            name="tonearm_pivot_block",
        ),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=tonearm_metal,
        name="pivot_block",
    )
    tonearm.visual(
        _rounded_slab_mesh(
            width=0.028,
            depth=0.018,
            height=0.005,
            corner_radius=0.002,
            z_center=0.009,
            name="tonearm_headshell",
        ),
        origin=Origin(xyz=(0.228, 0.0, 0.0)),
        material=tonearm_metal,
        name="headshell",
    )
    tonearm.visual(
        _rounded_slab_mesh(
            width=0.014,
            depth=0.012,
            height=0.006,
            corner_radius=0.0015,
            z_center=0.0045,
            name="tonearm_cartridge",
        ),
        origin=Origin(xyz=(0.229, 0.0, 0.0)),
        material=plinth_polymer,
        name="cartridge",
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=platter_axis),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=tonearm_axis, rpy=(0.0, 0.0, 1.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=1.5,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_swing = object_model.get_articulation("tonearm_swing")

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

    platter_limits = platter_spin.motion_limits
    tonearm_limits = tonearm_swing.motion_limits
    ctx.check(
        "platter_joint_is_continuous_and_unbounded",
        platter_spin.articulation_type == ArticulationType.CONTINUOUS
        and platter_limits is not None
        and platter_limits.lower is None
        and platter_limits.upper is None,
        details="The platter should use an unbounded continuous rotation joint.",
    )
    ctx.check(
        "tonearm_joint_has_play_arc",
        tonearm_swing.articulation_type == ArticulationType.REVOLUTE
        and tonearm_limits is not None
        and tonearm_limits.lower == 0.0
        and tonearm_limits.upper is not None
        and 0.9 <= tonearm_limits.upper <= 1.2,
        details="The tonearm should sweep inward over a realistic single-axis arc.",
    )

    ctx.expect_contact(
        platter,
        plinth,
        elem_a="hub_boss",
        elem_b="bearing_cap",
        name="platter_hub_is_supported",
    )
    ctx.expect_within(
        plinth,
        platter,
        axes="xy",
        inner_elem="bearing_cap",
        outer_elem="hub_boss",
        margin=0.0,
        name="bearing_cap_stays_within_hub",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_skirt",
        negative_elem="upper_shell",
        min_gap=0.008,
        max_gap=0.012,
        name="platter_clears_plinth_top",
    )

    ctx.expect_contact(
        tonearm,
        plinth,
        elem_a="pivot_collar",
        elem_b="arm_pivot_cap",
        name="tonearm_pivot_is_supported",
    )
    ctx.expect_within(
        plinth,
        tonearm,
        axes="xy",
        inner_elem="arm_pivot_cap",
        outer_elem="pivot_collar",
        margin=0.0,
        name="pivot_cap_stays_within_collar",
    )
    ctx.expect_gap(
        tonearm,
        plinth,
        axis="z",
        positive_elem="arm_tube",
        negative_elem="upper_shell",
        min_gap=0.030,
        max_gap=0.050,
        name="tonearm_tube_clears_plinth",
    )

    with ctx.pose({tonearm_swing: 0.95}):
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="headshell",
            elem_b="record_mat",
            min_overlap=0.008,
            name="headshell_reaches_record_area",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="cartridge",
            negative_elem="record_mat",
            min_gap=0.002,
            max_gap=0.008,
            name="cartridge_clears_record_surface",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
