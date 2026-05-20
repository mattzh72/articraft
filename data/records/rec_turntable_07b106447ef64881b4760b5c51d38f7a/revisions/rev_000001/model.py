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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _ring_prism_mesh(
    name: str,
    *,
    outer_size: tuple[float, float],
    inner_size: tuple[float, float],
    height: float,
    outer_radius: float,
    inner_radius: float,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(
                outer_size[0],
                outer_size[1],
                outer_radius,
                corner_segments=8,
            ),
            [
                rounded_rect_profile(
                    inner_size[0],
                    inner_size[1],
                    inner_radius,
                    corner_segments=8,
                )
            ],
            height=height,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_turntable")

    plinth_paint = model.material("plinth_paint", rgba=(0.22, 0.23, 0.24, 1.0))
    deck_black = model.material("deck_black", rgba=(0.10, 0.11, 0.12, 1.0))
    machined_alloy = model.material("machined_alloy", rgba=(0.75, 0.77, 0.79, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    cartridge_accent = model.material("cartridge_accent", rgba=(0.74, 0.36, 0.12, 1.0))

    top_deck_mesh = _ring_prism_mesh(
        "turntable_top_deck",
        outer_size=(0.48, 0.36),
        inner_size=(0.27, 0.18),
        height=0.012,
        outer_radius=0.024,
        inner_radius=0.016,
    )
    main_bearing_mesh = _ring_prism_mesh(
        "turntable_main_bearing",
        outer_size=(0.086, 0.086),
        inner_size=(0.034, 0.034),
        height=0.124,
        outer_radius=0.008,
        inner_radius=0.005,
    )
    tonearm_pedestal_mesh = _ring_prism_mesh(
        "turntable_tonearm_pedestal",
        outer_size=(0.060, 0.060),
        inner_size=(0.020, 0.020),
        height=0.044,
        outer_radius=0.006,
        inner_radius=0.003,
    )

    plinth = model.part("plinth")
    plinth.visual(
        Box((0.46, 0.34, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=plinth_paint,
        name="bottom_plate",
    )
    plinth.visual(
        top_deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material=deck_black,
        name="top_deck",
    )
    plinth.visual(
        Box((0.03, 0.30, 0.108)),
        origin=Origin(xyz=(-0.215, 0.0, 0.072)),
        material=plinth_paint,
        name="left_sidewall",
    )
    plinth.visual(
        Box((0.03, 0.30, 0.108)),
        origin=Origin(xyz=(0.215, 0.0, 0.072)),
        material=plinth_paint,
        name="right_sidewall",
    )
    plinth.visual(
        Box((0.43, 0.03, 0.108)),
        origin=Origin(xyz=(0.0, 0.155, 0.072)),
        material=plinth_paint,
        name="rear_panel",
    )
    plinth.visual(
        Box((0.43, 0.03, 0.038)),
        origin=Origin(xyz=(0.0, -0.155, 0.037)),
        material=plinth_paint,
        name="front_tie_bar",
    )
    plinth.visual(
        main_bearing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=dark_steel,
        name="main_bearing",
    )
    plinth.visual(
        Box((0.082, 0.082, 0.012)),
        origin=Origin(xyz=(0.170, 0.110, 0.144)),
        material=dark_steel,
        name="tonearm_pad",
    )
    plinth.visual(
        tonearm_pedestal_mesh,
        origin=Origin(xyz=(0.170, 0.110, 0.166)),
        material=dark_steel,
        name="tonearm_pedestal",
    )
    for foot_name, foot_x, foot_y in (
        ("front_left_foot", -0.175, -0.120),
        ("front_right_foot", 0.175, -0.120),
        ("rear_left_foot", -0.175, 0.120),
        ("rear_right_foot", 0.175, 0.120),
    ):
        plinth.visual(
            Cylinder(radius=0.018, length=0.016),
            origin=Origin(xyz=(foot_x, foot_y, -0.008)),
            material=rubber,
            name=foot_name,
        )
    plinth.inertial = Inertial.from_geometry(
        Box((0.48, 0.36, 0.160)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.034, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_steel,
        name="support_flange",
    )
    platter.visual(
        Cylinder(radius=0.040, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="hub_collar",
    )
    platter.visual(
        Cylinder(radius=0.145, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=machined_alloy,
        name="main_disc",
    )
    platter.visual(
        Cylinder(radius=0.149, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=machined_alloy,
        name="outer_rim",
    )
    platter.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=machined_alloy,
        name="spindle",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.149, length=0.038),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    slip_mat = model.part("slip_mat")
    slip_mat.visual(
        Cylinder(radius=0.133, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0355)),
        material=rubber,
        name="mat_disc",
    )
    slip_mat.inertial = Inertial.from_geometry(
        Cylinder(radius=0.133, length=0.003),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.0355)),
    )

    tonearm_stage = model.part("tonearm_stage")
    tonearm_stage.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_steel,
        name="pivot_flange",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=dark_steel,
        name="pivot_spindle",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="pivot_housing",
    )
    tonearm_stage.visual(
        Box((0.032, 0.024, 0.016)),
        origin=Origin(xyz=(0.018, 0.0, 0.014)),
        material=dark_steel,
        name="arm_socket",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.0075, length=0.150),
        origin=Origin(xyz=(0.085, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_alloy,
        name="arm_tube",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.0060, length=0.060),
        origin=Origin(xyz=(0.190, 0.0, 0.013), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_alloy,
        name="forearm_tube",
    )
    tonearm_stage.visual(
        Box((0.024, 0.018, 0.004)),
        origin=Origin(xyz=(0.226, 0.0, 0.009)),
        material=machined_alloy,
        name="headshell_plate",
    )
    tonearm_stage.visual(
        Box((0.016, 0.010, 0.010)),
        origin=Origin(xyz=(0.212, 0.0, 0.011)),
        material=dark_steel,
        name="headshell_brace",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.005, length=0.070),
        origin=Origin(xyz=(-0.032, 0.0, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="counterweight_shaft",
    )
    tonearm_stage.visual(
        Cylinder(radius=0.017, length=0.042),
        origin=Origin(xyz=(-0.082, 0.0, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="counterweight",
    )
    tonearm_stage.visual(
        Box((0.012, 0.020, 0.026)),
        origin=Origin(xyz=(0.010, 0.016, 0.015)),
        material=dark_steel,
        name="service_rest_post",
    )
    tonearm_stage.inertial = Inertial.from_geometry(
        Box((0.300, 0.060, 0.060)),
        mass=0.95,
        origin=Origin(xyz=(0.070, 0.0, 0.014)),
    )

    cartridge = model.part("cartridge")
    cartridge.visual(
        Box((0.018, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=cartridge_accent,
        name="mount_block",
    )
    cartridge.visual(
        Box((0.008, 0.010, 0.008)),
        origin=Origin(xyz=(0.011, 0.0, -0.011)),
        material=deck_black,
        name="stylus_nose",
    )
    cartridge.inertial = Inertial.from_geometry(
        Box((0.026, 0.014, 0.014)),
        mass=0.03,
        origin=Origin(xyz=(0.004, 0.0, -0.006)),
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.142)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=18.0),
    )
    model.articulation(
        "platter_to_slip_mat",
        ArticulationType.FIXED,
        parent=platter,
        child=slip_mat,
        origin=Origin(),
    )
    model.articulation(
        "plinth_to_tonearm_stage",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm_stage,
        origin=Origin(xyz=(0.170, 0.110, 0.188), rpy=(0.0, 0.0, -1.95)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=0.65,
        ),
    )
    model.articulation(
        "tonearm_to_cartridge",
        ArticulationType.FIXED,
        parent=tonearm_stage,
        child=cartridge,
        origin=Origin(xyz=(0.226, 0.0, 0.007)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    slip_mat = object_model.get_part("slip_mat")
    tonearm_stage = object_model.get_part("tonearm_stage")
    cartridge = object_model.get_part("cartridge")

    platter_spin = object_model.get_articulation("plinth_to_platter")
    tonearm_sweep = object_model.get_articulation("plinth_to_tonearm_stage")

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
        "platter_joint_is_continuous",
        platter_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"expected CONTINUOUS, got {platter_spin.articulation_type}",
    )
    ctx.check(
        "tonearm_joint_axis_is_vertical",
        tuple(tonearm_sweep.axis) == (0.0, 0.0, -1.0),
        details=f"unexpected tonearm axis {tonearm_sweep.axis}",
    )

    ctx.expect_contact(
        platter,
        plinth,
        elem_a="support_flange",
        elem_b="main_bearing",
        name="platter_is_seated_on_bearing",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="main_disc",
        negative_elem="top_deck",
        min_gap=0.010,
        max_gap=0.030,
        name="platter_clears_top_deck",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        min_overlap=0.20,
        elem_a="main_disc",
        elem_b="top_deck",
        name="platter_spans_plinth_opening",
    )
    ctx.expect_contact(
        slip_mat,
        platter,
        elem_a="mat_disc",
        elem_b="main_disc",
        name="slip_mat_is_mounted_to_platter",
    )
    ctx.expect_contact(
        tonearm_stage,
        plinth,
        elem_a="pivot_flange",
        elem_b="tonearm_pedestal",
        name="tonearm_stage_is_seated_on_pedestal",
    )
    ctx.expect_contact(
        cartridge,
        tonearm_stage,
        elem_a="mount_block",
        elem_b="headshell_plate",
        name="cartridge_is_bolted_to_headshell",
    )

    with ctx.pose({tonearm_sweep: 0.40}):
        ctx.expect_overlap(
            cartridge,
            slip_mat,
            axes="xy",
            elem_a="mount_block",
            elem_b="mat_disc",
            min_overlap=0.008,
            name="tonearm_reaches_record_surface",
        )
        ctx.expect_gap(
            cartridge,
            slip_mat,
            axis="z",
            positive_elem="mount_block",
            negative_elem="mat_disc",
            min_gap=0.001,
            max_gap=0.016,
            name="cartridge_flies_above_record_without_collision",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
