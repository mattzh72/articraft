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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _ring_mesh(*, outer_radius: float, inner_radius: float, height: float, name: str):
    outer = [(outer_radius, 0.0), (outer_radius, height)]
    inner = [(inner_radius, 0.0), (inner_radius, height)]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer,
            inner,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _plinth_mesh(name: str):
    geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(0.470, 0.365, 0.018),
        0.054,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def _tonearm_tube_mesh(name: str):
    geom = tube_from_spline_points(
        [
            (-0.048, 0.000, 0.026),
            (-0.086, -0.001, 0.027),
            (-0.140, -0.005, 0.028),
            (-0.188, -0.008, 0.027),
            (-0.208, -0.010, 0.026),
        ],
        radius=0.0052,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_calibration_turntable")

    plinth_black = model.material("plinth_black", rgba=(0.16, 0.17, 0.18, 1.0))
    datum_dark = model.material("datum_dark", rgba=(0.23, 0.24, 0.25, 1.0))
    metal = model.material("machined_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    arm_metal = model.material("arm_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    mat_black = model.material("mat_black", rgba=(0.11, 0.12, 0.13, 1.0))
    mark_light = model.material("mark_light", rgba=(0.86, 0.87, 0.88, 1.0))
    mark_red = model.material("mark_red", rgba=(0.80, 0.16, 0.14, 1.0))

    plinth = model.part("plinth")
    platter_center = (-0.055, 0.000)
    plinth.visual(_plinth_mesh("turntable_plinth"), material=plinth_black, name="plinth_body")
    plinth.visual(
        Box((0.140, 0.318, 0.006)),
        origin=Origin(xyz=(-0.149, 0.000, 0.057)),
        material=datum_dark,
        name="datum_deck",
    )
    plinth.visual(
        Box((0.250, 0.318, 0.006)),
        origin=Origin(xyz=(0.094, 0.000, 0.057)),
        material=datum_dark,
        name="datum_deck_right",
    )
    plinth.visual(
        Box((0.048, 0.138, 0.006)),
        origin=Origin(xyz=(platter_center[0], -0.090, 0.057)),
        material=datum_dark,
        name="spindle_front_land",
    )
    plinth.visual(
        Box((0.048, 0.138, 0.006)),
        origin=Origin(xyz=(platter_center[0], 0.090, 0.057)),
        material=datum_dark,
        name="spindle_rear_land",
    )
    plinth.visual(
        Box((0.360, 0.020, 0.004)),
        origin=Origin(xyz=(-0.010, -0.118, 0.062)),
        material=datum_dark,
        name="front_datum_rail",
    )
    plinth.visual(
        Box((0.116, 0.108, 0.004)),
        origin=Origin(xyz=(0.160, 0.092, 0.062)),
        material=datum_dark,
        name="armboard_pad",
    )
    plinth.visual(
        Box((0.126, 0.016, 0.004)),
        origin=Origin(xyz=(-0.164, 0.122, 0.062)),
        material=datum_dark,
        name="lateral_datum_strip",
    )

    plinth.visual(
        _ring_mesh(
            outer_radius=0.018,
            inner_radius=0.0052,
            height=0.014,
            name="platter_bearing_ring",
        ),
        origin=Origin(xyz=(platter_center[0], platter_center[1], 0.046)),
        material=metal,
        name="bearing_sleeve",
    )
    plinth.visual(
        Cylinder(radius=0.0036, length=0.002),
        origin=Origin(xyz=(platter_center[0], platter_center[1], 0.050)),
        material=metal,
        name="thrust_pad",
    )
    plinth.visual(
        Cylinder(radius=0.0056, length=0.003),
        origin=Origin(xyz=(platter_center[0], platter_center[1], 0.0485)),
        material=metal,
        name="bearing_floor",
    )

    tonearm_pivot_xy = (0.168, 0.092)
    plinth.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(tonearm_pivot_xy[0], tonearm_pivot_xy[1], 0.067)),
        material=metal,
        name="tonearm_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(tonearm_pivot_xy[0], tonearm_pivot_xy[1], 0.084)),
        material=metal,
        name="tonearm_pivot_post",
    )
    plinth.visual(
        Cylinder(radius=0.004, length=0.036),
        origin=Origin(xyz=(0.118, 0.145, 0.078)),
        material=metal,
        name="arm_rest_post",
    )
    plinth.visual(
        Box((0.020, 0.006, 0.004)),
        origin=Origin(xyz=(0.109, 0.145, 0.096)),
        material=metal,
        name="arm_rest_cradle",
    )
    plinth.visual(
        Box((0.022, 0.0025, 0.0015)),
        origin=Origin(xyz=(0.168, 0.126, 0.06475)),
        material=mark_light,
        name="stage_zero_mark",
    )
    plinth.visual(
        Box((0.014, 0.0020, 0.0015)),
        origin=Origin(xyz=(0.154, 0.121, 0.06475), rpy=(0.0, 0.0, 0.42)),
        material=mark_light,
        name="stage_minus_mark",
    )
    plinth.visual(
        Box((0.014, 0.0020, 0.0015)),
        origin=Origin(xyz=(0.182, 0.121, 0.06475), rpy=(0.0, 0.0, -0.42)),
        material=mark_light,
        name="stage_plus_mark",
    )
    plinth.visual(
        Box((0.020, 0.0025, 0.0015)),
        origin=Origin(xyz=(-0.055, -0.118, 0.06475)),
        material=mark_red,
        name="platter_index_mark",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.470, 0.365, 0.064)),
        mass=12.0,
        origin=Origin(xyz=(0.000, 0.000, 0.032)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.152, length=0.028),
        origin=Origin(xyz=(0.000, 0.000, 0.018)),
        material=metal,
        name="platter_rim",
    )
    platter.visual(
        Cylinder(radius=0.138, length=0.003),
        origin=Origin(xyz=(0.000, 0.000, 0.0295)),
        material=mat_black,
        name="platter_mat",
    )
    platter.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=metal,
        name="hub_shoulder",
    )
    platter.visual(
        Cylinder(radius=0.0032, length=0.009),
        origin=Origin(xyz=(0.000, 0.000, -0.0045)),
        material=metal,
        name="spindle_shaft",
    )
    platter.visual(
        Cylinder(radius=0.0032, length=0.032),
        origin=Origin(xyz=(0.000, 0.000, 0.026)),
        material=metal,
        name="spindle_pin",
    )
    platter.visual(
        Cylinder(radius=0.018, length=0.005),
        origin=Origin(xyz=(0.000, 0.000, 0.0305)),
        material=metal,
        name="center_clamp",
    )
    platter.visual(
        Box((0.004, 0.012, 0.001)),
        origin=Origin(xyz=(0.145, 0.000, 0.0315)),
        material=mark_red,
        name="platter_phase_mark",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.152, length=0.028),
        mass=4.8,
        origin=Origin(xyz=(0.000, 0.000, 0.018)),
    )

    tonearm = model.part("tonearm_stage")
    tonearm.visual(
        _ring_mesh(
            outer_radius=0.021,
            inner_radius=0.0105,
            height=0.020,
            name="tonearm_collar_ring",
        ),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=arm_metal,
        name="bearing_collar",
    )
    tonearm.visual(
        Box((0.034, 0.030, 0.032)),
        origin=Origin(xyz=(-0.034, 0.000, 0.016)),
        material=arm_metal,
        name="pivot_block",
    )
    tonearm.visual(
        _tonearm_tube_mesh("tonearm_tube"),
        material=arm_metal,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.024, 0.018, 0.004)),
        origin=Origin(xyz=(-0.220, -0.010, 0.028)),
        material=arm_metal,
        name="headshell",
    )
    tonearm.visual(
        Box((0.010, 0.014, 0.008)),
        origin=Origin(xyz=(-0.224, -0.010, 0.0235)),
        material=datum_dark,
        name="cartridge_body",
    )
    tonearm.visual(
        Cylinder(radius=0.0035, length=0.056),
        origin=Origin(
            xyz=(0.010, 0.000, 0.022),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=arm_metal,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.014, length=0.022),
        origin=Origin(
            xyz=(0.045, 0.000, 0.022),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="counterweight",
    )
    tonearm.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(
            xyz=(-0.020, 0.016, 0.020),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name="adjustment_knob",
    )
    tonearm.visual(
        Box((0.014, 0.003, 0.003)),
        origin=Origin(xyz=(0.000, 0.021, 0.0185)),
        material=mark_red,
        name="stage_pointer",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.280, 0.060, 0.040)),
        mass=0.65,
        origin=Origin(xyz=(-0.080, 0.000, 0.020)),
    )

    platter_joint = model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(platter_center[0], platter_center[1], 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
    )

    tonearm_joint = model.articulation(
        "plinth_to_tonearm_stage",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(
            xyz=(tonearm_pivot_xy[0], tonearm_pivot_xy[1], 0.074),
            rpy=(0.0, 0.0, -1.40),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=1.5,
            lower=0.0,
            upper=1.95,
        ),
    )

    model.meta["primary_articulations"] = (platter_joint.name, tonearm_joint.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm_stage")
    platter_joint = object_model.get_articulation("plinth_to_platter")
    tonearm_joint = object_model.get_articulation("plinth_to_tonearm_stage")

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
        "platter_joint_is_continuous_vertical",
        platter_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(platter_joint.axis) == (0.0, 0.0, 1.0),
        details=f"Expected continuous +Z platter joint, got {platter_joint.articulation_type} axis={platter_joint.axis}",
    )
    ctx.check(
        "tonearm_joint_is_revolute_vertical",
        tonearm_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(tonearm_joint.axis) == (0.0, 0.0, 1.0),
        details=f"Expected revolute +Z tonearm joint, got {tonearm_joint.articulation_type} axis={tonearm_joint.axis}",
    )

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_rim",
        negative_elem="datum_deck",
        min_gap=0.003,
        max_gap=0.006,
        name="platter_runs_with_controlled_deck_gap",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="spindle_shaft",
        elem_b="bearing_sleeve",
        min_overlap=0.006,
        name="spindle_is_captured_by_bearing_sleeve",
    )
    ctx.expect_contact(
        platter,
        plinth,
        elem_a="spindle_shaft",
        elem_b="thrust_pad",
        contact_tol=0.0005,
        name="spindle_is_supported_by_thrust_pad",
    )
    ctx.expect_gap(
        tonearm,
        plinth,
        axis="z",
        positive_elem="bearing_collar",
        negative_elem="tonearm_pedestal",
        max_penetration=1e-6,
        max_gap=0.001,
        name="tonearm_stage_sits_on_pedestal_with_controlled_gap",
    )
    ctx.expect_overlap(
        tonearm,
        plinth,
        axes="xy",
        elem_a="bearing_collar",
        elem_b="tonearm_pivot_post",
        min_overlap=0.014,
        name="tonearm_pivot_post_is_captured_inside_stage_collar",
    )

    with ctx.pose({tonearm_joint: 1.82}):
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="cartridge_body",
            elem_b="platter_mat",
            min_overlap=0.006,
            name="tonearm_can_reach_the_record_plane",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="cartridge_body",
            negative_elem="platter_mat",
            min_gap=0.001,
            max_gap=0.004,
            name="cartridge_hovers_above_platter_mat_without_contact",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
