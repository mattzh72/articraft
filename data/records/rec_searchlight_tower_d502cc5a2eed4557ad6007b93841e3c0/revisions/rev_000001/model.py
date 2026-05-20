from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _lamp_shell_mesh():
    """Thin tapered searchlight barrel, authored along local Z then turned to X."""
    return LatheGeometry.from_shell_profiles(
        [
            (0.072, -0.160),
            (0.092, -0.138),
            (0.112, -0.060),
            (0.128, 0.080),
            (0.146, 0.218),
            (0.150, 0.244),
        ],
        [
            (0.052, -0.142),
            (0.076, -0.108),
            (0.094, 0.026),
            (0.112, 0.174),
            (0.122, 0.236),
        ],
        segments=64,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    painted_metal = model.material("painted_metal", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.07, 0.08, 0.09, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.46, 0.48, 0.50, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.68, 0.10, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.55, 0.78, 0.96, 0.55))
    warm_reflector = model.material("warm_reflector", rgba=(0.92, 0.84, 0.62, 1.0))

    mast_support = model.part("mast_support")
    mast_support.visual(
        Cylinder(radius=0.38, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_steel,
        name="ground_plate",
    )
    mast_support.visual(
        Cylinder(radius=0.078, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=painted_metal,
        name="lower_mast_collar",
    )
    mast_support.visual(
        Cylinder(radius=0.033, length=1.78),
        origin=Origin(xyz=(0.0, 0.0, 0.925)),
        material=painted_metal,
        name="central_mast",
    )
    mast_support.visual(
        Cylinder(radius=0.086, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 1.770)),
        material=painted_metal,
        name="upper_mast_collar",
    )
    mast_support.visual(
        Cylinder(radius=0.047, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 1.860)),
        material=painted_metal,
        name="bearing_stem",
    )

    # Open, light tower bracing: three slim tubes rising from the base plate.
    for index in range(3):
        angle = index * 2.0 * math.pi / 3.0 + math.pi / 6.0
        lower = (0.30 * math.cos(angle), 0.30 * math.sin(angle), 0.060)
        mid = (0.18 * math.cos(angle + 0.10), 0.18 * math.sin(angle + 0.10), 0.860)
        upper = (0.070 * math.cos(angle), 0.070 * math.sin(angle), 1.740)
        mast_support.visual(
            _mesh(
                f"mast_brace_{index}",
                tube_from_spline_points(
                    [lower, mid, upper],
                    radius=0.013,
                    samples_per_segment=10,
                    radial_segments=14,
                    cap_ends=True,
                ),
            ),
            material=worn_steel,
            name=f"mast_brace_{index}",
        )

    mast_support.visual(
        _mesh("mid_brace_ring", TorusGeometry(radius=0.180, tube=0.010, radial_segments=12, tubular_segments=48)),
        origin=Origin(xyz=(0.0, 0.0, 0.860)),
        material=worn_steel,
        name="mid_brace_ring",
    )
    mast_support.visual(
        Cylinder(radius=0.130, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.930)),
        material=dark_steel,
        name="fixed_bearing",
    )
    mast_support.visual(
        Cylinder(radius=0.043, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 1.975)),
        material=worn_steel,
        name="fixed_spindle",
    )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.132, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_steel,
        name="rotary_table",
    )
    pan_stage.visual(
        _mesh("slewing_ring", TorusGeometry(radius=0.132, tube=0.010, radial_segments=14, tubular_segments=56)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=worn_steel,
        name="slewing_ring",
    )
    pan_stage.visual(
        Cylinder(radius=0.055, length=0.165),
        origin=Origin(xyz=(0.0, 0.0, 0.0825)),
        material=worn_steel,
        name="bearing_sleeve",
    )
    pan_stage.visual(
        Cylinder(radius=0.047, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=painted_metal,
        name="yoke_riser",
    )
    pan_stage.visual(
        Box((0.260, 0.470, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.272)),
        material=safety_yellow,
        name="yoke_base",
    )
    pan_stage.visual(
        Box((0.078, 0.046, 0.235)),
        origin=Origin(xyz=(0.0, -0.190, 0.405)),
        material=safety_yellow,
        name="yoke_arm_0_lower",
    )
    pan_stage.visual(
        Box((0.078, 0.046, 0.235)),
        origin=Origin(xyz=(0.0, 0.190, 0.405)),
        material=safety_yellow,
        name="yoke_arm_1_lower",
    )
    pan_stage.visual(
        Box((0.078, 0.046, 0.115)),
        origin=Origin(xyz=(0.0, -0.190, 0.665)),
        material=safety_yellow,
        name="yoke_arm_0_upper",
    )
    pan_stage.visual(
        Box((0.078, 0.046, 0.115)),
        origin=Origin(xyz=(0.0, 0.190, 0.665)),
        material=safety_yellow,
        name="yoke_arm_1_upper",
    )
    pan_stage.visual(
        Box((0.018, 0.046, 0.150)),
        origin=Origin(xyz=(0.048, -0.190, 0.535)),
        material=safety_yellow,
        name="yoke_arm_0_web",
    )
    pan_stage.visual(
        Box((0.018, 0.046, 0.150)),
        origin=Origin(xyz=(0.048, 0.190, 0.535)),
        material=safety_yellow,
        name="yoke_arm_1_web",
    )
    pan_stage.visual(
        Cylinder(radius=0.041, length=0.078),
        origin=Origin(xyz=(0.0, -0.190, 0.585), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_collar_0",
    )
    pan_stage.visual(
        Cylinder(radius=0.041, length=0.078),
        origin=Origin(xyz=(0.0, 0.190, 0.585), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="bearing_collar_1",
    )
    pan_stage.visual(
        _mesh(
            "rear_yoke_tie",
            tube_from_spline_points(
                [(0.0, -0.190, 0.325), (-0.080, 0.0, 0.372), (0.0, 0.190, 0.325)],
                radius=0.014,
                samples_per_segment=8,
                radial_segments=14,
                cap_ends=True,
            ),
        ),
        material=worn_steel,
        name="rear_yoke_tie",
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.023, length=0.455),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="tilt_trunnion",
    )
    lamp_head.visual(
        _mesh("lamp_shell", _lamp_shell_mesh()),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_metal,
        name="lamp_shell",
    )
    lamp_head.visual(
        Cylinder(radius=0.129, length=0.012),
        origin=Origin(xyz=(0.236, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp_head.visual(
        _mesh("front_bezel", TorusGeometry(radius=0.140, tube=0.012, radial_segments=16, tubular_segments=64)),
        origin=Origin(xyz=(0.235, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.083, length=0.030),
        origin=Origin(xyz=(-0.156, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_cap",
    )
    lamp_head.visual(
        Cylinder(radius=0.040, length=0.035),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_reflector,
        name="reflector_core",
    )
    lamp_head.visual(
        Cylinder(radius=0.009, length=0.090),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="bulb_stem",
    )
    lamp_head.visual(
        _mesh(
            "rear_handle",
            tube_from_spline_points(
                [(-0.160, -0.058, 0.050), (-0.215, 0.0, 0.105), (-0.160, 0.058, 0.050)],
                radius=0.008,
                samples_per_segment=12,
                radial_segments=12,
                cap_ends=True,
            ),
        ),
        material=worn_steel,
        name="rear_handle",
    )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=mast_support,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.960)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.0, lower=-0.60, upper=1.05),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("mast_support")
    pan = object_model.get_part("pan_stage")
    lamp = object_model.get_part("lamp_head")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_axis = object_model.get_articulation("tilt_axis")

    ctx.allow_overlap(
        tower,
        pan,
        elem_a="fixed_spindle",
        elem_b="bearing_sleeve",
        reason="The exposed pan stage is represented with a captured spindle running inside the rotary sleeve.",
    )
    ctx.allow_overlap(
        tower,
        pan,
        elem_a="fixed_spindle",
        elem_b="rotary_table",
        reason="The rotary table has a represented central bore, with the mast spindle intentionally passing through it.",
    )
    ctx.allow_overlap(
        pan,
        lamp,
        elem_a="bearing_collar_0",
        elem_b="tilt_trunnion",
        reason="The lamp trunnion is intentionally captured in the yoke bearing collar.",
    )
    ctx.allow_overlap(
        pan,
        lamp,
        elem_a="bearing_collar_1",
        elem_b="tilt_trunnion",
        reason="The lamp trunnion is intentionally captured in the yoke bearing collar.",
    )

    ctx.expect_gap(
        pan,
        tower,
        axis="z",
        positive_elem="rotary_table",
        negative_elem="fixed_bearing",
        max_gap=0.002,
        max_penetration=0.001,
        name="rotary table seats on fixed bearing",
    )
    ctx.expect_within(
        tower,
        pan,
        axes="xy",
        inner_elem="fixed_spindle",
        outer_elem="bearing_sleeve",
        margin=0.002,
        name="pan spindle is concentric in sleeve",
    )
    ctx.expect_overlap(
        tower,
        pan,
        axes="z",
        elem_a="fixed_spindle",
        elem_b="bearing_sleeve",
        min_overlap=0.080,
        name="pan spindle remains captured vertically",
    )
    ctx.expect_within(
        tower,
        pan,
        axes="xy",
        inner_elem="fixed_spindle",
        outer_elem="rotary_table",
        margin=0.002,
        name="spindle passes through rotary table center",
    )
    ctx.expect_overlap(
        tower,
        pan,
        axes="z",
        elem_a="fixed_spindle",
        elem_b="rotary_table",
        min_overlap=0.030,
        name="spindle passes through table bore",
    )
    ctx.expect_gap(
        pan,
        lamp,
        axis="y",
        positive_elem="yoke_arm_1_lower",
        negative_elem="lamp_shell",
        min_gap=0.008,
        name="lamp clears positive yoke arm",
    )
    ctx.expect_gap(
        lamp,
        pan,
        axis="y",
        positive_elem="lamp_shell",
        negative_elem="yoke_arm_0_lower",
        min_gap=0.008,
        name="lamp clears negative yoke arm",
    )
    ctx.expect_within(
        lamp,
        pan,
        axes="xz",
        inner_elem="tilt_trunnion",
        outer_elem="bearing_collar_1",
        margin=0.003,
        name="tilt pin centered in collar",
    )
    ctx.expect_overlap(
        lamp,
        pan,
        axes="y",
        elem_a="tilt_trunnion",
        elem_b="bearing_collar_1",
        min_overlap=0.035,
        name="tilt pin passes through yoke collar",
    )

    rest_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    with ctx.pose({pan_axis: math.pi / 2.0}):
        panned_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    ctx.check(
        "pan joint swings lamp around vertical axis",
        rest_lens is not None
        and panned_lens is not None
        and 0.5 * (panned_lens[0][1] + panned_lens[1][1])
        > 0.5 * (rest_lens[0][0] + rest_lens[1][0]) - 0.020,
        details=f"rest_lens={rest_lens}, panned_lens={panned_lens}",
    )

    with ctx.pose({tilt_axis: 0.75}):
        tilted_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    ctx.check(
        "tilt joint raises lamp nose",
        rest_lens is not None
        and tilted_lens is not None
        and 0.5 * (tilted_lens[0][2] + tilted_lens[1][2])
        > 0.5 * (rest_lens[0][2] + rest_lens[1][2]) + 0.060,
        details=f"rest_lens={rest_lens}, tilted_lens={tilted_lens}",
    )

    return ctx.report()


object_model = build_object_model()
