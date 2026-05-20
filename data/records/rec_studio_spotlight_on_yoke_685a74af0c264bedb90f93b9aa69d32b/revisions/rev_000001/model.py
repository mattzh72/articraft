from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _lathed_shell(name: str, outer_profile, inner_profile):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_pt, max_pt = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(min_pt, max_pt))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_spotlight")

    stand_black = model.material("stand_black", rgba=(0.13, 0.13, 0.14, 1.0))
    body_black = model.material("body_black", rgba=(0.11, 0.11, 0.12, 1.0))
    graphite = model.material("graphite", rgba=(0.27, 0.28, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    lens_smoke = model.material("lens_smoke", rgba=(0.45, 0.49, 0.55, 0.65))
    marker_white = model.material("marker_white", rgba=(0.90, 0.90, 0.92, 1.0))

    base_shell = _lathed_shell(
        "stand_crown_shell",
        outer_profile=[
            (0.036, 0.140),
            (0.060, 0.170),
            (0.064, 0.205),
            (0.046, 0.245),
            (0.030, 0.255),
            (0.030, 0.585),
            (0.036, 0.620),
        ],
        inner_profile=[
            (0.0225, 0.145),
            (0.0225, 0.605),
        ],
    )
    leg_beam = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (0.006, 0.0, -0.002),
                (0.150, 0.0, -0.060),
                (0.325, 0.0, -0.148),
                (0.475, 0.0, -0.232),
            ],
            profile=rounded_rect_profile(0.018, 0.014, radius=0.003),
            samples_per_segment=18,
            cap_profile=True,
        ),
        "tripod_leg_beam",
    )
    can_shell = _lathed_shell(
        "spot_can_shell",
        outer_profile=[
            (0.040, -0.165),
            (0.070, -0.155),
            (0.094, -0.115),
            (0.104, -0.030),
            (0.108, 0.080),
            (0.110, 0.135),
            (0.116, 0.168),
        ],
        inner_profile=[
            (0.022, -0.145),
            (0.078, -0.135),
            (0.090, -0.105),
            (0.096, 0.130),
            (0.088, 0.156),
        ],
    )
    dial_cap = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.017,
            body_style="skirted",
            top_diameter=0.044,
            base_diameter=0.052,
            edge_radius=0.001,
        ),
        "mode_dial_cap",
    )

    base = model.part("base")
    base.visual(base_shell, material=stand_black, name="sleeve_shell")
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        for cheek_index, y_local in enumerate((-0.015, 0.015)):
            base.visual(
                Box((0.025, 0.010, 0.036)),
                origin=Origin(
                    xyz=(0.062 * c - y_local * s, 0.062 * s + y_local * c, 0.208),
                    rpy=(0.0, 0.0, angle),
                ),
                material=stand_black,
                name=f"hinge_cheek_{index}_{cheek_index}",
            )
    base.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(xyz=(0.035, 0.0, 0.586), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="clamp_stem",
    )
    base.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.050, 0.0, 0.586)),
        material=graphite,
        name="clamp_knob",
    )

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.0075, length=0.020),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name="hinge_barrel",
        )
        leg.visual(leg_beam, material=stand_black, name="leg_beam")
        leg.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(0.488, 0.0, -0.236)),
            material=rubber,
            name="foot_pad",
        )
        model.articulation(
            f"leg_{index}_hinge",
            ArticulationType.REVOLUTE,
            parent=base,
            child=leg,
            origin=Origin(
                xyz=(0.082 * math.cos(angle), 0.082 * math.sin(angle), 0.208),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=25.0,
                velocity=1.6,
                lower=-0.20,
                upper=0.90,
            ),
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=graphite,
        name="travel_collar",
    )
    mast.visual(
        Cylinder(radius=0.018, length=1.260),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=steel,
        name="mast_tube",
    )
    mast.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.690)),
        material=graphite,
        name="mast_collar",
    )
    mast.visual(
        Cylinder(radius=0.026, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        material=stand_black,
        name="head_receiver",
    )
    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=85.0,
            velocity=0.22,
            lower=0.0,
            upper=0.320,
        ),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.055, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=stand_black,
        name="pan_collar",
    )
    yoke.visual(
        Box((0.090, 0.290, 0.030)),
        origin=Origin(xyz=(-0.015, 0.0, 0.050)),
        material=stand_black,
        name="bridge_plate",
    )
    yoke.visual(
        Box((0.060, 0.026, 0.260)),
        origin=Origin(xyz=(-0.032, 0.145, 0.195)),
        material=body_black,
        name="arm_0",
    )
    yoke.visual(
        Box((0.060, 0.026, 0.260)),
        origin=Origin(xyz=(-0.032, -0.145, 0.195)),
        material=body_black,
        name="arm_1",
    )
    model.articulation(
        "pan_joint",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.860)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5),
    )

    can = model.part("can")
    can.visual(
        can_shell,
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_black,
        name="can_shell",
    )
    can.visual(
        Cylinder(radius=0.090, length=0.020),
        origin=Origin(xyz=(0.166, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_smoke,
        name="front_lens",
    )
    can.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.0, 0.117, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="trunnion_0",
    )
    can.visual(
        Cylinder(radius=0.024, length=0.030),
        origin=Origin(xyz=(0.0, -0.117, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="trunnion_1",
    )
    can.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(-0.145, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="rear_boss",
    )
    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=can,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.4,
            lower=-1.10,
            upper=0.75,
        ),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(-0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shaft",
    )
    dial.visual(
        dial_cap,
        origin=Origin(xyz=(-0.0265, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="dial_cap",
    )
    dial.visual(
        Box((0.010, 0.012, 0.004)),
        origin=Origin(xyz=(-0.0295, 0.022, 0.0)),
        material=marker_white,
        name="marker",
    )
    model.articulation(
        "dial_turn",
        ArticulationType.CONTINUOUS,
        parent=can,
        child=dial,
        origin=Origin(xyz=(-0.160, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    yoke = object_model.get_part("yoke")
    can = object_model.get_part("can")
    dial = object_model.get_part("dial")
    leg_0 = object_model.get_part("leg_0")

    mast_slide = object_model.get_articulation("mast_slide")
    pan_joint = object_model.get_articulation("pan_joint")
    tilt_joint = object_model.get_articulation("tilt_joint")
    dial_turn = object_model.get_articulation("dial_turn")
    leg_hinge = object_model.get_articulation("leg_0_hinge")

    slide_upper = 0.0 if mast_slide.motion_limits is None or mast_slide.motion_limits.upper is None else mast_slide.motion_limits.upper
    tilt_upper = 0.0 if tilt_joint.motion_limits is None or tilt_joint.motion_limits.upper is None else tilt_joint.motion_limits.upper
    leg_upper = 0.0 if leg_hinge.motion_limits is None or leg_hinge.motion_limits.upper is None else leg_hinge.motion_limits.upper

    ctx.expect_origin_distance(
        mast,
        base,
        axes="xy",
        max_dist=0.001,
        name="mast stays centered in tripod sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="mast_tube",
        elem_b="sleeve_shell",
        min_overlap=0.170,
        name="mast remains inserted at rest",
    )
    ctx.expect_contact(
        dial,
        can,
        elem_a="shaft",
        elem_b="rear_boss",
        contact_tol=0.0005,
        name="rear dial seats on its shaft boss",
    )

    rest_head = ctx.part_world_position(yoke)
    with ctx.pose({mast_slide: slide_upper}):
        ctx.expect_origin_distance(
            mast,
            base,
            axes="xy",
            max_dist=0.001,
            name="mast stays centered when extended",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="mast_tube",
            elem_b="sleeve_shell",
            min_overlap=0.110,
            name="mast retains insertion at full extension",
        )
        extended_head = ctx.part_world_position(yoke)
    ctx.check(
        "mast raises the head assembly",
        rest_head is not None and extended_head is not None and extended_head[2] > rest_head[2] + 0.20,
        details=f"rest={rest_head}, extended={extended_head}",
    )

    foot_rest = _aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot_pad"))
    with ctx.pose({leg_hinge: leg_upper}):
        foot_folded = _aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot_pad"))
    ctx.check(
        "tripod leg folds upward on crown hinge",
        foot_rest is not None and foot_folded is not None and foot_folded[2] > foot_rest[2] + 0.10,
        details=f"rest={foot_rest}, folded={foot_folded}",
    )

    lens_rest = _aabb_center(ctx.part_element_world_aabb(can, elem="front_lens"))
    with ctx.pose({pan_joint: math.pi / 2.0}):
        lens_panned = _aabb_center(ctx.part_element_world_aabb(can, elem="front_lens"))
    ctx.check(
        "yoke pans the lamp around the mast",
        lens_rest is not None
        and lens_panned is not None
        and lens_panned[1] > lens_rest[1] + 0.12
        and lens_panned[0] < lens_rest[0] - 0.10,
        details=f"rest={lens_rest}, panned={lens_panned}",
    )

    with ctx.pose({tilt_joint: tilt_upper}):
        lens_tilted = _aabb_center(ctx.part_element_world_aabb(can, elem="front_lens"))
    ctx.check(
        "lamp can tilts upward in the yoke",
        lens_rest is not None and lens_tilted is not None and lens_tilted[2] > lens_rest[2] + 0.08,
        details=f"rest={lens_rest}, tilted={lens_tilted}",
    )

    marker_rest = _aabb_center(ctx.part_element_world_aabb(dial, elem="marker"))
    with ctx.pose({dial_turn: math.pi / 2.0}):
        marker_turned = _aabb_center(ctx.part_element_world_aabb(dial, elem="marker"))
    ctx.check(
        "rear mode dial rotates around its shaft axis",
        marker_rest is not None
        and marker_turned is not None
        and marker_turned[2] > marker_rest[2] + 0.015
        and marker_turned[1] < marker_rest[1] - 0.015,
        details=f"rest={marker_rest}, turned={marker_turned}",
    )

    return ctx.report()


object_model = build_object_model()
