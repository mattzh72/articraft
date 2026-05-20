from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="symphony_music_stand")

    black = model.material("satin_black_metal", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_edge = model.material("worn_black_edges", rgba=(0.05, 0.048, 0.045, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.62, 0.64, 0.62, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))

    base = model.part("base")
    base_profile = [
        (0.000, 0.000),
        (0.130, 0.000),
        (0.160, 0.010),
        (0.160, 0.028),
        (0.130, 0.040),
        (0.020, 0.040),
        (0.000, 0.034),
    ]
    base.visual(
        mesh_from_geometry(LatheGeometry(base_profile, segments=72), "weighted_round_base"),
        material=black,
        name="weighted_round_base",
    )
    base.visual(
        Cylinder(radius=0.037, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=dark_edge,
        name="base_socket",
    )
    lower_tube = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.020, 0.0), (0.020, 0.720)],
        inner_profile=[(0.0120, 0.0), (0.0120, 0.720)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    base.visual(
        mesh_from_geometry(lower_tube, "lower_tube"),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=black,
        name="lower_tube",
    )
    top_collar = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.028, 0.0), (0.031, 0.012), (0.031, 0.042), (0.026, 0.052)],
        inner_profile=[(0.0120, 0.0), (0.0120, 0.052)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    base.visual(
        mesh_from_geometry(top_collar, "top_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.742)),
        material=dark_edge,
        name="top_collar",
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.012, length=0.860),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=chrome,
        name="inner_tube",
    )
    mast.visual(
        Box((0.046, 0.060, 0.030)),
        origin=Origin(xyz=(0.0, -0.030, 0.535)),
        material=black,
        name="head_web",
    )
    mast.visual(
        Box((0.130, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, -0.055, 0.535)),
        material=black,
        name="head_bridge",
    )
    mast.visual(
        Box((0.018, 0.028, 0.070)),
        origin=Origin(xyz=(-0.055, -0.055, 0.580)),
        material=black,
        name="tilt_ear_0",
    )
    mast.visual(
        Box((0.018, 0.028, 0.070)),
        origin=Origin(xyz=(0.055, -0.055, 0.580)),
        material=black,
        name="tilt_ear_1",
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.760)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=95.0, velocity=0.18, lower=0.0, upper=0.250),
    )

    desk = model.part("desk")
    desk.visual(
        Box((0.560, 0.012, 0.360)),
        origin=Origin(xyz=(0.0, -0.045, 0.090)),
        material=black,
        name="desk_plate",
    )
    desk.visual(
        Cylinder(radius=0.014, length=0.092),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_edge,
        name="tilt_barrel",
    )
    for x in (-0.035, 0.035):
        desk.visual(
            Box((0.018, 0.052, 0.016)),
            origin=Origin(xyz=(x, -0.026, 0.010)),
            material=dark_edge,
            name=f"barrel_strap_{0 if x < 0 else 1}",
        )
    desk.visual(
        Box((0.560, 0.055, 0.014)),
        origin=Origin(xyz=(0.0, -0.075, -0.095)),
        material=black,
        name="score_shelf",
    )
    desk.visual(
        Box((0.560, 0.012, 0.055)),
        origin=Origin(xyz=(0.0, -0.105, -0.063)),
        material=black,
        name="score_lip",
    )
    desk.visual(
        Box((0.560, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.056, 0.268)),
        material=dark_edge,
        name="top_rim",
    )
    for x in (-0.271, 0.271):
        desk.visual(
            Box((0.018, 0.010, 0.340)),
            origin=Origin(xyz=(x, -0.056, 0.090)),
            material=dark_edge,
            name=f"side_rim_{0 if x < 0 else 1}",
        )

    model.articulation(
        "desk_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, -0.055, 0.580), rpy=(-0.22, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.45, upper=0.45),
    )

    def add_clip(name: str, x: float, inward_sign: float) -> None:
        clip = model.part(name)
        angle = -0.55 * inward_sign
        arm_center_x = 0.036 * inward_sign
        pad_center_x = 0.070 * inward_sign
        clip.visual(
            Cylinder(radius=0.017, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_edge,
            name="pivot_button",
        )
        clip.visual(
            Box((0.018, 0.006, 0.140)),
            origin=Origin(xyz=(arm_center_x, 0.003, -0.058), rpy=(0.0, angle, 0.0)),
            material=black,
            name="retainer_arm",
        )
        clip.visual(
            Box((0.030, 0.007, 0.018)),
            origin=Origin(xyz=(pad_center_x, 0.003, -0.116), rpy=(0.0, angle, 0.0)),
            material=rubber,
            name="rubber_tip",
        )
        model.articulation(
            f"{name}_pivot",
            ArticulationType.REVOLUTE,
            parent=desk,
            child=clip,
            origin=Origin(xyz=(x, -0.065, 0.246)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=-0.85, upper=0.85),
        )

    add_clip("clip_0", -0.235, 1.0)
    add_clip("clip_1", 0.235, -1.0)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    clip_0 = object_model.get_part("clip_0")
    clip_1 = object_model.get_part("clip_1")
    slide = object_model.get_articulation("mast_slide")
    tilt = object_model.get_articulation("desk_tilt")
    clip_0_pivot = object_model.get_articulation("clip_0_pivot")
    clip_1_pivot = object_model.get_articulation("clip_1_pivot")

    ctx.allow_overlap(
        base,
        mast,
        elem_a="lower_tube",
        elem_b="inner_tube",
        reason=(
            "The visible lower tube is a sleeve for the telescoping mast; its "
            "mesh-backed hollow bore is represented conservatively by the overlap checker."
        ),
    )
    ctx.allow_overlap(
        base,
        mast,
        elem_a="top_collar",
        elem_b="inner_tube",
        reason="The mast intentionally passes through the black clamp collar at the sleeve mouth.",
    )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="lower_tube",
        margin=0.001,
        name="telescoping mast is centered in the lower tube",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="lower_tube",
        min_overlap=0.250,
        name="collapsed mast retains deep insertion",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="top_collar",
        min_overlap=0.040,
        name="mast passes through the clamp collar",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({slide: 0.250}):
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="lower_tube",
            margin=0.001,
            name="extended mast stays centered in the lower tube",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="lower_tube",
            min_overlap=0.025,
            name="extended mast remains captured in the sleeve",
        )
        extended_mast_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast slide raises the head",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.20,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    ctx.expect_contact(
        desk,
        mast,
        elem_a="tilt_barrel",
        elem_b="tilt_ear_0",
        contact_tol=0.006,
        name="desk tilt barrel is captured by one yoke ear",
    )
    ctx.expect_contact(
        desk,
        mast,
        elem_a="tilt_barrel",
        elem_b="tilt_ear_1",
        contact_tol=0.006,
        name="desk tilt barrel is captured by the other yoke ear",
    )
    ctx.expect_contact(
        clip_0,
        desk,
        elem_a="pivot_button",
        elem_b="top_rim",
        contact_tol=0.002,
        name="clip 0 pivot sits on the upper rim",
    )
    ctx.expect_contact(
        clip_1,
        desk,
        elem_a="pivot_button",
        elem_b="top_rim",
        contact_tol=0.002,
        name="clip 1 pivot sits on the upper rim",
    )

    rest_desk_aabb = ctx.part_world_aabb(desk)
    with ctx.pose({tilt: 0.35}):
        tilted_desk_aabb = ctx.part_world_aabb(desk)
    ctx.check(
        "desk tilt changes the desk attitude",
        rest_desk_aabb is not None
        and tilted_desk_aabb is not None
        and abs(tilted_desk_aabb[0][2] - rest_desk_aabb[0][2]) > 0.018,
        details=f"rest={rest_desk_aabb}, tilted={tilted_desk_aabb}",
    )

    clip_0_rest = ctx.part_world_aabb(clip_0)
    clip_1_rest = ctx.part_world_aabb(clip_1)
    with ctx.pose({clip_0_pivot: 0.50, clip_1_pivot: -0.50}):
        clip_0_swung = ctx.part_world_aabb(clip_0)
        clip_1_swung = ctx.part_world_aabb(clip_1)
    ctx.check(
        "page clips rotate on their pivots",
        clip_0_rest is not None
        and clip_1_rest is not None
        and clip_0_swung is not None
        and clip_1_swung is not None
        and abs(clip_0_swung[0][0] - clip_0_rest[0][0]) > 0.010
        and abs(clip_1_swung[1][0] - clip_1_rest[1][0]) > 0.010,
        details=f"clip0_rest={clip_0_rest}, clip0_swung={clip_0_swung}, clip1_rest={clip_1_rest}, clip1_swung={clip_1_swung}",
    )

    return ctx.report()


object_model = build_object_model()
