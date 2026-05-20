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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orchestra_pedestal_music_stand")

    satin_black = model.material("satin_black", rgba=(0.15, 0.15, 0.16, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    clip_steel = model.material("clip_steel", rgba=(0.45, 0.47, 0.50, 1.0))

    lower_outer_tube = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.0185, 0.030),
                (0.0185, 0.560),
                (0.0260, 0.560),
                (0.0260, 0.630),
            ],
            inner_profile=[
                (0.0155, 0.030),
                (0.0155, 0.630),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "pedestal_lower_outer_tube",
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.180, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin_black,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.070, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=graphite,
        name="base_hub",
    )
    pedestal.visual(
        lower_outer_tube,
        material=satin_black,
        name="lower_outer_tube",
    )
    pedestal.visual(
        Cylinder(radius=0.005, length=0.032),
        origin=Origin(
            xyz=(0.030, 0.0, 0.595),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="clamp_stem",
    )
    pedestal.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.051, 0.0, 0.595)),
        material=dark_rubber,
        name="clamp_knob",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.40, 0.40, 0.66)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
    )

    upper_mast = model.part("upper_mast")
    upper_mast.visual(
        Cylinder(radius=0.0125, length=0.780),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=graphite,
        name="inner_tube",
    )
    upper_mast.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile=[
                    (0.0235, 0.000),
                    (0.0235, 0.012),
                ],
                inner_profile=[
                    (0.0125, 0.000),
                    (0.0125, 0.012),
                ],
                segments=40,
                start_cap="flat",
                end_cap="flat",
                lip_samples=6,
            ),
            "upper_mast_stop_collar",
        ),
        material=satin_black,
        name="stop_collar",
    )
    upper_mast.visual(
        Box((0.060, 0.045, 0.032)),
        origin=Origin(xyz=(0.0, -0.004, 0.496)),
        material=satin_black,
        name="mast_head",
    )
    upper_mast.inertial = Inertial.from_geometry(
        Box((0.070, 0.060, 0.800)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )

    model.articulation(
        "pedestal_to_upper_mast",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=upper_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.630)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.220,
        ),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Box((0.050, 0.013, 0.015)),
        origin=Origin(xyz=(0.0, -0.02025, 0.0075)),
        material=satin_black,
        name="mount_block",
    )
    yoke.visual(
        Box((0.024, 0.014, 0.115)),
        origin=Origin(xyz=(0.0, -0.028, 0.0725)),
        material=satin_black,
        name="yoke_neck",
    )
    yoke.visual(
        Cylinder(radius=0.009, length=0.340),
        origin=Origin(
            xyz=(0.0, -0.028, 0.130),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=clip_steel,
        name="yoke_crossbar",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.360, 0.060, 0.150)),
        mass=0.6,
        origin=Origin(xyz=(0.0, -0.018, 0.075)),
    )

    model.articulation(
        "upper_mast_to_yoke",
        ArticulationType.FIXED,
        parent=upper_mast,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.512)),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.540, 0.006, 0.340)),
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
        material=graphite,
        name="panel",
    )
    tray.visual(
        Box((0.120, 0.028, 0.230)),
        origin=Origin(xyz=(0.0, 0.031, -0.015)),
        material=satin_black,
        name="center_reinforcement",
    )
    tray.visual(
        Box((0.016, 0.024, 0.340)),
        origin=Origin(xyz=(-0.262, 0.055, 0.0)),
        material=graphite,
        name="left_flange",
    )
    tray.visual(
        Box((0.016, 0.024, 0.340)),
        origin=Origin(xyz=(0.262, 0.055, 0.0)),
        material=graphite,
        name="right_flange",
    )
    tray.visual(
        Box((0.500, 0.036, 0.014)),
        origin=Origin(xyz=(0.0, 0.061, -0.163)),
        material=graphite,
        name="bottom_shelf",
    )
    tray.visual(
        Box((0.500, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, 0.076, -0.160)),
        material=graphite,
        name="shelf_lip",
    )
    tray.visual(
        Cylinder(radius=0.009, length=0.500),
        origin=Origin(
            xyz=(0.0, 0.050, 0.173),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=clip_steel,
        name="top_rail",
    )
    tray.visual(
        Box((0.330, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, 0.021, 0.0)),
        material=satin_black,
        name="pivot_bridge",
    )
    tray.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(
            xyz=(-0.155, 0.017, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=clip_steel,
        name="left_pivot_lug",
    )
    tray.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(
            xyz=(0.155, 0.017, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=clip_steel,
        name="right_pivot_lug",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.560, 0.080, 0.360)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.032, 0.0)),
    )

    model.articulation(
        "yoke_to_tray",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=tray,
        origin=Origin(xyz=(0.0, -0.028, 0.130)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-0.50,
            upper=0.70,
        ),
    )

    def add_page_clip(name: str) -> None:
        clip = model.part(name)
        clip.visual(
            Cylinder(radius=0.006, length=0.030),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=clip_steel,
            name="clip_barrel",
        )
        clip.visual(
            Box((0.022, 0.010, 0.014)),
            origin=Origin(xyz=(0.0, 0.006, 0.003)),
            material=clip_steel,
            name="hinge_leaf",
        )
        clip.visual(
            Box((0.022, 0.006, 0.054)),
            origin=Origin(xyz=(0.0, 0.008, 0.027)),
            material=clip_steel,
            name="clip_arm",
        )
        clip.visual(
            Box((0.028, 0.012, 0.008)),
            origin=Origin(xyz=(0.0, 0.012, 0.058)),
            material=dark_rubber,
            name="clip_pad",
        )
        clip.inertial = Inertial.from_geometry(
            Box((0.032, 0.030, 0.080)),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.012, 0.035)),
        )

    add_page_clip("left_page_clip")
    add_page_clip("right_page_clip")

    model.articulation(
        "tray_to_left_page_clip",
        ArticulationType.REVOLUTE,
        parent=tray,
        child="left_page_clip",
        origin=Origin(xyz=(-0.155, 0.062, 0.182)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-1.20,
            upper=0.35,
        ),
    )
    model.articulation(
        "tray_to_right_page_clip",
        ArticulationType.REVOLUTE,
        parent=tray,
        child="right_page_clip",
        origin=Origin(xyz=(0.155, 0.062, 0.182)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-1.20,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    pedestal = object_model.get_part("pedestal")
    upper_mast = object_model.get_part("upper_mast")
    yoke = object_model.get_part("yoke")
    tray = object_model.get_part("tray")
    left_clip = object_model.get_part("left_page_clip")
    right_clip = object_model.get_part("right_page_clip")

    mast_slide = object_model.get_articulation("pedestal_to_upper_mast")
    tray_tilt = object_model.get_articulation("yoke_to_tray")
    left_clip_joint = object_model.get_articulation("tray_to_left_page_clip")

    slide_upper = mast_slide.motion_limits.upper if mast_slide.motion_limits else 0.0
    tray_upper = tray_tilt.motion_limits.upper if tray_tilt.motion_limits else 0.0
    clip_lower = left_clip_joint.motion_limits.lower if left_clip_joint.motion_limits else 0.0

    ctx.expect_within(
        upper_mast,
        pedestal,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="lower_outer_tube",
        margin=0.0035,
        name="upper mast stays centered inside the lower mast",
    )
    ctx.expect_overlap(
        upper_mast,
        pedestal,
        axes="z",
        elem_a="inner_tube",
        elem_b="lower_outer_tube",
        min_overlap=0.295,
        name="collapsed upper mast remains deeply inserted",
    )
    ctx.expect_gap(
        tray,
        yoke,
        axis="y",
        positive_elem="left_pivot_lug",
        negative_elem="yoke_crossbar",
        min_gap=0.0,
        max_gap=0.002,
        name="tray hinge lug sits on the yoke crossbar",
    )
    ctx.expect_origin_distance(
        left_clip,
        right_clip,
        axes="x",
        min_dist=0.300,
        max_dist=0.320,
        name="page clips are spaced across the tray top rail",
    )

    rest_mast_pos = ctx.part_world_position(upper_mast)
    with ctx.pose({mast_slide: slide_upper}):
        ctx.expect_within(
            upper_mast,
            pedestal,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="lower_outer_tube",
            margin=0.0035,
            name="extended upper mast stays centered inside the lower mast",
        )
        ctx.expect_overlap(
            upper_mast,
            pedestal,
            axes="z",
            elem_a="inner_tube",
            elem_b="lower_outer_tube",
            min_overlap=0.075,
            name="extended upper mast still retains insertion",
        )
        extended_mast_pos = ctx.part_world_position(upper_mast)
    ctx.check(
        "upper mast extends upward",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.15,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    rest_top_rail_center = _aabb_center(ctx.part_element_world_aabb(tray, elem="top_rail"))
    with ctx.pose({tray_tilt: tray_upper}):
        tilted_top_rail_center = _aabb_center(ctx.part_element_world_aabb(tray, elem="top_rail"))
    ctx.check(
        "tray tilts backward about the horizontal yoke hinge",
        rest_top_rail_center is not None
        and tilted_top_rail_center is not None
        and tilted_top_rail_center[1] < rest_top_rail_center[1] - 0.09,
        details=f"rest={rest_top_rail_center}, tilted={tilted_top_rail_center}",
    )

    open_clip_pad_center = _aabb_center(ctx.part_element_world_aabb(left_clip, elem="clip_pad"))
    with ctx.pose({left_clip_joint: clip_lower}):
        closed_clip_pad_center = _aabb_center(ctx.part_element_world_aabb(left_clip, elem="clip_pad"))
    ctx.check(
        "page clip swings down from the tray top rail",
        open_clip_pad_center is not None
        and closed_clip_pad_center is not None
        and closed_clip_pad_center[2] < open_clip_pad_center[2] - 0.020
        and closed_clip_pad_center[1] > open_clip_pad_center[1] + 0.010,
        details=f"open={open_clip_pad_center}, closed={closed_clip_pad_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
