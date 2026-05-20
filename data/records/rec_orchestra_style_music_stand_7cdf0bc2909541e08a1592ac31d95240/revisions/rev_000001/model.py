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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orchestra_music_stand")

    powder_black = model.material("powder_black", rgba=(0.15, 0.15, 0.16, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    sleeve_outer = 0.048
    sleeve_inner = 0.034
    sleeve_wall = (sleeve_outer - sleeve_inner) / 2.0
    sleeve_height = 0.54
    sleeve_bottom = 0.12
    sleeve_top = sleeve_bottom + sleeve_height
    sleeve_center_z = sleeve_bottom + sleeve_height / 2.0
    wall_offset = sleeve_inner / 2.0 + sleeve_wall / 2.0

    base = model.part("base")
    base.visual(
        Box((0.102, 0.102, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=powder_black,
        name="tripod_hub",
    )
    base.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(0.0, wall_offset, sleeve_center_z)),
        material=powder_black,
        name="sleeve_front",
    )
    base.visual(
        Box((sleeve_outer, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(0.0, -wall_offset, sleeve_center_z)),
        material=powder_black,
        name="sleeve_back",
    )
    base.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(xyz=(wall_offset, 0.0, sleeve_center_z)),
        material=powder_black,
        name="sleeve_right",
    )
    base.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(xyz=(-wall_offset, 0.0, sleeve_center_z)),
        material=powder_black,
        name="sleeve_left",
    )

    clamp_outer = 0.070
    clamp_wall = (clamp_outer - sleeve_inner) / 2.0
    clamp_offset = sleeve_inner / 2.0 + clamp_wall / 2.0
    clamp_center_z = sleeve_top + 0.022
    clamp_height = 0.044
    base.visual(
        Box((clamp_outer, clamp_wall, clamp_height)),
        origin=Origin(xyz=(0.0, clamp_offset, clamp_center_z)),
        material=powder_black,
        name="clamp_front",
    )
    base.visual(
        Box((clamp_outer, clamp_wall, clamp_height)),
        origin=Origin(xyz=(0.0, -clamp_offset, clamp_center_z)),
        material=powder_black,
        name="clamp_back",
    )
    base.visual(
        Box((clamp_wall, clamp_outer, clamp_height)),
        origin=Origin(xyz=(clamp_offset, 0.0, clamp_center_z)),
        material=powder_black,
        name="clamp_right",
    )
    base.visual(
        Box((clamp_wall, clamp_outer, clamp_height)),
        origin=Origin(xyz=(-clamp_offset, 0.0, clamp_center_z)),
        material=powder_black,
        name="clamp_left",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(
            xyz=(0.046, 0.0, sleeve_top + 0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="clamp_screw",
    )
    base.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.062, 0.0, sleeve_top + 0.010)),
        material=rubber,
        name="clamp_knob",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mesh = tube_from_spline_points(
            [
                (0.035 * c, 0.035 * s, 0.118),
                (0.185 * c, 0.185 * s, 0.072),
                (0.355 * c, 0.355 * s, 0.020),
            ],
            radius=0.010,
            samples_per_segment=18,
            radial_segments=20,
            cap_ends=True,
        )
        base.visual(
            mesh_from_geometry(leg_mesh, f"music_stand_tripod_leg_{index}"),
            material=powder_black,
            name=f"leg_{index}",
        )
        base.visual(
            Sphere(radius=0.013),
            origin=Origin(xyz=(0.355 * c, 0.355 * s, 0.013)),
            material=rubber,
            name=f"foot_{index}",
        )

    base.inertial = Inertial.from_geometry(
        Box((0.78, 0.78, 0.78)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
    )

    stem = model.part("upper_stem")
    stem.visual(
        Box((0.029, 0.029, 0.920)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=graphite,
        name="inner_mast",
    )
    stem.visual(
        Box((0.018, 0.005, 0.070)),
        origin=Origin(xyz=(0.0, 0.0145, -0.140)),
        material=steel,
        name="guide_pad",
    )
    stem.visual(
        Box((0.045, 0.045, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.500)),
        material=powder_black,
        name="stem_head_block",
    )
    stem.visual(
        Box((0.038, 0.115, 0.018)),
        origin=Origin(xyz=(0.0, 0.0575, 0.505)),
        material=powder_black,
        name="tilt_support_arm",
    )
    stem.visual(
        Box((0.090, 0.025, 0.038)),
        origin=Origin(xyz=(0.0, 0.1145, 0.505)),
        material=powder_black,
        name="tilt_head",
    )
    stem.inertial = Inertial.from_geometry(
        Box((0.10, 0.18, 0.98)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.060, 0.120)),
    )

    stem_slide = model.articulation(
        "base_to_upper_stem",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, sleeve_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.18,
            lower=0.0,
            upper=0.220,
        ),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.520, 0.0035, 0.340)),
        origin=Origin(xyz=(0.0, 0.00175, 0.170)),
        material=graphite,
        name="tray_back",
    )
    tray.visual(
        Box((0.014, 0.032, 0.340)),
        origin=Origin(xyz=(-0.253, 0.016, 0.170)),
        material=graphite,
        name="tray_left_flange",
    )
    tray.visual(
        Box((0.014, 0.032, 0.340)),
        origin=Origin(xyz=(0.253, 0.016, 0.170)),
        material=graphite,
        name="tray_right_flange",
    )
    tray.visual(
        Box((0.520, 0.032, 0.014)),
        origin=Origin(xyz=(0.0, 0.016, 0.333)),
        material=graphite,
        name="tray_top_flange",
    )
    tray.visual(
        Box((0.480, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.007, 0.009)),
        material=graphite,
        name="tray_bottom_strip",
    )
    tray.visual(
        Box((0.460, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.018, 0.005)),
        material=steel,
        name="tray_hinge_leaf",
    )
    tray.visual(
        Box((0.120, 0.018, 0.230)),
        origin=Origin(xyz=(0.0, 0.010, 0.170)),
        material=powder_black,
        name="tray_center_rib",
    )
    tray.visual(
        Box((0.080, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, -0.009, 0.030)),
        material=powder_black,
        name="tray_mount_block",
    )
    tray.visual(
        Cylinder(radius=0.008, length=0.090),
        origin=Origin(
            xyz=(0.0, -0.010, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="tray_tilt_trunnion",
    )
    tray.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(
            xyz=(-0.269, 0.006, 0.000),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="left_tilt_knob",
    )
    tray.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(
            xyz=(0.269, 0.006, 0.000),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="right_tilt_knob",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.56, 0.09, 0.38)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.012, 0.180)),
    )

    tray_tilt = model.articulation(
        "upper_stem_to_tray",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=tray,
        origin=Origin(xyz=(0.0, 0.145, 0.505)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-0.40,
            upper=0.42,
        ),
    )

    lip = model.part("retaining_lip")
    lip.visual(
        Cylinder(radius=0.004, length=0.460),
        origin=Origin(
            xyz=(0.0, -0.003, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="lip_hinge_barrel",
    )
    lip.visual(
        Box((0.472, 0.032, 0.005)),
        origin=Origin(xyz=(0.0, 0.016, 0.0025)),
        material=graphite,
        name="lip_shelf",
    )
    lip.visual(
        Box((0.472, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, 0.031, 0.014)),
        material=graphite,
        name="lip_fence",
    )
    lip.visual(
        Box((0.010, 0.032, 0.018)),
        origin=Origin(xyz=(-0.231, 0.016, 0.009)),
        material=graphite,
        name="lip_left_end",
    )
    lip.visual(
        Box((0.010, 0.032, 0.018)),
        origin=Origin(xyz=(0.231, 0.016, 0.009)),
        material=graphite,
        name="lip_right_end",
    )
    lip.inertial = Inertial.from_geometry(
        Box((0.48, 0.05, 0.03)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.015, 0.010)),
    )

    lip_hinge = model.articulation(
        "tray_to_retaining_lip",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=lip,
        origin=Origin(xyz=(0.0, 0.031, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=-1.25,
            upper=1.05,
        ),
    )

    model.meta["primary_articulations"] = (
        stem_slide.name,
        tray_tilt.name,
        lip_hinge.name,
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

    base = object_model.get_part("base")
    stem = object_model.get_part("upper_stem")
    tray = object_model.get_part("tray")
    lip = object_model.get_part("retaining_lip")
    stem_slide = object_model.get_articulation("base_to_upper_stem")
    tray_tilt = object_model.get_articulation("upper_stem_to_tray")
    lip_hinge = object_model.get_articulation("tray_to_retaining_lip")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mn, mx = aabb
        return tuple((mn[i] + mx[i]) / 2.0 for i in range(3))

    ctx.expect_origin_distance(
        stem,
        base,
        axes="xy",
        max_dist=1e-6,
        name="upper stem stays centered over the base sleeve",
    )

    rest_pos = ctx.part_world_position(stem)
    with ctx.pose({stem_slide: stem_slide.motion_limits.upper}):
        ctx.expect_overlap(
            stem,
            base,
            axes="z",
            elem_a="inner_mast",
            elem_b="sleeve_front",
            min_overlap=0.18,
            name="telescoping mast retains insertion at full extension",
        )
        ctx.expect_origin_distance(
            stem,
            base,
            axes="xy",
            max_dist=1e-6,
            name="mast extension remains vertical and not offset sideways",
        )
        extended_pos = ctx.part_world_position(stem)
        ctx.check(
            "prismatic stem extends upward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] > rest_pos[2] + 0.18,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    rest_top_aabb = ctx.part_element_world_aabb(tray, elem="tray_top_flange")
    with ctx.pose({tray_tilt: tray_tilt.motion_limits.upper}):
        tilted_top_aabb = ctx.part_element_world_aabb(tray, elem="tray_top_flange")
    rest_top_center = aabb_center(rest_top_aabb)
    tilted_top_center = aabb_center(tilted_top_aabb)
    ctx.check(
        "tray tilts backward about the horizontal head axis",
        rest_top_center is not None
        and tilted_top_center is not None
        and tilted_top_center[1] < rest_top_center[1] - 0.10,
        details=f"rest_top={rest_top_center}, tilted_top={tilted_top_center}",
    )

    rest_lip_aabb = ctx.part_element_world_aabb(lip, elem="lip_fence")
    with ctx.pose({lip_hinge: lip_hinge.motion_limits.upper}):
        raised_lip_aabb = ctx.part_element_world_aabb(lip, elem="lip_fence")
    rest_lip_center = aabb_center(rest_lip_aabb)
    raised_lip_center = aabb_center(raised_lip_aabb)
    ctx.check(
        "retaining lip folds upward on its front hinge",
        rest_lip_center is not None
        and raised_lip_center is not None
        and raised_lip_center[2] > rest_lip_center[2] + 0.015,
        details=f"rest_lip={rest_lip_center}, raised_lip={raised_lip_center}",
    )

    ctx.expect_gap(
        lip,
        tray,
        axis="y",
        positive_elem="lip_shelf",
        negative_elem="tray_back",
        min_gap=0.0,
        max_gap=0.08,
        name="deployed retaining lip sits in front of the tray face",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
