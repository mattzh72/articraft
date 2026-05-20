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
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orchestra_music_stand")

    powder_black = model.material("powder_black", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    outer_tube_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.022, -0.310), (0.022, 0.310)],
            [(0.0185, -0.310), (0.0185, 0.310)],
            segments=56,
        ),
        "outer_tube_shell",
    )
    clamp_collar_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(0.028, -0.020), (0.028, 0.020)],
            [(0.0205, -0.020), (0.0205, 0.020)],
            segments=56,
        ),
        "clamp_collar_shell",
    )
    base.visual(
        Cylinder(radius=0.19, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=powder_black,
        name="weight_disc",
    )
    base.visual(
        Cylinder(radius=0.155, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=graphite,
        name="top_cover",
    )
    base.visual(
        Cylinder(radius=0.145, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=rubber,
        name="rubber_pad",
    )
    base.visual(
        Cylinder(radius=0.036, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=powder_black,
        name="column_socket",
    )
    base.visual(
        outer_tube_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.408)),
        material=powder_black,
        name="outer_tube",
    )
    base.visual(
        clamp_collar_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.715)),
        material=powder_black,
        name="clamp_collar",
    )
    base.visual(
        Box((0.034, 0.018, 0.030)),
        origin=Origin(xyz=(0.036, 0.0, 0.715)),
        material=powder_black,
        name="clamp_block",
    )
    base.visual(
        Box((0.010, 0.010, 0.020)),
        origin=Origin(xyz=(0.023, 0.0, 0.715)),
        material=satin_steel,
        name="clamp_pad",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(xyz=(0.056, 0.0, 0.715), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="clamp_knob",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.40, 0.40, 0.74)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
    )

    inner_mast = model.part("inner_mast")
    inner_mast.visual(
        Cylinder(radius=0.018, length=0.920),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=graphite,
        name="inner_tube",
    )
    inner_mast.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
        material=powder_black,
        name="head_collar",
    )
    inner_mast.visual(
        Box((0.078, 0.035, 0.084)),
        origin=Origin(xyz=(0.0, 0.0, 0.606)),
        material=powder_black,
        name="head_block",
    )
    inner_mast.visual(
        Box((0.094, 0.026, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.689)),
        material=powder_black,
        name="yoke_bridge",
    )
    inner_mast.visual(
        Box((0.016, 0.028, 0.114)),
        origin=Origin(xyz=(-0.041, 0.0, 0.663)),
        material=powder_black,
        name="left_yoke_arm",
    )
    inner_mast.visual(
        Box((0.016, 0.028, 0.114)),
        origin=Origin(xyz=(0.041, 0.0, 0.663)),
        material=powder_black,
        name="right_yoke_arm",
    )
    inner_mast.inertial = Inertial.from_geometry(
        Box((0.12, 0.08, 0.78)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
    )

    model.articulation(
        "mast_extension",
        ArticulationType.PRISMATIC,
        parent=base,
        child=inner_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.710)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.240,
        ),
    )

    desk = model.part("desk")
    rest_tilt = 0.28
    desk.visual(
        Cylinder(radius=0.010, length=0.062),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="pivot_rod",
    )
    desk.visual(
        Box((0.040, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, 0.020, 0.010)),
        material=powder_black,
        name="rod_bracket",
    )
    desk.visual(
        Box((0.060, 0.028, 0.055)),
        origin=Origin(xyz=(0.0, 0.036, 0.032), rpy=(rest_tilt, 0.0, 0.0)),
        material=powder_black,
        name="pivot_block",
    )
    desk.visual(
        Box((0.560, 0.0022, 0.360)),
        origin=Origin(xyz=(0.0, 0.015, 0.180), rpy=(rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="back_panel",
    )
    desk.visual(
        Box((0.014, 0.080, 0.330)),
        origin=Origin(xyz=(-0.273, 0.055, 0.165), rpy=(rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="left_side_flange",
    )
    desk.visual(
        Box((0.014, 0.080, 0.330)),
        origin=Origin(xyz=(0.273, 0.055, 0.165), rpy=(rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="right_side_flange",
    )
    desk.visual(
        Box((0.538, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, 0.018, 0.339), rpy=(rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="top_hem",
    )
    desk.visual(
        Box((0.534, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.015, 0.330), rpy=(rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="top_return_web",
    )
    desk.visual(
        Box((0.540, 0.090, 0.018)),
        origin=Origin(xyz=(0.0, 0.066, 0.020), rpy=(rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="lower_shelf",
    )
    desk.visual(
        Box((0.518, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.107, 0.031), rpy=(rest_tilt, 0.0, 0.0)),
        material=graphite,
        name="retaining_lip",
    )
    desk.visual(
        Box((0.150, 0.034, 0.210)),
        origin=Origin(xyz=(0.0, 0.022, 0.150), rpy=(rest_tilt, 0.0, 0.0)),
        material=powder_black,
        name="center_rib",
    )
    desk.visual(
        Box((0.018, 0.120, 0.012)),
        origin=Origin(xyz=(-0.150, 0.075, 0.006), rpy=(rest_tilt, 0.0, 0.0)),
        material=powder_black,
        name="left_runner_track",
    )
    desk.visual(
        Box((0.018, 0.120, 0.012)),
        origin=Origin(xyz=(0.150, 0.075, 0.006), rpy=(rest_tilt, 0.0, 0.0)),
        material=powder_black,
        name="right_runner_track",
    )
    desk.inertial = Inertial.from_geometry(
        Box((0.58, 0.16, 0.42)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.060, 0.180)),
    )

    model.articulation(
        "desk_tilt",
        ArticulationType.REVOLUTE,
        parent=inner_mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.663)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-0.35,
            upper=0.55,
        ),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.320, 0.108, 0.003)),
        origin=Origin(xyz=(0.0, -0.006, -0.025)),
        material=graphite,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((0.008, 0.108, 0.026)),
        origin=Origin(xyz=(-0.156, -0.006, -0.012)),
        material=graphite,
        name="left_drawer_wall",
    )
    drawer.visual(
        Box((0.008, 0.108, 0.026)),
        origin=Origin(xyz=(0.156, -0.006, -0.012)),
        material=graphite,
        name="right_drawer_wall",
    )
    drawer.visual(
        Box((0.320, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, -0.056, -0.013)),
        material=graphite,
        name="rear_drawer_wall",
    )
    drawer.visual(
        Box((0.340, 0.008, 0.032)),
        origin=Origin(xyz=(0.0, 0.056, -0.010)),
        material=powder_black,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.180, 0.012, 0.007)),
        origin=Origin(xyz=(0.0, 0.063, 0.001)),
        material=satin_steel,
        name="pull_lip",
    )
    drawer.visual(
        Box((0.012, 0.120, 0.006)),
        origin=Origin(xyz=(-0.150, 0.0, 0.003)),
        material=powder_black,
        name="left_slide_rail",
    )
    drawer.visual(
        Box((0.012, 0.120, 0.006)),
        origin=Origin(xyz=(0.150, 0.0, 0.003)),
        material=powder_black,
        name="right_slide_rail",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.35, 0.14, 0.05)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=desk,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.075, -0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.18,
            lower=0.0,
            upper=0.090,
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

    base = object_model.get_part("base")
    inner_mast = object_model.get_part("inner_mast")
    desk = object_model.get_part("desk")
    drawer = object_model.get_part("drawer")

    mast_extension = object_model.get_articulation("mast_extension")
    desk_tilt = object_model.get_articulation("desk_tilt")
    drawer_slide = object_model.get_articulation("drawer_slide")

    outer_tube = base.get_visual("outer_tube")
    inner_tube = inner_mast.get_visual("inner_tube")
    back_panel = desk.get_visual("back_panel")
    runner_track = desk.get_visual("left_runner_track")
    slide_rail = drawer.get_visual("left_slide_rail")

    ctx.check(
        "all prompt-critical parts exist",
        all(part is not None for part in (base, inner_mast, desk, drawer)),
    )
    ctx.check(
        "all prompt-critical articulations exist",
        all(joint is not None for joint in (mast_extension, desk_tilt, drawer_slide)),
    )

    ctx.expect_within(
        inner_mast,
        base,
        axes="xy",
        inner_elem=inner_tube,
        outer_elem=outer_tube,
        margin=0.004,
        name="inner mast stays centered in lower tube at rest",
    )
    ctx.expect_overlap(
        inner_mast,
        base,
        axes="z",
        elem_a=inner_tube,
        elem_b=outer_tube,
        min_overlap=0.12,
        name="inner mast remains inserted in lower tube at rest",
    )

    desk_rest_pos = ctx.part_world_position(desk)
    with ctx.pose({mast_extension: mast_extension.motion_limits.upper}):
        ctx.expect_within(
            inner_mast,
            base,
            axes="xy",
            inner_elem=inner_tube,
            outer_elem=outer_tube,
            margin=0.004,
            name="inner mast stays centered in lower tube when extended",
        )
        ctx.expect_overlap(
            inner_mast,
            base,
            axes="z",
            elem_a=inner_tube,
            elem_b=outer_tube,
            min_overlap=0.10,
            name="inner mast retains insertion when extended",
        )
        desk_extended_pos = ctx.part_world_position(desk)

    ctx.check(
        "mast extension raises the desk",
        desk_rest_pos is not None
        and desk_extended_pos is not None
        and desk_extended_pos[2] > desk_rest_pos[2] + 0.18,
        details=f"rest={desk_rest_pos}, extended={desk_extended_pos}",
    )

    panel_rest = ctx.part_element_world_aabb(desk, elem=back_panel)
    with ctx.pose({desk_tilt: desk_tilt.motion_limits.upper}):
        panel_tilted = ctx.part_element_world_aabb(desk, elem=back_panel)

    panel_rest_center_y = None
    panel_tilted_center_y = None
    if panel_rest is not None:
        panel_rest_center_y = (panel_rest[0][1] + panel_rest[1][1]) * 0.5
    if panel_tilted is not None:
        panel_tilted_center_y = (panel_tilted[0][1] + panel_tilted[1][1]) * 0.5
    ctx.check(
        "desk tilt rotates the panel rearward",
        panel_rest_center_y is not None
        and panel_tilted_center_y is not None
        and panel_tilted_center_y < panel_rest_center_y - 0.09,
        details=f"rest_center_y={panel_rest_center_y}, tilted_center_y={panel_tilted_center_y}",
    )

    drawer_rest_pos = ctx.part_world_position(drawer)
    ctx.expect_overlap(
        drawer,
        desk,
        axes="y",
        elem_a=slide_rail,
        elem_b=runner_track,
        min_overlap=0.10,
        name="drawer runners overlap deeply when closed",
    )
    with ctx.pose({drawer_slide: drawer_slide.motion_limits.upper}):
        ctx.expect_overlap(
            drawer,
            desk,
            axes="y",
            elem_a=slide_rail,
            elem_b=runner_track,
            min_overlap=0.025,
            name="drawer runners retain engagement when extended",
        )
        drawer_extended_pos = ctx.part_world_position(drawer)

    ctx.check(
        "drawer slides outward",
        drawer_rest_pos is not None
        and drawer_extended_pos is not None
        and drawer_extended_pos[1] > drawer_rest_pos[1] + 0.05,
        details=f"rest={drawer_rest_pos}, extended={drawer_extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
