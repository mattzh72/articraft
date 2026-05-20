from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/tmp")

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.292
BODY_DEPTH = 0.192
BODY_HEIGHT = 0.034
WALL_THICKNESS = 0.008
BOTTOM_THICKNESS = 0.004
TOP_PLATE_THICKNESS = 0.0035
TOP_SURFACE_Z = BODY_HEIGHT


def _translated_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_top_panel_mesh():
    outer_profile = rounded_rect_profile(
        BODY_WIDTH - 2.0 * WALL_THICKNESS,
        BODY_DEPTH - 2.0 * WALL_THICKNESS,
        0.008,
    )
    hole_profiles = [
        _translated_profile(rounded_rect_profile(0.012, 0.084, 0.003), -0.072, -0.024),
        _translated_profile(rounded_rect_profile(0.012, 0.084, 0.003), 0.072, -0.024),
        _translated_profile(rounded_rect_profile(0.142, 0.010, 0.0025), 0.000, -0.074),
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            hole_profiles,
            TOP_PLATE_THICKNESS,
            center=True,
            cap=True,
            closed=True,
        ),
        ASSETS.mesh_path("dj_mixer_top_panel.obj"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_mixer", assets=ASSETS)

    shell = model.material("shell", rgba=(0.11, 0.11, 0.12, 1.0))
    panel = model.material("panel", rgba=(0.15, 0.15, 0.17, 1.0))
    strip = model.material("strip", rgba=(0.07, 0.07, 0.08, 1.0))
    metal = model.material("metal", rgba=(0.72, 0.74, 0.76, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    accent = model.material("accent", rgba=(0.90, 0.58, 0.14, 1.0))
    cue_red = model.material("cue_red", rgba=(0.80, 0.18, 0.16, 1.0))

    body = model.part("body")
    side_height = BODY_HEIGHT - BOTTOM_THICKNESS - TOP_PLATE_THICKNESS
    side_z = BOTTOM_THICKNESS + side_height * 0.5

    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS * 0.5)),
        material=shell,
        name="bottom_plate",
    )
    body.visual(
        Box((WALL_THICKNESS, BODY_DEPTH, side_height)),
        origin=Origin(xyz=(-(BODY_WIDTH - WALL_THICKNESS) * 0.5, 0.0, side_z)),
        material=shell,
        name="left_wall",
    )
    body.visual(
        Box((WALL_THICKNESS, BODY_DEPTH, side_height)),
        origin=Origin(xyz=((BODY_WIDTH - WALL_THICKNESS) * 0.5, 0.0, side_z)),
        material=shell,
        name="right_wall",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, side_height)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH - WALL_THICKNESS) * 0.5, side_z)),
        material=shell,
        name="front_wall",
    )
    body.visual(
        Box((BODY_WIDTH - 2.0 * WALL_THICKNESS, WALL_THICKNESS, side_height)),
        origin=Origin(xyz=(0.0, (BODY_DEPTH - WALL_THICKNESS) * 0.5, side_z)),
        material=shell,
        name="rear_wall",
    )
    body.visual(
        _build_top_panel_mesh(),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - TOP_PLATE_THICKNESS * 0.5)),
        material=panel,
        name="top_panel",
    )
    body.visual(
        Box((0.064, 0.116, 0.0008)),
        origin=Origin(xyz=(-0.072, 0.020, TOP_SURFACE_Z + 0.0004)),
        material=strip,
        name="left_channel_strip",
    )
    body.visual(
        Box((0.064, 0.116, 0.0008)),
        origin=Origin(xyz=(0.072, 0.020, TOP_SURFACE_Z + 0.0004)),
        material=strip,
        name="right_channel_strip",
    )
    body.visual(
        Box((0.046, 0.056, 0.0008)),
        origin=Origin(xyz=(0.108, 0.052, TOP_SURFACE_Z + 0.0004)),
        material=strip,
        name="cue_section_plate",
    )
    body.visual(
        Box((0.022, 0.020, 0.007)),
        origin=Origin(xyz=(0.112, 0.056, TOP_SURFACE_Z + 0.0035)),
        material=metal,
        name="cue_pedestal",
    )
    body.visual(
        Box((0.150, 0.022, 0.0008)),
        origin=Origin(xyz=(0.0, -0.074, TOP_SURFACE_Z + 0.0004)),
        material=strip,
        name="crossfader_plate",
    )
    body.visual(
        Box((0.064, 0.088, 0.0008)),
        origin=Origin(xyz=(-0.072, -0.004, TOP_SURFACE_Z + 0.0004)),
        material=strip,
        name="left_fader_gate",
    )
    body.visual(
        Box((0.064, 0.088, 0.0008)),
        origin=Origin(xyz=(0.072, -0.004, TOP_SURFACE_Z + 0.0004)),
        material=strip,
        name="right_fader_gate",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    def add_channel_knob(
        part_name: str,
        joint_name: str,
        position: tuple[float, float, float],
    ):
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=0.009, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=knob_dark,
            name="knob_skirt",
        )
        knob.visual(
            Cylinder(radius=0.0065, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
            material=metal,
            name="knob_cap",
        )
        knob.visual(
            Box((0.0025, 0.010, 0.0015)),
            origin=Origin(xyz=(0.0, 0.0045, 0.01525)),
            material=accent,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.020, 0.020, 0.019)),
            mass=0.04,
            origin=Origin(xyz=(0.0, 0.0, 0.0095)),
        )
        articulation = model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=position),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=5.0,
                lower=-2.5,
                upper=2.5,
            ),
        )
        return knob, articulation

    for part_name, joint_name, position in [
        ("left_trim_knob", "left_trim_turn", (-0.072, 0.076, TOP_SURFACE_Z)),
        ("left_hi_knob", "left_hi_turn", (-0.072, 0.050, TOP_SURFACE_Z)),
        ("left_low_knob", "left_low_turn", (-0.072, 0.026, TOP_SURFACE_Z)),
        ("right_trim_knob", "right_trim_turn", (0.072, 0.076, TOP_SURFACE_Z)),
        ("right_hi_knob", "right_hi_turn", (0.072, 0.050, TOP_SURFACE_Z)),
        ("right_low_knob", "right_low_turn", (0.072, 0.026, TOP_SURFACE_Z)),
    ]:
        add_channel_knob(part_name, joint_name, position)

    def add_channel_fader(
        part_name: str,
        joint_name: str,
        x_pos: float,
    ):
        fader = model.part(part_name)
        fader.visual(
            Box((0.014, 0.022, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=metal,
            name="fader_cap",
        )
        fader.visual(
            Box((0.0045, 0.010, 0.022)),
            origin=Origin(xyz=(0.0, 0.0, -0.011)),
            material=accent,
            name="fader_stem",
        )
        fader.visual(
            Box((0.010, 0.012, 0.003)),
            origin=Origin(xyz=(0.0, 0.0, 0.0115)),
            material=knob_dark,
            name="fader_grip",
        )
        fader.inertial = Inertial.from_geometry(
            Box((0.016, 0.024, 0.034)),
            mass=0.035,
            origin=Origin(xyz=(0.0, 0.0, -0.001)),
        )
        articulation = model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=fader,
            origin=Origin(xyz=(x_pos, -0.024, TOP_SURFACE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.18,
                lower=-0.026,
                upper=0.024,
            ),
        )
        return fader, articulation

    add_channel_fader("left_channel_fader", "left_channel_slide", -0.072)
    add_channel_fader("right_channel_fader", "right_channel_slide", 0.072)

    crossfader = model.part("crossfader")
    crossfader.visual(
        Box((0.020, 0.034, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=metal,
        name="fader_cap",
    )
    crossfader.visual(
        Box((0.006, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=accent,
        name="fader_stem",
    )
    crossfader.visual(
        Box((0.012, 0.016, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0135)),
        material=knob_dark,
        name="fader_grip",
    )
    crossfader.inertial = Inertial.from_geometry(
        Box((0.022, 0.036, 0.036)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
    )
    model.articulation(
        "crossfader_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=crossfader,
        origin=Origin(xyz=(0.0, -0.074, TOP_SURFACE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.4,
            velocity=0.24,
            lower=-0.058,
            upper=0.058,
        ),
    )

    cue_lever = model.part("cue_lever")
    cue_lever.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="pivot_barrel",
    )
    cue_lever.visual(
        Box((0.006, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, 0.012, 0.007)),
        material=metal,
        name="cue_arm",
    )
    cue_lever.visual(
        Box((0.012, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.028, 0.015)),
        material=cue_red,
        name="cue_tip",
    )
    cue_lever.inertial = Inertial.from_geometry(
        Box((0.018, 0.046, 0.024)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.018, 0.010)),
    )
    model.articulation(
        "cue_lever_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cue_lever,
        origin=Origin(xyz=(0.112, 0.056, TOP_SURFACE_Z + 0.011)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.0,
            lower=-0.25,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)
    body = object_model.get_part("body")
    top_panel = body.get_visual("top_panel")
    cue_pedestal = body.get_visual("cue_pedestal")

    left_channel_fader = object_model.get_part("left_channel_fader")
    right_channel_fader = object_model.get_part("right_channel_fader")
    crossfader = object_model.get_part("crossfader")
    cue_lever = object_model.get_part("cue_lever")

    left_channel_slide = object_model.get_articulation("left_channel_slide")
    right_channel_slide = object_model.get_articulation("right_channel_slide")
    crossfader_slide = object_model.get_articulation("crossfader_slide")
    cue_lever_hinge = object_model.get_articulation("cue_lever_hinge")

    knob_data = [
        (object_model.get_part("left_trim_knob"), object_model.get_articulation("left_trim_turn")),
        (object_model.get_part("left_hi_knob"), object_model.get_articulation("left_hi_turn")),
        (object_model.get_part("left_low_knob"), object_model.get_articulation("left_low_turn")),
        (object_model.get_part("right_trim_knob"), object_model.get_articulation("right_trim_turn")),
        (object_model.get_part("right_hi_knob"), object_model.get_articulation("right_hi_turn")),
        (object_model.get_part("right_low_knob"), object_model.get_articulation("right_low_turn")),
    ]

    left_fader_cap = left_channel_fader.get_visual("fader_cap")
    right_fader_cap = right_channel_fader.get_visual("fader_cap")
    crossfader_cap = crossfader.get_visual("fader_cap")
    cue_barrel = cue_lever.get_visual("pivot_barrel")
    cue_tip = cue_lever.get_visual("cue_tip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("body_present", "Body AABB could not be resolved.")
    else:
        body_size = tuple(body_aabb[1][axis] - body_aabb[0][axis] for axis in range(3))
        ctx.check(
            "mixer_body_realistic_size",
            0.26 <= body_size[0] <= 0.32 and 0.18 <= body_size[1] <= 0.22 and 0.03 <= body_size[2] <= 0.05,
            f"Unexpected body dimensions {body_size!r}.",
        )

    ctx.check(
        "channel_knob_count",
        len(knob_data) == 6,
        f"Expected 6 rotary channel knobs, found {len(knob_data)}.",
    )

    for knob_part, knob_joint in knob_data:
        knob_skirt = knob_part.get_visual("knob_skirt")
        ctx.check(
            f"{knob_joint.name}_joint_type",
            knob_joint.joint_type == ArticulationType.REVOLUTE,
            f"{knob_joint.name} should be revolute.",
        )
        ctx.check(
            f"{knob_joint.name}_joint_axis",
            tuple(knob_joint.axis) == (0.0, 0.0, 1.0),
            f"{knob_joint.name} axis should be +Z, got {knob_joint.axis!r}.",
        )
        ctx.expect_gap(
            knob_part,
            body,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=knob_skirt,
            negative_elem=top_panel,
            name=f"{knob_part.name}_seated_on_panel",
        )
        ctx.expect_within(
            knob_part,
            body,
            axes="xy",
            margin=0.002,
            name=f"{knob_part.name}_within_mixer_footprint",
        )

    for knob_part, knob_joint in knob_data:
        indicator = knob_part.get_visual("indicator")
        indicator_rest_aabb = ctx.part_element_world_aabb(knob_part, elem=indicator)
        if indicator_rest_aabb is None:
            ctx.fail(f"{knob_joint.name}_indicator_rest", "Indicator AABB missing at rest pose.")
            continue
        limits = knob_joint.motion_limits
        if limits is None or limits.upper is None:
            ctx.fail(f"{knob_joint.name}_limits", "Knob joint limits are missing.")
            continue
        with ctx.pose({knob_joint: limits.upper * 0.5}):
            indicator_turned_aabb = ctx.part_element_world_aabb(knob_part, elem=indicator)
            if indicator_turned_aabb is None:
                ctx.fail(f"{knob_joint.name}_indicator_turned", "Indicator AABB missing in turned pose.")
                continue
            rest_center = _aabb_center(indicator_rest_aabb)
            turned_center = _aabb_center(indicator_turned_aabb)
            dx = turned_center[0] - rest_center[0]
            dy = turned_center[1] - rest_center[1]
            ctx.check(
                f"{knob_joint.name}_rotates_indicator",
                math.hypot(dx, dy) >= 0.003,
                f"{knob_joint.name} indicator moved only {(dx, dy)!r}.",
            )
            ctx.expect_gap(
                knob_part,
                body,
                axis="z",
                max_gap=0.0005,
                max_penetration=0.0,
                positive_elem=knob_part.get_visual("knob_skirt"),
                negative_elem=top_panel,
                name=f"{knob_joint.name}_turned_pose_stays_seated",
            )

    for fader_part, fader_joint, cap_visual in [
        (left_channel_fader, left_channel_slide, left_fader_cap),
        (right_channel_fader, right_channel_slide, right_fader_cap),
    ]:
        ctx.check(
            f"{fader_joint.name}_joint_type",
            fader_joint.joint_type == ArticulationType.PRISMATIC,
            f"{fader_joint.name} should be prismatic.",
        )
        ctx.check(
            f"{fader_joint.name}_joint_axis",
            tuple(fader_joint.axis) == (0.0, 1.0, 0.0),
            f"{fader_joint.name} axis should be +Y, got {fader_joint.axis!r}.",
        )
        ctx.expect_gap(
            fader_part,
            body,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem=cap_visual,
            negative_elem=top_panel,
            name=f"{fader_part.name}_cap_seated",
        )
        rest_position = ctx.part_world_position(fader_part)
        limits = fader_joint.motion_limits
        if rest_position is None or limits is None or limits.lower is None or limits.upper is None:
            ctx.fail(f"{fader_joint.name}_pose_data", "Fader pose data missing.")
            continue
        with ctx.pose({fader_joint: limits.lower}):
            low_position = ctx.part_world_position(fader_part)
            ctx.expect_gap(
                fader_part,
                body,
                axis="z",
                max_gap=0.0005,
                max_penetration=0.0,
                positive_elem=cap_visual,
                negative_elem=top_panel,
                name=f"{fader_joint.name}_lower_seated",
            )
        with ctx.pose({fader_joint: limits.upper}):
            high_position = ctx.part_world_position(fader_part)
            ctx.expect_gap(
                fader_part,
                body,
                axis="z",
                max_gap=0.0005,
                max_penetration=0.0,
                positive_elem=cap_visual,
                negative_elem=top_panel,
                name=f"{fader_joint.name}_upper_seated",
            )
        if low_position is None or high_position is None:
            ctx.fail(f"{fader_joint.name}_positions", "Could not resolve fader positions at limits.")
            continue
        ctx.check(
            f"{fader_joint.name}_slides_along_y",
            abs(high_position[1] - low_position[1]) >= 0.05
            and abs(high_position[0] - rest_position[0]) <= 1e-6
            and abs(high_position[2] - rest_position[2]) <= 1e-6,
            f"{fader_joint.name} positions were rest={rest_position!r}, low={low_position!r}, high={high_position!r}.",
        )

    ctx.check(
        "crossfader_joint_type",
        crossfader_slide.joint_type == ArticulationType.PRISMATIC,
        "Crossfader should slide on a prismatic joint.",
    )
    ctx.check(
        "crossfader_joint_axis",
        tuple(crossfader_slide.axis) == (1.0, 0.0, 0.0),
        f"Crossfader axis should be +X, got {crossfader_slide.axis!r}.",
    )
    ctx.expect_gap(
        crossfader,
        body,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=crossfader_cap,
        negative_elem=top_panel,
        name="crossfader_cap_seated",
    )
    crossfader_rest = ctx.part_world_position(crossfader)
    cross_limits = crossfader_slide.motion_limits
    if crossfader_rest is None or cross_limits is None or cross_limits.lower is None or cross_limits.upper is None:
        ctx.fail("crossfader_pose_data", "Crossfader pose data missing.")
    else:
        with ctx.pose({crossfader_slide: cross_limits.lower}):
            crossfader_left = ctx.part_world_position(crossfader)
            ctx.expect_gap(
                crossfader,
                body,
                axis="z",
                max_gap=0.0005,
                max_penetration=0.0,
                positive_elem=crossfader_cap,
                negative_elem=top_panel,
                name="crossfader_left_limit_seated",
            )
        with ctx.pose({crossfader_slide: cross_limits.upper}):
            crossfader_right = ctx.part_world_position(crossfader)
            ctx.expect_gap(
                crossfader,
                body,
                axis="z",
                max_gap=0.0005,
                max_penetration=0.0,
                positive_elem=crossfader_cap,
                negative_elem=top_panel,
                name="crossfader_right_limit_seated",
            )
        if crossfader_left is None or crossfader_right is None:
            ctx.fail("crossfader_positions", "Could not resolve crossfader limit positions.")
        else:
            ctx.check(
                "crossfader_moves_horizontally",
                abs(crossfader_right[0] - crossfader_left[0]) >= 0.10
                and abs(crossfader_right[1] - crossfader_rest[1]) <= 1e-6
                and abs(crossfader_right[2] - crossfader_rest[2]) <= 1e-6,
                f"Crossfader positions were rest={crossfader_rest!r}, left={crossfader_left!r}, right={crossfader_right!r}.",
            )

    ctx.check(
        "cue_lever_joint_type",
        cue_lever_hinge.joint_type == ArticulationType.REVOLUTE,
        "Cue lever should be revolute.",
    )
    ctx.check(
        "cue_lever_joint_axis",
        tuple(cue_lever_hinge.axis) == (1.0, 0.0, 0.0),
        f"Cue lever axis should be +X, got {cue_lever_hinge.axis!r}.",
    )
    ctx.expect_gap(
        cue_lever,
        body,
        axis="z",
        max_gap=0.0005,
        max_penetration=1e-5,
        positive_elem=cue_barrel,
        negative_elem=cue_pedestal,
        name="cue_barrel_seated_on_pedestal",
    )
    cue_tip_rest_aabb = ctx.part_element_world_aabb(cue_lever, elem=cue_tip)
    cue_limits = cue_lever_hinge.motion_limits
    if cue_tip_rest_aabb is None or cue_limits is None or cue_limits.upper is None:
        ctx.fail("cue_lever_pose_data", "Cue lever pose data missing.")
    else:
        with ctx.pose({cue_lever_hinge: cue_limits.upper}):
            cue_tip_up_aabb = ctx.part_element_world_aabb(cue_lever, elem=cue_tip)
            ctx.expect_gap(
                cue_lever,
                body,
                axis="z",
                min_gap=0.014,
                positive_elem=cue_tip,
                negative_elem=top_panel,
                name="cue_tip_lifts_clear_of_panel",
            )
        if cue_tip_up_aabb is None:
            ctx.fail("cue_tip_up_pose", "Cue tip AABB missing at raised pose.")
        else:
            rest_center = _aabb_center(cue_tip_rest_aabb)
            up_center = _aabb_center(cue_tip_up_aabb)
            ctx.check(
                "cue_lever_raises_tip",
                up_center[2] >= rest_center[2] + 0.010,
                f"Cue tip centers were rest={rest_center!r}, raised={up_center!r}.",
            )

    bounded_joints = [joint for _, joint in knob_data] + [
        left_channel_slide,
        right_channel_slide,
        crossfader_slide,
        cue_lever_hinge,
    ]
    for articulated_joint in bounded_joints:
        limits = articulated_joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            ctx.fail(f"{articulated_joint.name}_bounded_limits", "Expected finite articulation limits.")
            continue
        with ctx.pose({articulated_joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulated_joint.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulated_joint.name}_lower_no_floating")
        with ctx.pose({articulated_joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{articulated_joint.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{articulated_joint.name}_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
