from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_full_motion_wall_mount")

    black = model.material("black_powder_coat", rgba=(0.015, 0.017, 0.018, 1.0))
    dark = model.material("dark_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    screw = model.material("blackened_screw_heads", rgba=(0.005, 0.005, 0.006, 1.0))
    wall = model.material("matte_wall_plate", rgba=(0.18, 0.19, 0.20, 1.0))

    # Object frame: +X comes out from the wall, +Y runs along the wall,
    # and +Z is up.  The rest pose is the compact folded pose.
    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.018, 0.200, 0.320)),
        origin=Origin(xyz=(0.009, 0.0, 0.160)),
        material=wall,
        name="wall_plate",
    )
    wall_bracket.visual(
        Box((0.016, 0.060, 0.145)),
        origin=Origin(xyz=(0.008, -0.070, 0.160)),
        material=dark,
        name="pivot_standoff",
    )
    wall_bracket.visual(
        Cylinder(radius=0.020, length=0.085),
        origin=Origin(xyz=(0.036, -0.070, 0.160)),
        material=black,
        name="wall_pivot_barrel",
    )
    for i, (yy, zz) in enumerate(
        ((-0.060, 0.265), (0.060, 0.265), (-0.060, 0.055), (0.060, 0.055))
    ):
        wall_bracket.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(0.0195, yy, zz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=screw,
            name=f"wall_screw_{i}",
        )

    link_0 = model.part("link_0")
    first_len = 0.280
    link_0.visual(
        Cylinder(radius=0.019, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="proximal_bearing",
    )
    link_0.visual(
        Box((0.018, 0.254, 0.030)),
        origin=Origin(xyz=(0.0, 0.146, 0.0)),
        material=dark,
        name="center_web",
    )
    link_0.visual(
        Box((0.026, 0.240, 0.018)),
        origin=Origin(xyz=(0.0, 0.160, 0.065)),
        material=black,
        name="upper_arm",
    )
    link_0.visual(
        Box((0.026, 0.240, 0.018)),
        origin=Origin(xyz=(0.0, 0.160, -0.065)),
        material=black,
        name="lower_arm",
    )
    link_0.visual(
        Cylinder(radius=0.014, length=0.146),
        origin=Origin(xyz=(0.0, first_len, 0.0)),
        material=black,
        name="elbow_spacer",
    )
    link_0.visual(
        Box((0.042, 0.040, 0.035)),
        origin=Origin(xyz=(0.021, first_len, 0.0)),
        material=dark,
        name="second_pivot_lug",
    )

    link_1 = model.part("link_1")
    second_len = 0.240
    link_1.visual(
        Box((0.026, 0.240, 0.034)),
        origin=Origin(xyz=(0.0, -0.120, 0.0)),
        material=black,
        name="arm_bar",
    )
    link_1.visual(
        Cylinder(radius=0.017, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="proximal_barrel",
    )
    link_1.visual(
        Cylinder(radius=0.020, length=0.058),
        origin=Origin(xyz=(0.0, -second_len, 0.0)),
        material=black,
        name="distal_barrel",
    )

    head_yoke = model.part("head_yoke")
    pitch_x = 0.048
    head_yoke.visual(
        Cylinder(radius=0.014, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark,
        name="pan_barrel",
    )
    head_yoke.visual(
        Box((0.030, 0.110, 0.030)),
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
        material=dark,
        name="yoke_neck",
    )
    for side, yy in (("upper", 0.052), ("lower", -0.052)):
        head_yoke.visual(
            Box((0.020, 0.018, 0.030)),
            origin=Origin(xyz=(0.040, yy, 0.0)),
            material=dark,
            name=f"{side}_pitch_arm",
        )
    for side, yy in (("upper", 0.052), ("lower", -0.052)):
        head_yoke.visual(
            Box((0.028, 0.018, 0.070)),
            origin=Origin(xyz=(pitch_x, yy, 0.0)),
            material=dark,
            name=f"{side}_pitch_cheek",
        )

    vesa_frame = model.part("vesa_frame")
    outer_w = 0.280
    outer_h = 0.200
    rail = 0.026
    frame_x = 0.050
    frame_depth = 0.018
    vesa_frame.visual(
        Cylinder(radius=0.016, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="pitch_tube",
    )
    vesa_frame.visual(
        Box((0.050, 0.046, 0.036)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=black,
        name="center_bridge",
    )
    vesa_frame.visual(
        Box((frame_depth, 0.030, outer_h)),
        origin=Origin(xyz=(frame_x, 0.0, 0.0)),
        material=black,
        name="center_spine",
    )
    for side, yy in (("side_0", -(outer_w / 2.0 - rail / 2.0)), ("side_1", outer_w / 2.0 - rail / 2.0)):
        vesa_frame.visual(
            Box((frame_depth, rail, outer_h)),
            origin=Origin(xyz=(frame_x, yy, 0.0)),
            material=black,
            name=f"{side}_rail",
        )
    for side, zz in (("top", outer_h / 2.0 - rail / 2.0), ("bottom", -(outer_h / 2.0 - rail / 2.0))):
        vesa_frame.visual(
            Box((frame_depth, outer_w, rail)),
            origin=Origin(xyz=(frame_x, 0.0, zz)),
            material=black,
            name=f"{side}_rail",
        )
    for i, (yy, zz) in enumerate(((-0.112, -0.058), (0.112, -0.058), (-0.112, 0.058), (0.112, 0.058))):
        vesa_frame.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(frame_x + frame_depth / 2.0 + 0.003, yy, zz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=screw,
            name=f"vesa_screw_{i}",
        )

    model.articulation(
        "wall_pivot",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=link_0,
        origin=Origin(xyz=(0.036, -0.070, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-1.45, upper=0.75),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.035, first_len, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=0.0, upper=2.55),
    )
    model.articulation(
        "head_pan",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=head_yoke,
        origin=Origin(xyz=(0.0, -second_len, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.90, upper=0.90),
    )
    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=head_yoke,
        child=vesa_frame,
        origin=Origin(xyz=(pitch_x, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.35, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    head_yoke = object_model.get_part("head_yoke")
    vesa_frame = object_model.get_part("vesa_frame")

    wall_pivot = object_model.get_articulation("wall_pivot")
    elbow_pivot = object_model.get_articulation("elbow_pivot")
    head_pan = object_model.get_articulation("head_pan")
    head_pitch = object_model.get_articulation("head_pitch")

    ctx.allow_overlap(
        "wall_bracket",
        link_0,
        elem_a="wall_pivot_barrel",
        elem_b="proximal_bearing",
        reason="The first link bearing is intentionally captured around the wall pivot barrel.",
    )
    ctx.expect_overlap(
        "wall_bracket",
        link_0,
        axes="xy",
        elem_a="wall_pivot_barrel",
        elem_b="proximal_bearing",
        min_overlap=0.015,
        name="wall pivot bearing is coaxially captured",
    )
    ctx.expect_gap(
        link_0,
        "wall_bracket",
        axis="z",
        positive_elem="proximal_bearing",
        negative_elem="wall_pivot_barrel",
        max_penetration=0.060,
        name="wall pivot overlap is confined to the bearing height",
    )

    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="second_pivot_lug",
        elem_b="proximal_barrel",
        reason="The elbow lug and barrel intentionally share the captured pivot stack.",
    )
    ctx.expect_overlap(
        link_0,
        link_1,
        axes="xy",
        elem_a="second_pivot_lug",
        elem_b="proximal_barrel",
        min_overlap=0.010,
        name="elbow pivot parts are coaxially captured",
    )
    ctx.expect_gap(
        link_1,
        link_0,
        axis="z",
        positive_elem="proximal_barrel",
        negative_elem="second_pivot_lug",
        max_penetration=0.050,
        name="elbow pivot overlap is local through the stack",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="second_pivot_lug",
        elem_b="arm_bar",
        reason="The arm end locally nests into the elbow lug around the shared pivot.",
    )
    ctx.expect_overlap(
        link_0,
        link_1,
        axes="xy",
        elem_a="second_pivot_lug",
        elem_b="arm_bar",
        min_overlap=0.010,
        name="arm end remains seated in the elbow lug",
    )

    ctx.allow_overlap(
        link_1,
        head_yoke,
        elem_a="distal_barrel",
        elem_b="pan_barrel",
        reason="The pan barrel is intentionally nested inside the end-link pivot sleeve.",
    )
    ctx.allow_overlap(
        link_1,
        head_yoke,
        elem_a="arm_bar",
        elem_b="pan_barrel",
        reason="The end of the folded arm locally wraps the captured pan pivot barrel.",
    )
    ctx.allow_overlap(
        link_1,
        head_yoke,
        elem_a="distal_barrel",
        elem_b="yoke_neck",
        reason="The yoke neck locally intersects the captured end sleeve at the pan axis.",
    )
    ctx.allow_overlap(
        link_1,
        head_yoke,
        elem_a="arm_bar",
        elem_b="yoke_neck",
        reason="The folded arm end locally enters the same compact pan-yoke stack.",
    )
    ctx.expect_overlap(
        link_1,
        head_yoke,
        axes="xy",
        elem_a="distal_barrel",
        elem_b="pan_barrel",
        min_overlap=0.010,
        name="head pan is retained by the end sleeve",
    )
    ctx.expect_gap(
        head_yoke,
        link_1,
        axis="z",
        positive_elem="pan_barrel",
        negative_elem="distal_barrel",
        max_penetration=0.060,
        name="head pan barrel overlap stays inside the sleeve height",
    )
    ctx.expect_overlap(
        link_1,
        head_yoke,
        axes="xy",
        elem_a="arm_bar",
        elem_b="yoke_neck",
        min_overlap=0.010,
        name="pan yoke stack is compact at the arm end",
    )

    vertical_axis = (0.0, 0.0, 1.0)
    pitch_axis = (0.0, 1.0, 0.0)
    ctx.check(
        "three planar joints use vertical axes",
        wall_pivot.axis == vertical_axis and elbow_pivot.axis == vertical_axis and head_pan.axis == vertical_axis,
        details=f"axes: wall={wall_pivot.axis}, elbow={elbow_pivot.axis}, pan={head_pan.axis}",
    )
    ctx.check(
        "head pitch uses a horizontal axis",
        head_pitch.axis == pitch_axis,
        details=f"pitch axis={head_pitch.axis}",
    )

    folded_aabb = ctx.part_world_aabb(vesa_frame)
    ctx.check(
        "folded vesa frame sits close to the wall",
        folded_aabb is not None and folded_aabb[1][0] < 0.205,
        details=f"folded frame aabb={folded_aabb}",
    )

    with ctx.pose({wall_pivot: -1.35, elbow_pivot: 2.45, head_pan: 0.0, head_pitch: 0.0}):
        deployed_aabb = ctx.part_world_aabb(vesa_frame)
        ctx.check(
            "deployed links carry the frame outward",
            folded_aabb is not None
            and deployed_aabb is not None
            and deployed_aabb[1][0] > folded_aabb[1][0] + 0.330,
            details=f"folded={folded_aabb}, deployed={deployed_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
