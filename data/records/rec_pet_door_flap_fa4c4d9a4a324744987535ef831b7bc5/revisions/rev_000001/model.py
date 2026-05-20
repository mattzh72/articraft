from __future__ import annotations

from math import isclose, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_stored_pet_door")

    warm_plastic = Material("warm_white_plastic", rgba=(0.83, 0.78, 0.67, 1.0))
    dark_track = Material("dark_gray_track", rgba=(0.12, 0.13, 0.13, 1.0))
    smoky_clear = Material("smoky_clear_vinyl", rgba=(0.52, 0.68, 0.78, 0.42))
    cover_plastic = Material("smoke_gray_security_cover", rgba=(0.25, 0.29, 0.31, 0.92))
    screw_metal = Material("dull_screw_heads", rgba=(0.35, 0.33, 0.30, 1.0))

    # Coordinates: X is across the insert, Y is door thickness/front-back, Z is up.
    # The pet opening occupies the left bay; the security panel is stored in the
    # right-side pocket and slides leftward along the rear guide track.
    frame = model.part("frame")

    # Main molded rim around the opening and side pocket.
    frame.visual(
        Box((0.74, 0.070, 0.075)),
        origin=Origin(xyz=(0.010, 0.000, 0.2225)),
        material=warm_plastic,
        name="top_rail",
    )
    frame.visual(
        Box((0.74, 0.070, 0.075)),
        origin=Origin(xyz=(0.010, 0.000, -0.2225)),
        material=warm_plastic,
        name="bottom_rail",
    )
    frame.visual(
        Box((0.080, 0.070, 0.520)),
        origin=Origin(xyz=(-0.320, 0.000, 0.000)),
        material=warm_plastic,
        name="opening_jamb",
    )
    frame.visual(
        Box((0.030, 0.070, 0.520)),
        origin=Origin(xyz=(0.365, 0.000, 0.000)),
        material=warm_plastic,
        name="pocket_end_wall",
    )
    frame.visual(
        Box((0.030, 0.060, 0.520)),
        origin=Origin(xyz=(0.035, -0.005, 0.000)),
        material=warm_plastic,
        name="pocket_divider",
    )
    frame.visual(
        Box((0.340, 0.010, 0.420)),
        origin=Origin(xyz=(0.200, 0.060, 0.000)),
        material=warm_plastic,
        name="side_pocket_back",
    )

    # Rear C-channel guide: bearing bars above/below and narrow lips keep the
    # sliding security cover captured without blocking its travel.
    frame.visual(
        Box((0.680, 0.020, 0.016)),
        origin=Origin(xyz=(0.030, 0.045, 0.205)),
        material=dark_track,
        name="upper_track_bearing",
    )
    frame.visual(
        Box((0.680, 0.020, 0.016)),
        origin=Origin(xyz=(0.030, 0.045, -0.205)),
        material=dark_track,
        name="lower_track_bearing",
    )
    for z, name_prefix in ((0.191, "upper"), (-0.191, "lower")):
        frame.visual(
            Box((0.680, 0.004, 0.025)),
            origin=Origin(xyz=(0.030, 0.036, z)),
            material=dark_track,
            name=f"{name_prefix}_front_lip",
        )
        frame.visual(
            Box((0.680, 0.004, 0.025)),
            origin=Origin(xyz=(0.030, 0.054, z)),
            material=dark_track,
            name=f"{name_prefix}_rear_lip",
        )

    # Small molded hinge ears at either end of the opening.  The flap carries
    # the center barrel; the fixed ears are outside it, sharing the same axis.
    for x, name_prefix in ((-0.3025, "hinge_ear_0"), (0.0325, "hinge_ear_1")):
        frame.visual(
            Box((0.055, 0.014, 0.026)),
            origin=Origin(xyz=(x, -0.036, 0.185)),
            material=warm_plastic,
            name=f"{name_prefix}_bracket",
        )
        frame.visual(
            Cylinder(radius=0.010, length=0.045),
            origin=Origin(xyz=(x, -0.045, 0.185), rpy=(0.0, pi / 2.0, 0.0)),
            material=warm_plastic,
            name=f"{name_prefix}_barrel",
        )

    # Four flush fasteners reinforce that this is a door insert frame.
    for i, (x, z) in enumerate(((-0.325, 0.235), (0.325, 0.235), (-0.325, -0.235), (0.325, -0.235))):
        frame.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(x, -0.038, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name=f"screw_{i}",
        )

    flap = model.part("flap")
    flap.visual(
        Box((0.280, 0.008, 0.350)),
        origin=Origin(xyz=(0.000, 0.000, -0.175)),
        material=smoky_clear,
        name="flex_panel",
    )
    flap.visual(
        Cylinder(radius=0.010, length=0.240),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=smoky_clear,
        name="hinge_barrel",
    )
    flap.visual(
        Box((0.210, 0.010, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, -0.030)),
        material=smoky_clear,
        name="top_reinforcement",
    )

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(-0.130, -0.045, 0.185)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=-1.15, upper=1.15),
    )

    cover = model.part("security_cover")
    cover.visual(
        Box((0.300, 0.010, 0.370)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=cover_plastic,
        name="cover_panel",
    )
    cover.visual(
        Box((0.300, 0.014, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, 0.191)),
        material=cover_plastic,
        name="upper_runner",
    )
    cover.visual(
        Box((0.300, 0.014, 0.012)),
        origin=Origin(xyz=(0.000, 0.000, -0.191)),
        material=cover_plastic,
        name="lower_runner",
    )
    cover.visual(
        Box((0.035, 0.014, 0.120)),
        origin=Origin(xyz=(-0.130, -0.011, 0.000)),
        material=cover_plastic,
        name="finger_pull",
    )

    model.articulation(
        "cover_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=cover,
        origin=Origin(xyz=(0.200, 0.045, 0.000)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.6, lower=0.0, upper=0.330),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    cover = object_model.get_part("security_cover")
    flap_hinge = object_model.get_articulation("flap_hinge")
    cover_slide = object_model.get_articulation("cover_slide")

    ctx.check(
        "flap uses upper horizontal hinge",
        flap_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(flap_hinge.axis) == (1.0, 0.0, 0.0)
        and flap_hinge.motion_limits is not None
        and isclose(flap_hinge.motion_limits.lower, -1.15)
        and isclose(flap_hinge.motion_limits.upper, 1.15),
        details=f"type={flap_hinge.articulation_type}, axis={flap_hinge.axis}, limits={flap_hinge.motion_limits}",
    )
    ctx.check(
        "security cover slides left from side pocket",
        cover_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(cover_slide.axis) == (-1.0, 0.0, 0.0)
        and cover_slide.motion_limits is not None
        and isclose(cover_slide.motion_limits.lower, 0.0)
        and isclose(cover_slide.motion_limits.upper, 0.330),
        details=f"type={cover_slide.articulation_type}, axis={cover_slide.axis}, limits={cover_slide.motion_limits}",
    )

    def expect_cover_captured(prefix: str) -> None:
        ctx.expect_gap(
            frame,
            cover,
            axis="z",
            positive_elem="upper_track_bearing",
            negative_elem="upper_runner",
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"{prefix} upper runner remains under bearing",
        )
        ctx.expect_gap(
            cover,
            frame,
            axis="z",
            positive_elem="lower_runner",
            negative_elem="lower_track_bearing",
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"{prefix} lower runner remains over bearing",
        )
        ctx.expect_within(
            cover,
            frame,
            axes="y",
            inner_elem="cover_panel",
            outer_elem="upper_track_bearing",
            margin=0.0005,
            name=f"{prefix} cover thickness stays between guide lips",
        )
        ctx.expect_overlap(
            cover,
            frame,
            axes="x",
            elem_a="upper_runner",
            elem_b="upper_track_bearing",
            min_overlap=0.25,
            name=f"{prefix} runner retains long engagement in track",
        )

    ctx.expect_within(
        cover,
        frame,
        axes="xz",
        inner_elem="cover_panel",
        outer_elem="side_pocket_back",
        margin=0.002,
        name="cover is stored in side pocket at rest",
    )
    expect_cover_captured("stored")

    rest_cover_pos = ctx.part_world_position(cover)
    with ctx.pose({cover_slide: 0.330}):
        expect_cover_captured("blocking")
        blocking_cover_pos = ctx.part_world_position(cover)
        panel_aabb = ctx.part_element_world_aabb(cover, elem="cover_panel")
        if panel_aabb is None:
            ctx.fail("cover panel aabb available", "cover_panel had no world AABB")
        else:
            panel_min, panel_max = panel_aabb
            ctx.check(
                "cover blocks full pet opening width",
                panel_min[0] <= -0.279 and panel_max[0] >= 0.019,
                details=f"panel_min={panel_min}, panel_max={panel_max}",
            )

    ctx.check(
        "cover travels from pocket across opening",
        rest_cover_pos is not None
        and blocking_cover_pos is not None
        and blocking_cover_pos[0] < rest_cover_pos[0] - 0.30,
        details=f"rest={rest_cover_pos}, blocking={blocking_cover_pos}",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(flap, elem="flex_panel")
    with ctx.pose({flap_hinge: 0.80}):
        open_flap_aabb = ctx.part_element_world_aabb(flap, elem="flex_panel")
        if closed_flap_aabb is None or open_flap_aabb is None:
            ctx.fail("flap panel aabb available", "flex_panel had no world AABB")
        else:
            closed_min, closed_max = closed_flap_aabb
            open_min, open_max = open_flap_aabb
            ctx.check(
                "positive hinge motion lifts lightweight flap",
                open_min[2] > closed_min[2] + 0.05 and open_max[1] > closed_max[1] + 0.08,
                details=f"closed=({closed_min}, {closed_max}), open=({open_min}, {open_max})",
            )

    return ctx.report()


object_model = build_object_model()
