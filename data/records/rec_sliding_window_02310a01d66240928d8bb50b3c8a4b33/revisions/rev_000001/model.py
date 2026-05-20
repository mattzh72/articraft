from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_sliding_window")

    painted_metal = model.material("satin_graphite_painted_aluminum", rgba=(0.10, 0.115, 0.12, 1.0))
    dark_shadow = model.material("dark_recess_shadow", rgba=(0.018, 0.020, 0.022, 1.0))
    glass = model.material("slightly_blue_low_e_glass", rgba=(0.62, 0.82, 0.92, 0.34))
    elastomer = model.material("matte_black_elastomer_gasket", rgba=(0.005, 0.006, 0.006, 1.0))
    polymer = model.material("low_friction_black_polymer", rgba=(0.025, 0.025, 0.023, 1.0))

    frame = model.part("frame")

    # Overall size is a realistic residential premium slider: 1.60 m wide,
    # 1.05 m tall, with a compact 110 mm deep thermally-broken metal frame.
    frame.visual(
        Box((1.60, 0.110, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=painted_metal,
        name="sill_floor",
    )
    frame.visual(
        Box((1.60, 0.110, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 1.015)),
        material=painted_metal,
        name="head_rail",
    )
    frame.visual(
        Box((0.075, 0.110, 1.05)),
        origin=Origin(xyz=(-0.7625, 0.0, 0.525)),
        material=painted_metal,
        name="jamb_0",
    )
    frame.visual(
        Box((0.075, 0.110, 1.05)),
        origin=Origin(xyz=(0.7625, 0.0, 0.525)),
        material=painted_metal,
        name="jamb_1",
    )

    # Front sliding channel: two lips on the sill and two lips descending from
    # the head rail.  The moving sash rides between these lips rather than in
    # a broad proxy block.
    for y, name in ((-0.004, "bottom_inner_lip"), (0.051, "bottom_outer_lip")):
        frame.visual(
            Box((1.455, 0.010, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.095)),
            material=painted_metal,
            name=name,
        )
    for y, name in ((-0.004, "top_inner_lip"), (0.051, "top_outer_lip")):
        frame.visual(
            Box((1.455, 0.010, 0.040)),
            origin=Origin(xyz=(0.0, y, 0.960)),
            material=painted_metal,
            name=name,
        )

    # Separate rear channel and fixed lite, permanently captured in the frame.
    for y, name in ((-0.052, "rear_outer_lip"), (-0.009, "rear_inner_lip")):
        frame.visual(
            Box((0.735, 0.008, 0.028)),
            origin=Origin(xyz=(-0.365, y, 0.096)),
            material=painted_metal,
            name=name,
        )
        frame.visual(
            Box((0.735, 0.008, 0.036)),
            origin=Origin(xyz=(-0.365, y, 0.958)),
            material=painted_metal,
            name=f"{name}_head",
        )

    # Fixed rear lite and its slim sash.  These are in the root frame because
    # they are not user-moving; rail overlaps at the jamb/sill make the support
    # path explicit instead of leaving a floating pane.
    fixed_x = -0.365
    fixed_y = -0.030
    fixed_z = 0.525
    frame.visual(
        Box((0.666, 0.008, 0.810)),
        origin=Origin(xyz=(fixed_x, fixed_y, fixed_z)),
        material=glass,
        name="fixed_glass",
    )
    frame.visual(
        Box((0.740, 0.036, 0.044)),
        origin=Origin(xyz=(fixed_x, fixed_y, 0.100)),
        material=painted_metal,
        name="fixed_bottom_rail",
    )
    frame.visual(
        Box((0.740, 0.036, 0.044)),
        origin=Origin(xyz=(fixed_x, fixed_y, 0.950)),
        material=painted_metal,
        name="fixed_top_rail",
    )
    frame.visual(
        Box((0.045, 0.036, 0.880)),
        origin=Origin(xyz=(-0.715, fixed_y, fixed_z)),
        material=painted_metal,
        name="fixed_outer_stile",
    )
    frame.visual(
        Box((0.050, 0.036, 0.880)),
        origin=Origin(xyz=(-0.012, fixed_y, fixed_z)),
        material=painted_metal,
        name="fixed_meeting_stile",
    )
    frame.visual(
        Box((0.012, 0.012, 0.790)),
        origin=Origin(xyz=(0.006, -0.007, fixed_z)),
        material=elastomer,
        name="fixed_interlock_gasket",
    )
    frame.visual(
        Box((0.646, 0.010, 0.010)),
        origin=Origin(xyz=(fixed_x, -0.022, 0.125)),
        material=elastomer,
        name="fixed_bottom_gasket",
    )
    frame.visual(
        Box((0.646, 0.010, 0.010)),
        origin=Origin(xyz=(fixed_x, -0.022, 0.925)),
        material=elastomer,
        name="fixed_top_gasket",
    )
    frame.visual(
        Box((0.010, 0.010, 0.790)),
        origin=Origin(xyz=(-0.700, -0.022, fixed_z)),
        material=elastomer,
        name="fixed_side_gasket_0",
    )
    frame.visual(
        Box((0.010, 0.010, 0.790)),
        origin=Origin(xyz=(-0.030, -0.022, fixed_z)),
        material=elastomer,
        name="fixed_side_gasket_1",
    )

    # Small elastomer bump stops are attached to the front channel.  They are
    # local, visible stops rather than arbitrary travel limits floating in space.
    frame.visual(
        Box((0.018, 0.030, 0.130)),
        origin=Origin(xyz=(0.744, 0.023, 0.145)),
        material=elastomer,
        name="closed_stop",
    )
    frame.visual(
        Box((0.018, 0.030, 0.130)),
        origin=Origin(xyz=(-0.562, 0.023, 0.145)),
        material=elastomer,
        name="open_stop",
    )

    sash = model.part("sash")
    # The sash part frame is at the center of the closed sliding panel.  The
    # joint origin places that frame in the front channel; positive travel is
    # toward the fixed lite, preserving insertion under the top and bottom lips.
    sash.visual(
        Box((0.640, 0.006, 0.730)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=glass,
        name="moving_glass",
    )
    sash.visual(
        Box((0.720, 0.038, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, -0.425)),
        material=painted_metal,
        name="bottom_rail",
    )
    sash.visual(
        Box((0.720, 0.038, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=painted_metal,
        name="top_rail",
    )
    sash.visual(
        Box((0.046, 0.038, 0.880)),
        origin=Origin(xyz=(-0.337, 0.0, 0.0)),
        material=painted_metal,
        name="meeting_stile",
    )
    sash.visual(
        Box((0.046, 0.038, 0.880)),
        origin=Origin(xyz=(0.337, 0.0, 0.0)),
        material=painted_metal,
        name="outer_stile",
    )
    sash.visual(
        Box((0.014, 0.012, 0.780)),
        origin=Origin(xyz=(-0.365, -0.006, 0.0)),
        material=elastomer,
        name="moving_interlock_gasket",
    )
    sash.visual(
        Box((0.626, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.004, -0.370)),
        material=elastomer,
        name="bottom_glazing_gasket",
    )
    sash.visual(
        Box((0.626, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.004, 0.370)),
        material=elastomer,
        name="top_glazing_gasket",
    )
    sash.visual(
        Box((0.010, 0.010, 0.710)),
        origin=Origin(xyz=(-0.324, 0.004, 0.0)),
        material=elastomer,
        name="side_glazing_gasket_0",
    )
    sash.visual(
        Box((0.010, 0.010, 0.710)),
        origin=Origin(xyz=(0.324, 0.004, 0.0)),
        material=elastomer,
        name="side_glazing_gasket_1",
    )
    sash.visual(
        Box((0.018, 0.010, 0.230)),
        origin=Origin(xyz=(0.306, 0.018, 0.035)),
        material=dark_shadow,
        name="flush_pull_recess",
    )
    sash.visual(
        Box((0.022, 0.014, 0.245)),
        origin=Origin(xyz=(0.324, 0.026, 0.035)),
        material=painted_metal,
        name="pull_lip",
    )
    for x, name in ((-0.230, "shoe_0"), (0.230, "shoe_1")):
        sash.visual(
            Box((0.095, 0.030, 0.010)),
            origin=Origin(xyz=(x, 0.0, -0.445)),
            material=polymer,
            name=name,
        )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(rpy=(-1.5708, 0.0, 0.0)),
        material=painted_metal,
        name="hub",
    )
    latch.visual(
        Box((0.022, 0.010, 0.120)),
        origin=Origin(xyz=(0.0, 0.010, 0.065)),
        material=painted_metal,
        name="thumb_turn",
    )
    latch.visual(
        Box((0.048, 0.010, 0.018)),
        origin=Origin(xyz=(-0.030, 0.010, 0.010)),
        material=polymer,
        name="cam_nose",
    )

    model.articulation(
        "frame_to_sash",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.365, 0.023, 0.530)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.555),
        motion_properties=MotionProperties(damping=8.0, friction=2.0),
    )
    model.articulation(
        "sash_to_latch",
        ArticulationType.REVOLUTE,
        parent=sash,
        child=latch,
        origin=Origin(xyz=(-0.337, 0.026, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.10),
        motion_properties=MotionProperties(damping=0.3, friction=0.08),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    joint = object_model.get_articulation("frame_to_sash")
    latch_joint = object_model.get_articulation("sash_to_latch")

    def _min_max_y(aabb):
        return aabb[0][1], aabb[1][1]

    def _channel_check(check_name: str) -> None:
        rail = ctx.part_element_world_aabb(sash, elem="bottom_rail")
        inner = ctx.part_element_world_aabb(frame, elem="bottom_inner_lip")
        outer = ctx.part_element_world_aabb(frame, elem="bottom_outer_lip")
        ok = rail is not None and inner is not None and outer is not None
        if ok:
            rail_min_y, rail_max_y = _min_max_y(rail)
            inner_max_y = inner[1][1]
            outer_min_y = outer[0][1]
            ok = rail_min_y > inner_max_y + 0.002 and rail_max_y < outer_min_y - 0.002
        ctx.check(
            check_name,
            ok,
            details=f"rail={rail}, inner_lip={inner}, outer_lip={outer}",
        )

    ctx.check(
        "sliding sash has realistic horizontal travel",
        joint.motion_limits is not None
        and joint.motion_limits.lower == 0.0
        and joint.motion_limits.upper is not None
        and 0.50 <= joint.motion_limits.upper <= 0.60
        and tuple(joint.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={joint.axis}, limits={joint.motion_limits}",
    )
    ctx.check(
        "thumb latch has a short rotary throw",
        latch_joint.motion_limits is not None
        and latch_joint.motion_limits.lower == 0.0
        and latch_joint.motion_limits.upper is not None
        and 0.9 <= latch_joint.motion_limits.upper <= 1.25
        and tuple(latch_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={latch_joint.axis}, limits={latch_joint.motion_limits}",
    )

    _channel_check("closed sash rail sits between front channel lips")
    ctx.expect_gap(
        sash,
        frame,
        axis="z",
        positive_elem="shoe_0",
        negative_elem="sill_floor",
        min_gap=0.0,
        max_gap=0.001,
        name="closed polymer shoe bears on sill track",
    )
    ctx.expect_overlap(
        sash,
        frame,
        axes="x",
        elem_a="bottom_rail",
        elem_b="bottom_inner_lip",
        min_overlap=0.68,
        name="closed sash retained along lower guide",
    )

    rest_pos = ctx.part_world_position(sash)
    with ctx.pose({joint: 0.555}):
        _channel_check("open sash rail remains between front channel lips")
        ctx.expect_overlap(
            sash,
            frame,
            axes="x",
            elem_a="bottom_rail",
            elem_b="bottom_inner_lip",
            min_overlap=0.68,
            name="open sash still retained in lower guide",
        )
        ctx.expect_gap(
            sash,
            frame,
            axis="z",
            positive_elem="shoe_0",
            negative_elem="sill_floor",
            min_gap=0.0,
            max_gap=0.001,
            name="open polymer shoe still bears on sill track",
        )
        open_pos = ctx.part_world_position(sash)

    ctx.check(
        "positive joint travel opens toward fixed lite",
        rest_pos is not None and open_pos is not None and open_pos[0] < rest_pos[0] - 0.50,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
