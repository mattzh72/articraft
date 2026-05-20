from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_sash_sliding_window")

    white_vinyl = model.material("white_vinyl", rgba=(0.92, 0.91, 0.86, 1.0))
    track_shadow = model.material("dark_track_shadow", rgba=(0.08, 0.085, 0.09, 1.0))
    glass = model.material("slightly_blue_glass", rgba=(0.56, 0.78, 0.95, 0.34))
    rubber = model.material("black_rubber", rgba=(0.015, 0.018, 0.02, 1.0))
    handle_mat = model.material("brushed_dark_handle", rgba=(0.18, 0.18, 0.17, 1.0))

    base = model.part("base_frame")
    # Rectangular outer frame, sized like a domestic horizontal sliding window.
    base.visual(Box((1.60, 0.10, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.56)), material=white_vinyl, name="top_rail")
    base.visual(Box((1.60, 0.10, 0.08)), origin=Origin(xyz=(0.0, 0.0, -0.56)), material=white_vinyl, name="bottom_rail")
    base.visual(Box((0.08, 0.10, 1.20)), origin=Origin(xyz=(-0.76, 0.0, 0.0)), material=white_vinyl, name="jamb_0")
    base.visual(Box((0.08, 0.10, 1.20)), origin=Origin(xyz=(0.76, 0.0, 0.0)), material=white_vinyl, name="jamb_1")

    # The fixed half of the window is built into the base frame.
    base.visual(Box((0.065, 0.085, 1.04)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=white_vinyl, name="meeting_stile")
    base.visual(Box((0.75, 0.014, 1.06)), origin=Origin(xyz=(-0.395, -0.006, 0.0)), material=glass, name="fixed_glass")
    base.visual(Box((0.68, 0.030, 0.030)), origin=Origin(xyz=(-0.395, -0.020, 0.49)), material=rubber, name="fixed_top_gasket")
    base.visual(Box((0.68, 0.030, 0.030)), origin=Origin(xyz=(-0.395, -0.020, -0.49)), material=rubber, name="fixed_bottom_gasket")

    guide = model.part("guide_module")
    # Top and bottom U-shaped guide channels are connected by end returns that
    # touch the outer jambs, making the module read as a single mounted track.
    guide.visual(Box((1.36, 0.120, 0.025)), origin=Origin(xyz=(0.0, -0.100, 0.5075)), material=white_vinyl, name="top_web")
    guide.visual(Box((1.36, 0.012, 0.050)), origin=Origin(xyz=(0.0, -0.145, 0.470)), material=white_vinyl, name="top_front_lip")
    guide.visual(Box((1.36, 0.012, 0.050)), origin=Origin(xyz=(0.0, -0.055, 0.470)), material=white_vinyl, name="top_rear_lip")
    guide.visual(Box((1.36, 0.120, 0.025)), origin=Origin(xyz=(0.0, -0.100, -0.5075)), material=white_vinyl, name="bottom_web")
    guide.visual(Box((1.36, 0.012, 0.050)), origin=Origin(xyz=(0.0, -0.145, -0.470)), material=white_vinyl, name="bottom_front_lip")
    guide.visual(Box((1.36, 0.012, 0.050)), origin=Origin(xyz=(0.0, -0.055, -0.470)), material=white_vinyl, name="bottom_rear_lip")
    guide.visual(Box((0.040, 0.120, 1.04)), origin=Origin(xyz=(-0.700, -0.100, 0.0)), material=white_vinyl, name="end_return_0")
    guide.visual(Box((0.040, 0.120, 1.04)), origin=Origin(xyz=(0.700, -0.100, 0.0)), material=white_vinyl, name="end_return_1")
    guide.visual(Box((1.30, 0.004, 0.035)), origin=Origin(xyz=(0.0, -0.152, 0.470)), material=track_shadow, name="top_shadow_slot")
    guide.visual(Box((1.30, 0.004, 0.035)), origin=Origin(xyz=(0.0, -0.152, -0.470)), material=track_shadow, name="bottom_shadow_slot")

    sash = model.part("sash_carriage")
    # Child-frame origin is the moving sash center.  At q=0 the sash covers the
    # right opening; positive joint travel slides it left over the fixed pane.
    sash.visual(Box((0.680, 0.035, 0.065)), origin=Origin(xyz=(0.0, 0.0, 0.4375)), material=white_vinyl, name="sash_top_rail")
    sash.visual(Box((0.680, 0.035, 0.065)), origin=Origin(xyz=(0.0, 0.0, -0.4375)), material=white_vinyl, name="sash_bottom_rail")
    sash.visual(Box((0.055, 0.035, 0.855)), origin=Origin(xyz=(-0.3125, 0.0, 0.0)), material=white_vinyl, name="sash_stile_0")
    sash.visual(Box((0.055, 0.035, 0.855)), origin=Origin(xyz=(0.3125, 0.0, 0.0)), material=white_vinyl, name="sash_stile_1")
    sash.visual(Box((0.590, 0.012, 0.800)), origin=Origin(xyz=(0.0, 0.002, 0.0)), material=glass, name="sash_glass")
    sash.visual(Box((0.660, 0.052, 0.020)), origin=Origin(xyz=(0.0, 0.0, 0.480)), material=rubber, name="top_shoe")
    sash.visual(Box((0.660, 0.052, 0.020)), origin=Origin(xyz=(0.0, 0.0, -0.480)), material=rubber, name="bottom_shoe")
    sash.visual(Box((0.025, 0.025, 0.260)), origin=Origin(xyz=(-0.290, -0.030, 0.0)), material=handle_mat, name="pull_handle")

    model.articulation(
        "frame_to_guide",
        ArticulationType.FIXED,
        parent=base,
        child=guide,
        origin=Origin(),
    )
    model.articulation(
        "guide_to_sash",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=sash,
        origin=Origin(xyz=(0.340, -0.100, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.58),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    guide = object_model.get_part("guide_module")
    sash = object_model.get_part("sash_carriage")
    slide = object_model.get_articulation("guide_to_sash")

    ctx.expect_contact(
        base,
        guide,
        contact_tol=0.001,
        name="guide module seats against the outer frame",
    )
    ctx.expect_within(
        sash,
        guide,
        axes="xy",
        inner_elem="top_shoe",
        outer_elem="top_web",
        margin=0.003,
        name="top shoe sits inside the top guide channel at rest",
    )
    ctx.expect_within(
        sash,
        guide,
        axes="xy",
        inner_elem="bottom_shoe",
        outer_elem="bottom_web",
        margin=0.003,
        name="bottom shoe sits inside the bottom guide channel at rest",
    )
    ctx.expect_overlap(
        sash,
        guide,
        axes="x",
        elem_a="top_shoe",
        elem_b="top_web",
        min_overlap=0.60,
        name="closed sash is retained by the upper guide",
    )

    rest_pos = ctx.part_world_position(sash)
    with ctx.pose({slide: 0.58}):
        ctx.expect_within(
            sash,
            guide,
            axes="xy",
            inner_elem="top_shoe",
            outer_elem="top_web",
            margin=0.003,
            name="top shoe remains captured at full slide",
        )
        ctx.expect_overlap(
            sash,
            guide,
            axes="x",
            elem_a="bottom_shoe",
            elem_b="bottom_web",
            min_overlap=0.55,
            name="open sash remains retained by the lower guide",
        )
        extended_pos = ctx.part_world_position(sash)

    ctx.check(
        "prismatic sash travels horizontally left",
        rest_pos is not None and extended_pos is not None and extended_pos[0] < rest_pos[0] - 0.50,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
