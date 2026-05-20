from __future__ import annotations

from math import atan, pi

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
    model = ArticulatedObject(name="sliding_security_gate")

    galvanized = model.material("galvanized_steel", rgba=(0.55, 0.58, 0.57, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.07, 0.08, 0.08, 1.0))
    safety_yellow = model.material("yellow_stop_caps", rgba=(0.95, 0.72, 0.10, 1.0))
    concrete = model.material("brushed_concrete", rgba=(0.45, 0.43, 0.39, 1.0))

    frame = model.part("frame")
    # One fixed root part: concrete footing, rear posts, and rigid top/bottom guide channels.
    frame.visual(Box((4.70, 0.46, 0.10)), origin=Origin(xyz=(0.45, -0.02, 0.05)), material=concrete, name="footing")
    frame.visual(Box((0.16, 0.16, 1.70)), origin=Origin(xyz=(-1.45, -0.20, 0.90)), material=galvanized, name="fixed_post")
    frame.visual(Box((0.16, 0.16, 1.70)), origin=Origin(xyz=(2.65, -0.20, 0.90)), material=galvanized, name="end_post")
    frame.visual(Box((0.20, 0.32, 0.08)), origin=Origin(xyz=(-1.45, -0.08, 1.56)), material=galvanized, name="front_track_bracket")
    frame.visual(Box((0.20, 0.32, 0.08)), origin=Origin(xyz=(2.65, -0.08, 1.56)), material=galvanized, name="rear_track_bracket")

    # Overhead C-track: thick side cheeks and lower lips leave a center slot for the gate hangers.
    frame.visual(Box((4.35, 0.26, 0.06)), origin=Origin(xyz=(0.45, 0.00, 1.71)), material=galvanized, name="top_track_cap")
    frame.visual(Box((4.35, 0.035, 0.22)), origin=Origin(xyz=(0.45, 0.1125, 1.61)), material=galvanized, name="top_cheek_pos")
    frame.visual(Box((4.35, 0.035, 0.22)), origin=Origin(xyz=(0.45, -0.1125, 1.61)), material=galvanized, name="top_cheek_neg")
    frame.visual(Box((4.35, 0.065, 0.035)), origin=Origin(xyz=(0.45, 0.0675, 1.495)), material=galvanized, name="top_lip_pos")
    frame.visual(Box((4.35, 0.065, 0.035)), origin=Origin(xyz=(0.45, -0.0675, 1.495)), material=galvanized, name="top_lip_neg")
    frame.visual(Box((0.10, 0.30, 0.30)), origin=Origin(xyz=(-1.72, 0.00, 1.59)), material=safety_yellow, name="track_stop")

    # Bottom guide channel: a low U-section captures a central shoe on the moving leaf.
    frame.visual(Box((4.35, 0.18, 0.04)), origin=Origin(xyz=(0.45, 0.00, 0.105)), material=galvanized, name="bottom_guide_base")
    frame.visual(Box((4.35, 0.035, 0.18)), origin=Origin(xyz=(0.45, 0.075, 0.20)), material=galvanized, name="guide_cheek_pos")
    frame.visual(Box((4.35, 0.035, 0.18)), origin=Origin(xyz=(0.45, -0.075, 0.20)), material=galvanized, name="guide_cheek_neg")

    leaf = model.part("gate_leaf")
    # Sliding leaf: a welded rectangular lattice with hangers, rollers, and bottom guide shoe.
    leaf.visual(Box((2.20, 0.060, 0.10)), origin=Origin(xyz=(0.00, 0.00, 0.345)), material=dark_steel, name="bottom_tube")
    leaf.visual(Box((2.20, 0.060, 0.10)), origin=Origin(xyz=(0.00, 0.00, 1.370)), material=dark_steel, name="top_tube")
    leaf.visual(Box((0.10, 0.060, 1.13)), origin=Origin(xyz=(-1.05, 0.00, 0.855)), material=dark_steel, name="front_stile")
    leaf.visual(Box((0.10, 0.060, 1.13)), origin=Origin(xyz=(1.05, 0.00, 0.855)), material=dark_steel, name="rear_stile")
    leaf.visual(Box((2.08, 0.045, 0.060)), origin=Origin(xyz=(0.00, 0.00, 0.850)), material=dark_steel, name="mid_rail")

    for i, x in enumerate((-0.78, -0.52, -0.26, 0.00, 0.26, 0.52, 0.78)):
        leaf.visual(Box((0.040, 0.040, 1.05)), origin=Origin(xyz=(x, 0.00, 0.855)), material=dark_steel, name=f"picket_{i}")

    brace_angle = -atan(0.83 / 2.02)
    leaf.visual(
        Box((2.18, 0.040, 0.050)),
        origin=Origin(xyz=(0.00, 0.00, 0.855), rpy=(0.0, brace_angle, 0.0)),
        material=dark_steel,
        name="diagonal_brace",
    )

    # Bottom shoe and central web ride inside the guide channel without touching its cheeks.
    leaf.visual(Box((2.12, 0.070, 0.080)), origin=Origin(xyz=(0.00, 0.00, 0.165)), material=dark_steel, name="bottom_shoe")
    leaf.visual(Box((2.08, 0.035, 0.100)), origin=Origin(xyz=(0.00, 0.00, 0.245)), material=dark_steel, name="shoe_web")

    leaf.visual(Box((0.050, 0.030, 0.180)), origin=Origin(xyz=(-0.65, 0.00, 1.505)), material=dark_steel, name="hanger_0")
    leaf.visual(Cylinder(radius=0.012, length=0.160), origin=Origin(xyz=(-0.65, 0.00, 1.585), rpy=(pi / 2.0, 0.0, 0.0)), material=galvanized, name="axle_0")
    leaf.visual(Cylinder(radius=0.055, length=0.045), origin=Origin(xyz=(-0.65, 0.00, 1.585), rpy=(pi / 2.0, 0.0, 0.0)), material=dark_steel, name="roller_0")
    leaf.visual(Box((0.050, 0.030, 0.180)), origin=Origin(xyz=(0.65, 0.00, 1.505)), material=dark_steel, name="hanger_1")
    leaf.visual(Cylinder(radius=0.012, length=0.160), origin=Origin(xyz=(0.65, 0.00, 1.585), rpy=(pi / 2.0, 0.0, 0.0)), material=galvanized, name="axle_1")
    leaf.visual(Cylinder(radius=0.055, length=0.045), origin=Origin(xyz=(0.65, 0.00, 1.585), rpy=(pi / 2.0, 0.0, 0.0)), material=dark_steel, name="roller_1")

    model.articulation(
        "frame_to_gate_leaf",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.35, lower=0.0, upper=1.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_gate_leaf")

    ctx.check(
        "gate leaf has one prismatic slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (1.0, 0.0, 0.0)
        and slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper == 1.20,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )

    # The guide shoe is narrower than the U-channel cheeks at rest.
    ctx.expect_gap(
        frame,
        leaf,
        axis="y",
        positive_elem="guide_cheek_pos",
        negative_elem="bottom_shoe",
        min_gap=0.010,
        name="positive guide cheek clears shoe",
    )
    ctx.expect_gap(
        leaf,
        frame,
        axis="y",
        positive_elem="bottom_shoe",
        negative_elem="guide_cheek_neg",
        min_gap=0.010,
        name="negative guide cheek clears shoe",
    )
    ctx.expect_overlap(
        leaf,
        frame,
        axes="x",
        elem_a="bottom_shoe",
        elem_b="bottom_guide_base",
        min_overlap=2.0,
        name="closed leaf is retained in bottom guide",
    )

    rest_pos = ctx.part_world_position(leaf)
    with ctx.pose({slide: 1.20}):
        ctx.expect_overlap(
            leaf,
            frame,
            axes="x",
            elem_a="bottom_shoe",
            elem_b="bottom_guide_base",
            min_overlap=2.0,
            name="open leaf remains captured by bottom guide",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="y",
            positive_elem="top_lip_pos",
            negative_elem="hanger_0",
            min_gap=0.010,
            name="hanger clears positive top lip while sliding",
        )
        ctx.expect_gap(
            leaf,
            frame,
            axis="y",
            positive_elem="hanger_0",
            negative_elem="top_lip_neg",
            min_gap=0.010,
            name="hanger clears negative top lip while sliding",
        )
        open_pos = ctx.part_world_position(leaf)

    ctx.check(
        "upper limit moves gate along track",
        rest_pos is not None and open_pos is not None and open_pos[0] > rest_pos[0] + 1.15,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
