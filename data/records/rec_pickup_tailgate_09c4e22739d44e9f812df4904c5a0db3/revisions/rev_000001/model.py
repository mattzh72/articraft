from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="pickup_tailgate")

    body_blue = model.material("painted_blue", rgba=(0.05, 0.18, 0.36, 1.0))
    edge_blue = model.material("edge_highlight_blue", rgba=(0.07, 0.24, 0.45, 1.0))
    dark_band = model.material("black_latch_band", rgba=(0.015, 0.017, 0.018, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.005, 0.005, 0.006, 1.0))
    hinge_steel = model.material("dark_hinge_steel", rgba=(0.12, 0.12, 0.12, 1.0))

    bed = model.part("bed_stubs")
    # Short bed-side remnants and a rear sill form the fixed frame that the
    # stand-alone tailgate hangs from.
    bed.visual(
        Box((0.46, 0.12, 0.70)),
        origin=Origin(xyz=(-0.18, 0.86, 0.39)),
        material=body_blue,
        name="side_stub_0",
    )
    bed.visual(
        Box((0.46, 0.12, 0.70)),
        origin=Origin(xyz=(-0.18, -0.86, 0.39)),
        material=body_blue,
        name="side_stub_1",
    )
    bed.visual(
        Box((0.16, 1.86, 0.08)),
        origin=Origin(xyz=(-0.07, 0.0, 0.04)),
        material=edge_blue,
        name="lower_sill",
    )
    bed.visual(
        Box((0.055, 0.040, 0.140)),
        origin=Origin(xyz=(0.000, 0.785, 0.120)),
        material=hinge_steel,
        name="hinge_bracket_0",
    )
    bed.visual(
        Box((0.055, 0.040, 0.140)),
        origin=Origin(xyz=(0.000, -0.785, 0.120)),
        material=hinge_steel,
        name="hinge_bracket_1",
    )
    bed.visual(
        Cylinder(radius=0.014, length=0.110),
        origin=Origin(xyz=(0.0, 0.725, 0.120), rpy=(-pi / 2, 0.0, 0.0)),
        material=hinge_steel,
        name="hinge_pin_0",
    )
    bed.visual(
        Cylinder(radius=0.014, length=0.110),
        origin=Origin(xyz=(0.0, -0.725, 0.120), rpy=(-pi / 2, 0.0, 0.0)),
        material=hinge_steel,
        name="hinge_pin_1",
    )

    tailgate = model.part("tailgate")
    # Child frame is on the lower transverse hinge axis.  The panel extends
    # upward at q=0 and swings outward/downward for positive rotation.
    tailgate.visual(
        Box((0.070, 1.500, 0.550)),
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
        material=body_blue,
        name="outer_panel",
    )
    tailgate.visual(
        Box((0.018, 1.420, 0.065)),
        origin=Origin(xyz=(0.040, 0.0, 0.575)),
        material=edge_blue,
        name="top_rail",
    )
    tailgate.visual(
        Box((0.018, 1.420, 0.055)),
        origin=Origin(xyz=(0.040, 0.0, 0.095)),
        material=edge_blue,
        name="bottom_rail",
    )
    tailgate.visual(
        Box((0.018, 0.070, 0.470)),
        origin=Origin(xyz=(0.040, 0.675, 0.335)),
        material=edge_blue,
        name="side_rail_0",
    )
    tailgate.visual(
        Box((0.018, 0.070, 0.470)),
        origin=Origin(xyz=(0.040, -0.675, 0.335)),
        material=edge_blue,
        name="side_rail_1",
    )
    tailgate.visual(
        Box((0.014, 1.260, 0.110)),
        origin=Origin(xyz=(0.0405, 0.0, 0.505)),
        material=dark_band,
        name="upper_latch_band",
    )
    tailgate.visual(
        Box((0.012, 0.900, 0.035)),
        origin=Origin(xyz=(0.039, 0.0, 0.390)),
        material=edge_blue,
        name="pressed_rib",
    )
    tailgate.visual(
        Box((0.050, 0.180, 0.042)),
        origin=Origin(xyz=(0.0, 0.660, 0.047)),
        material=hinge_steel,
        name="hinge_leaf_0",
    )
    tailgate.visual(
        Box((0.050, 0.180, 0.042)),
        origin=Origin(xyz=(0.0, -0.660, 0.047)),
        material=hinge_steel,
        name="hinge_leaf_1",
    )
    tailgate.visual(
        Cylinder(radius=0.028, length=0.140),
        origin=Origin(xyz=(0.0, 0.660, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=hinge_steel,
        name="hinge_barrel_0",
    )
    tailgate.visual(
        Cylinder(radius=0.028, length=0.140),
        origin=Origin(xyz=(0.0, -0.660, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=hinge_steel,
        name="hinge_barrel_1",
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.012, length=0.380),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=black_plastic,
        name="pivot_rod",
    )
    latch_handle.visual(
        Box((0.016, 0.340, 0.075)),
        origin=Origin(xyz=(0.010, 0.0, -0.042)),
        material=black_plastic,
        name="handle_paddle",
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.0, lower=0.0, upper=pi / 2),
    )
    model.articulation(
        "tailgate_to_handle",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=latch_handle,
        origin=Origin(xyz=(0.052, 0.0, 0.525)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=5.0, lower=0.0, upper=0.70),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed = object_model.get_part("bed_stubs")
    tailgate = object_model.get_part("tailgate")
    handle = object_model.get_part("latch_handle")
    tailgate_hinge = object_model.get_articulation("bed_to_tailgate")
    handle_pivot = object_model.get_articulation("tailgate_to_handle")

    for suffix in ("0", "1"):
        ctx.allow_overlap(
            bed,
            tailgate,
            elem_a=f"hinge_pin_{suffix}",
            elem_b=f"hinge_barrel_{suffix}",
            reason="The fixed hinge pin is intentionally captured inside the tailgate hinge barrel.",
        )
        ctx.expect_overlap(
            bed,
            tailgate,
            axes="y",
            elem_a=f"hinge_pin_{suffix}",
            elem_b=f"hinge_barrel_{suffix}",
            min_overlap=0.030,
            name=f"hinge pin {suffix} is retained in barrel",
        )
        ctx.expect_within(
            bed,
            tailgate,
            axes="xz",
            inner_elem=f"hinge_pin_{suffix}",
            outer_elem=f"hinge_barrel_{suffix}",
            margin=0.001,
            name=f"hinge pin {suffix} sits inside barrel bore",
        )

    ctx.allow_overlap(
        tailgate,
        handle,
        elem_a="upper_latch_band",
        elem_b="pivot_rod",
        reason="The latch handle pivot rod is seated through the recessed latch band in the tailgate skin.",
    )
    ctx.expect_overlap(
        tailgate,
        handle,
        axes="yz",
        elem_a="upper_latch_band",
        elem_b="pivot_rod",
        min_overlap=0.020,
        name="handle pivot is embedded in latch band",
    )

    ctx.expect_gap(
        tailgate,
        bed,
        axis="z",
        positive_elem="bottom_rail",
        negative_elem="lower_sill",
        min_gap=0.020,
        name="closed tailgate clears lower sill",
    )
    ctx.expect_within(
        handle,
        tailgate,
        axes="y",
        inner_elem="handle_paddle",
        outer_elem="upper_latch_band",
        margin=0.0,
        name="handle stays within upper band width",
    )

    closed_aabb = ctx.part_world_aabb(tailgate)
    closed_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({tailgate_hinge: 1.25}):
        lowered_aabb = ctx.part_world_aabb(tailgate)
    with ctx.pose({handle_pivot: 0.55}):
        pulled_handle_aabb = ctx.part_world_aabb(handle)

    ctx.check(
        "tailgate rotates downward and outward",
        closed_aabb is not None
        and lowered_aabb is not None
        and lowered_aabb[1][2] < closed_aabb[1][2] - 0.10
        and lowered_aabb[1][0] > closed_aabb[1][0] + 0.20,
        details=f"closed={closed_aabb}, lowered={lowered_aabb}",
    )
    ctx.check(
        "latch handle rotates out of skin",
        closed_handle_aabb is not None
        and pulled_handle_aabb is not None
        and pulled_handle_aabb[1][0] > closed_handle_aabb[1][0] + 0.015,
        details=f"closed={closed_handle_aabb}, pulled={pulled_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
