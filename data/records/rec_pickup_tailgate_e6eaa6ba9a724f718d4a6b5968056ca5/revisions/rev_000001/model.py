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
    model = ArticulatedObject(name="pickup_tailgate")

    painted_blue = model.material("painted_blue", color=(0.07, 0.16, 0.28, 1.0))
    dark_trim = model.material("black_latch_trim", color=(0.015, 0.017, 0.020, 1.0))
    satin_steel = model.material("satin_steel", color=(0.62, 0.60, 0.55, 1.0))
    shadow = model.material("recess_shadow", color=(0.035, 0.055, 0.080, 1.0))

    bed_stubs = model.part("bed_stubs")
    # A short, connected fragment of the rear pickup bed: two side-wall stubs
    # joined by a lower sill and a shallow floor lip so it is visibly not the
    # rest of the truck body.
    bed_stubs.visual(
        Box((0.14, 0.42, 0.68)),
        origin=Origin(xyz=(-0.91, 0.12, 0.30)),
        material=painted_blue,
        name="side_stub_0",
    )
    bed_stubs.visual(
        Box((0.14, 0.42, 0.68)),
        origin=Origin(xyz=(0.91, 0.12, 0.30)),
        material=painted_blue,
        name="side_stub_1",
    )
    bed_stubs.visual(
        Box((1.96, 0.18, 0.09)),
        origin=Origin(xyz=(0.0, 0.02, -0.085)),
        material=painted_blue,
        name="lower_sill",
    )
    bed_stubs.visual(
        Box((1.70, 0.30, 0.04)),
        origin=Origin(xyz=(0.0, 0.20, 0.035)),
        material=shadow,
        name="bed_floor_lip",
    )
    for x, suffix in [(-0.827, "0"), (0.827, "1")]:
        bed_stubs.visual(
            Box((0.034, 0.090, 0.115)),
            origin=Origin(xyz=(x, -0.030, 0.575)),
            material=satin_steel,
            name=f"latch_striker_{suffix}",
        )
        bed_stubs.visual(
            Box((0.040, 0.105, 0.090)),
            origin=Origin(xyz=(x, -0.020, 0.000)),
            material=satin_steel,
            name=f"hinge_cheek_{suffix}",
        )

    # Short hinge pins are fixed to the bed-side stubs and pass through the
    # tailgate barrels.  They are intentionally captured inside the moving
    # barrels and are allowed in tests below.
    bed_stubs.visual(
        Cylinder(radius=0.012, length=0.23),
        origin=Origin(xyz=(-0.735, -0.020, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="hinge_pin_0",
    )
    bed_stubs.visual(
        Cylinder(radius=0.012, length=0.23),
        origin=Origin(xyz=(0.735, -0.020, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="hinge_pin_1",
    )

    tailgate = model.part("tailgate")
    # The tailgate part frame is the lower transverse hinge axis.  In the closed
    # pose the panel rises along local +Z; positive rotation about +X drops the
    # top edge rearward/downward.
    tailgate.visual(
        Box((1.54, 0.065, 0.58)),
        origin=Origin(xyz=(0.0, -0.025, 0.320)),
        material=painted_blue,
        name="main_panel",
    )
    tailgate.visual(
        Box((1.48, 0.018, 0.070)),
        origin=Origin(xyz=(0.0, -0.066, 0.585)),
        material=dark_trim,
        name="latch_band",
    )
    tailgate.visual(
        Box((1.38, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, -0.067, 0.108)),
        material=painted_blue,
        name="lower_stamp_rib",
    )
    tailgate.visual(
        Box((1.38, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, -0.067, 0.465)),
        material=painted_blue,
        name="upper_stamp_rib",
    )
    tailgate.visual(
        Box((0.040, 0.018, 0.360)),
        origin=Origin(xyz=(-0.565, -0.067, 0.285)),
        material=painted_blue,
        name="stamp_rib_0",
    )
    tailgate.visual(
        Box((0.040, 0.018, 0.360)),
        origin=Origin(xyz=(0.565, -0.067, 0.285)),
        material=painted_blue,
        name="stamp_rib_1",
    )
    tailgate.visual(
        Box((0.32, 0.012, 0.065)),
        origin=Origin(xyz=(0.0, -0.078, 0.535)),
        material=satin_steel,
        name="release_handle",
    )
    for x, suffix in [(-0.700, "0"), (0.700, "1")]:
        tailgate.visual(
            Cylinder(radius=0.035, length=0.16),
            origin=Origin(xyz=(x, -0.020, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name=f"hinge_barrel_{suffix}",
        )
        tailgate.visual(
            Box((0.150, 0.018, 0.120)),
            origin=Origin(xyz=(x, -0.066, 0.080)),
            material=satin_steel,
            name=f"hinge_strap_{suffix}",
        )

    model.articulation(
        "tailgate_hinge",
        ArticulationType.REVOLUTE,
        parent=bed_stubs,
        child=tailgate,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed_stubs = object_model.get_part("bed_stubs")
    tailgate = object_model.get_part("tailgate")
    hinge = object_model.get_articulation("tailgate_hinge")

    for suffix in ("0", "1"):
        ctx.allow_overlap(
            bed_stubs,
            tailgate,
            elem_a=f"hinge_pin_{suffix}",
            elem_b=f"hinge_barrel_{suffix}",
            reason="The fixed hinge pin is intentionally captured inside the moving tailgate barrel.",
        )
        ctx.expect_within(
            bed_stubs,
            tailgate,
            axes="yz",
            inner_elem=f"hinge_pin_{suffix}",
            outer_elem=f"hinge_barrel_{suffix}",
            margin=0.0,
            name=f"hinge pin {suffix} is centered inside its barrel",
        )
        ctx.expect_overlap(
            bed_stubs,
            tailgate,
            axes="x",
            elem_a=f"hinge_pin_{suffix}",
            elem_b=f"hinge_barrel_{suffix}",
            min_overlap=0.12,
            name=f"hinge pin {suffix} passes through the barrel",
        )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            tailgate,
            bed_stubs,
            axis="x",
            positive_elem="main_panel",
            negative_elem="side_stub_0",
            min_gap=0.025,
            name="closed tailgate clears side stub 0",
        )
        ctx.expect_gap(
            bed_stubs,
            tailgate,
            axis="x",
            positive_elem="side_stub_1",
            negative_elem="main_panel",
            min_gap=0.025,
            name="closed tailgate clears side stub 1",
        )
        ctx.expect_overlap(
            tailgate,
            bed_stubs,
            axes="z",
            elem_a="latch_band",
            elem_b="latch_striker_0",
            min_overlap=0.035,
            name="latch band aligns with side strikers",
        )

    closed_panel = ctx.part_element_world_aabb(tailgate, elem="main_panel")
    with ctx.pose({hinge: math.pi / 2.0}):
        open_panel = ctx.part_element_world_aabb(tailgate, elem="main_panel")

    closed_top_z = closed_panel[1][2] if closed_panel else None
    open_top_z = open_panel[1][2] if open_panel else None
    open_rear_y = open_panel[0][1] if open_panel else None
    ctx.check(
        "tailgate rotates downward about lower hinge",
        closed_top_z is not None
        and open_top_z is not None
        and open_rear_y is not None
        and closed_top_z > 0.55
        and open_top_z < 0.08
        and open_rear_y < -0.58,
        details=f"closed_top_z={closed_top_z}, open_top_z={open_top_z}, open_rear_y={open_rear_y}",
    )

    return ctx.report()


object_model = build_object_model()
