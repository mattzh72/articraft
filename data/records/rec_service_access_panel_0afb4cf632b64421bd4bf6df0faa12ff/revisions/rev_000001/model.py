from __future__ import annotations

import math

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
    model = ArticulatedObject(name="desktop_service_access_panel")

    powder = model.material("powder_coated_frame", color=(0.62, 0.65, 0.66, 1.0))
    dark = model.material("dark_service_cavity", color=(0.04, 0.045, 0.05, 1.0))
    panel_paint = model.material("warm_white_panel", color=(0.88, 0.86, 0.80, 1.0))
    rubber = model.material("black_rubber_gasket", color=(0.01, 0.012, 0.014, 1.0))
    hardware = model.material("brushed_steel_hardware", color=(0.55, 0.56, 0.54, 1.0))
    latch_color = model.material("satin_black_latch", color=(0.015, 0.015, 0.014, 1.0))

    frame = model.part("frame")
    # A compact desktop/apartment frame: shallow footprint, raised service opening,
    # and a small base plate so the panel reads as one supported assembly.
    frame.visual(
        Box((0.360, 0.160, 0.020)),
        origin=Origin(xyz=(0.000, 0.035, 0.010)),
        material=powder,
        name="desktop_base",
    )
    frame.visual(
        Box((0.320, 0.024, 0.030)),
        origin=Origin(xyz=(0.000, 0.000, 0.035)),
        material=powder,
        name="bottom_rail",
    )
    frame.visual(
        Box((0.320, 0.024, 0.030)),
        origin=Origin(xyz=(0.000, 0.000, 0.385)),
        material=powder,
        name="top_rail",
    )
    frame.visual(
        Box((0.030, 0.024, 0.360)),
        origin=Origin(xyz=(-0.145, 0.000, 0.210)),
        material=powder,
        name="hinge_stile",
    )
    frame.visual(
        Box((0.030, 0.024, 0.360)),
        origin=Origin(xyz=(0.145, 0.000, 0.210)),
        material=powder,
        name="latch_stile",
    )
    # Recessed dark back and side returns make the framed opening visibly hollow
    # without using a solid slab across the front opening.
    frame.visual(
        Box((0.252, 0.004, 0.300)),
        origin=Origin(xyz=(0.000, 0.044, 0.215)),
        material=dark,
        name="service_back",
    )
    frame.visual(
        Box((0.012, 0.042, 0.300)),
        origin=Origin(xyz=(-0.126, 0.028, 0.215)),
        material=dark,
        name="hinge_return",
    )
    frame.visual(
        Box((0.012, 0.042, 0.300)),
        origin=Origin(xyz=(0.126, 0.028, 0.215)),
        material=dark,
        name="latch_return",
    )
    frame.visual(
        Box((0.252, 0.058, 0.012)),
        origin=Origin(xyz=(0.000, 0.020, 0.059)),
        material=dark,
        name="bottom_return",
    )
    frame.visual(
        Box((0.252, 0.058, 0.012)),
        origin=Origin(xyz=(0.000, 0.020, 0.371)),
        material=dark,
        name="top_return",
    )
    # Desktop-friendly anti-tip gussets tied into the base and lower frame.
    frame.visual(
        Box((0.024, 0.105, 0.055)),
        origin=Origin(xyz=(-0.110, 0.044, 0.047)),
        material=powder,
        name="support_0",
    )
    frame.visual(
        Box((0.024, 0.105, 0.055)),
        origin=Origin(xyz=(0.110, 0.044, 0.047)),
        material=powder,
        name="support_1",
    )

    # Fixed hinge leaf and alternating center knuckle on the frame side.
    frame.visual(
        Box((0.019, 0.004, 0.266)),
        origin=Origin(xyz=(-0.139, -0.014, 0.215)),
        material=hardware,
        name="fixed_leaf",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.096),
        origin=Origin(xyz=(-0.128, -0.017, 0.215)),
        material=hardware,
        name="fixed_knuckle",
    )
    # Keeper on the latch side, just behind the closed door plane.
    frame.visual(
        Box((0.006, 0.012, 0.060)),
        origin=Origin(xyz=(0.133, -0.004, 0.215)),
        material=hardware,
        name="strike_lip",
    )

    panel = model.part("panel")
    # The panel part frame is the vertical hinge pin.  The panel extends along
    # local +X, so a negative Z hinge axis makes positive motion swing outward.
    panel.visual(
        Box((0.240, 0.010, 0.292)),
        origin=Origin(xyz=(0.132, 0.000, 0.000)),
        material=panel_paint,
        name="panel_shell",
    )
    panel.visual(
        Box((0.214, 0.003, 0.266)),
        origin=Origin(xyz=(0.139, -0.0067, 0.000)),
        material=rubber,
        name="front_reveal_gasket",
    )
    panel.visual(
        Box((0.020, 0.004, 0.260)),
        origin=Origin(xyz=(0.014, 0.000, 0.000)),
        material=hardware,
        name="moving_leaf",
    )
    panel.visual(
        Cylinder(radius=0.006, length=0.074),
        origin=Origin(xyz=(0.000, 0.000, 0.103)),
        material=hardware,
        name="upper_knuckle",
    )
    panel.visual(
        Cylinder(radius=0.006, length=0.074),
        origin=Origin(xyz=(0.000, 0.000, -0.103)),
        material=hardware,
        name="lower_knuckle",
    )
    panel.visual(
        Box((0.050, 0.004, 0.018)),
        origin=Origin(xyz=(0.246, -0.007, 0.000)),
        material=hardware,
        name="latch_label_plate",
    )

    latch = model.part("latch")
    # Quarter-turn service latch: front thumb paddle, shaft through the panel,
    # and rear cam that lines up with the strike lip when q=0.
    latch.visual(
        Cylinder(radius=0.004, length=0.028),
        origin=Origin(xyz=(0.000, -0.002, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=latch_color,
        name="shaft",
    )
    latch.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.000, -0.017, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=latch_color,
        name="thumb_disk",
    )
    latch.visual(
        Box((0.034, 0.008, 0.012)),
        origin=Origin(xyz=(0.000, -0.021, 0.000)),
        material=latch_color,
        name="thumb_paddle",
    )
    latch.visual(
        Box((0.040, 0.006, 0.014)),
        origin=Origin(xyz=(0.001, 0.011, 0.000)),
        material=hardware,
        name="cam",
    )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(-0.128, -0.017, 0.215)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=2.75),
    )
    model.articulation(
        "latch_turn",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=latch,
        origin=Origin(xyz=(0.236, 0.000, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
    )
    door_hinge.meta["note"] = "Upper stop folds the access panel nearly flat to the hinge side for compact stowage."
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("panel")
    latch = object_model.get_part("latch")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_turn = object_model.get_articulation("latch_turn")

    ctx.allow_overlap(
        panel,
        latch,
        elem_a="panel_shell",
        elem_b="shaft",
        reason="The quarter-turn latch shaft intentionally passes through the service panel bushing.",
    )

    ctx.expect_within(
        latch,
        panel,
        axes="xz",
        inner_elem="shaft",
        outer_elem="panel_shell",
        margin=0.002,
        name="latch shaft is captured in the panel",
    )
    ctx.expect_overlap(
        latch,
        panel,
        axes="y",
        elem_a="shaft",
        elem_b="panel_shell",
        min_overlap=0.006,
        name="latch shaft actually passes through panel thickness",
    )
    ctx.expect_gap(
        frame,
        latch,
        axis="x",
        positive_elem="strike_lip",
        negative_elem="cam",
        min_gap=0.001,
        max_gap=0.008,
        name="closed latch cam sits beside strike lip",
    )
    ctx.expect_overlap(
        frame,
        latch,
        axes="z",
        elem_a="strike_lip",
        elem_b="cam",
        min_overlap=0.010,
        name="closed latch cam aligns vertically with strike",
    )

    with ctx.pose({latch_turn: math.pi / 2.0}):
        ctx.expect_gap(
            frame,
            latch,
            axis="x",
            positive_elem="strike_lip",
            negative_elem="cam",
            min_gap=0.012,
            name="turned latch cam clears the strike",
        )

    closed_shell = ctx.part_element_world_aabb(panel, elem="panel_shell")
    with ctx.pose({door_hinge: 2.75, latch_turn: math.pi / 2.0}):
        open_shell = ctx.part_element_world_aabb(panel, elem="panel_shell")
        ctx.check(
            "panel folds flat toward hinge side",
            closed_shell is not None
            and open_shell is not None
            and open_shell[1][0] < -0.105
            and open_shell[1][1] < -0.010,
            details=f"closed_shell={closed_shell}, open_shell={open_shell}",
        )

    return ctx.report()


object_model = build_object_model()
