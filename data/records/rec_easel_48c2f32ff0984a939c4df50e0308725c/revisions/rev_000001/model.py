import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Material,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artist_easel")

    wood = Material(name="wood", rgba=(0.6, 0.4, 0.2, 1.0))
    metal = Material(name="metal", rgba=(0.5, 0.5, 0.5, 1.0))
    black_plastic = Material(name="black_plastic", rgba=(0.1, 0.1, 0.1, 1.0))

    # Base
    base = model.part("base")
    base.visual(Box((0.06, 0.7, 0.08)), origin=Origin(xyz=(-0.251, 0, 0.06)), name="left_runner", material=wood)
    base.visual(Box((0.06, 0.7, 0.08)), origin=Origin(xyz=(0.251, 0, 0.06)), name="right_runner", material=wood)
    base.visual(Box((0.46, 0.06, 0.06)), origin=Origin(xyz=(0, 0, 0.05)), name="base_crossbar", material=wood)
    base.visual(Box((0.04, 0.12, 0.12)), origin=Origin(xyz=(-0.251, 0, 0.14)), name="left_bracket", material=wood)
    base.visual(Box((0.04, 0.12, 0.12)), origin=Origin(xyz=(0.251, 0, 0.14)), name="right_bracket", material=wood)
    base.visual(Cylinder(radius=0.08, length=0.04), origin=Origin(xyz=(-0.251, 0, 0.17), rpy=(0, 1.5708, 0)), name="left_bracket_disc", material=wood)
    base.visual(Cylinder(radius=0.08, length=0.04), origin=Origin(xyz=(0.251, 0, 0.17), rpy=(0, 1.5708, 0)), name="right_bracket_disc", material=wood)
    base.visual(Cylinder(radius=0.01, length=0.55), origin=Origin(xyz=(0, 0, 0.17), rpy=(0, 1.5708, 0)), name="pivot_bolt", material=metal)
    base.visual(Cylinder(radius=0.03, length=0.02), origin=Origin(xyz=(0.28, 0, 0.17), rpy=(0, 1.5708, 0)), name="tilt_knob", material=black_plastic)

    base.visual(Cylinder(radius=0.02, length=0.02), origin=Origin(xyz=(-0.251, 0.3, 0.01)), name="foot_fl", material=black_plastic)
    base.visual(Cylinder(radius=0.02, length=0.02), origin=Origin(xyz=(0.251, 0.3, 0.01)), name="foot_fr", material=black_plastic)
    base.visual(Cylinder(radius=0.02, length=0.02), origin=Origin(xyz=(-0.251, -0.3, 0.01)), name="foot_rl", material=black_plastic)
    base.visual(Cylinder(radius=0.02, length=0.02), origin=Origin(xyz=(0.251, -0.3, 0.01)), name="foot_rr", material=black_plastic)

    # Mast frame
    mast_frame = model.part("mast_frame")
    mast_frame.visual(Box((0.04, 0.04, 1.5)), origin=Origin(xyz=(-0.209, 0, 0.75)), name="left_stile", material=wood)
    mast_frame.visual(Box((0.04, 0.04, 1.5)), origin=Origin(xyz=(0.209, 0, 0.75)), name="right_stile", material=wood)
    mast_frame.visual(Cylinder(radius=0.08, length=0.04), origin=Origin(xyz=(-0.209, 0, 0), rpy=(0, 1.5708, 0)), name="left_hinge_disc", material=wood)
    mast_frame.visual(Cylinder(radius=0.08, length=0.04), origin=Origin(xyz=(0.209, 0, 0), rpy=(0, 1.5708, 0)), name="right_hinge_disc", material=wood)
    mast_frame.visual(Box((0.458, 0.04, 0.06)), origin=Origin(xyz=(0, 0, 0.15)), name="mast_bottom_crossbar", material=wood)
    mast_frame.visual(Box((0.458, 0.04, 0.06)), origin=Origin(xyz=(0, 0, 1.47)), name="mast_top_crossbar", material=wood)
    mast_frame.visual(Box((0.06, 0.06, 1.5)), origin=Origin(xyz=(0, 0.01, 0.75)), name="central_post", material=wood)

    model.articulation(
        "tilt_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=mast_frame,
        origin=Origin(xyz=(0, 0, 0.17)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(lower=-0.1, upper=0.5)
    )

    # Canvas tray
    canvas_tray = model.part("canvas_tray")
    canvas_tray.visual(Box((0.6, 0.12, 0.04)), origin=Origin(xyz=(0, 0.06, 0)), name="tray_shelf", material=wood)
    canvas_tray.visual(Box((0.15, 0.04, 0.15)), origin=Origin(xyz=(0, 0.02, 0.075)), name="tray_frontplate", material=wood)
    canvas_tray.visual(Box((0.15, 0.02, 0.15)), origin=Origin(xyz=(0, -0.07, 0.075)), name="tray_rearplate", material=wood)
    canvas_tray.visual(Cylinder(radius=0.005, length=0.11), origin=Origin(xyz=(0.045, -0.035, 0.075), rpy=(1.5708, 0, 0)), name="tray_bolt_r", material=metal)
    canvas_tray.visual(Cylinder(radius=0.005, length=0.11), origin=Origin(xyz=(-0.045, -0.035, 0.075), rpy=(1.5708, 0, 0)), name="tray_bolt_l", material=metal)
    canvas_tray.visual(Cylinder(radius=0.02, length=0.02), origin=Origin(xyz=(0, 0.05, 0.075), rpy=(1.5708, 0, 0)), name="tray_knob", material=black_plastic)

    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=mast_frame,
        child=canvas_tray,
        origin=Origin(xyz=(0, 0.04, 0.2)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0.0, upper=0.8)
    )

    # Top clamp
    top_clamp = model.part("top_clamp")
    top_clamp.visual(Box((0.12, 0.04, 0.15)), origin=Origin(xyz=(0, 0.02, 0.075)), name="clamp_frontplate", material=wood)
    top_clamp.visual(Box((0.12, 0.02, 0.15)), origin=Origin(xyz=(0, -0.07, 0.075)), name="clamp_rearplate", material=wood)
    top_clamp.visual(Cylinder(radius=0.005, length=0.11), origin=Origin(xyz=(0.045, -0.035, 0.075), rpy=(1.5708, 0, 0)), name="clamp_bolt_r", material=metal)
    top_clamp.visual(Cylinder(radius=0.005, length=0.11), origin=Origin(xyz=(-0.045, -0.035, 0.075), rpy=(1.5708, 0, 0)), name="clamp_bolt_l", material=metal)
    top_clamp.visual(Cylinder(radius=0.02, length=0.02), origin=Origin(xyz=(0, 0.05, 0.075), rpy=(1.5708, 0, 0)), name="clamp_knob", material=black_plastic)
    top_clamp.visual(Box((0.12, 0.08, 0.02)), origin=Origin(xyz=(0, 0.04, -0.01)), name="clamp_lip", material=wood)

    model.articulation(
        "clamp_slide",
        ArticulationType.PRISMATIC,
        parent=mast_frame,
        child=top_clamp,
        origin=Origin(xyz=(0, 0.04, 0.8)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(lower=0.0, upper=0.6)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    mast_frame = object_model.get_part("mast_frame")
    canvas_tray = object_model.get_part("canvas_tray")
    top_clamp = object_model.get_part("top_clamp")

    ctx.allow_overlap(base, mast_frame, reason="Pivot bolt passes through mast frame hinge discs.")
    ctx.allow_overlap(mast_frame, canvas_tray, reason="Tray slides flush on the central post.")
    ctx.allow_overlap(mast_frame, top_clamp, reason="Clamp slides flush on the central post.")

    ctx.expect_within(canvas_tray, mast_frame, axes="x", inner_elem="tray_frontplate", outer_elem="mast_bottom_crossbar", margin=0.0)
    ctx.expect_contact(canvas_tray, mast_frame, elem_a="tray_frontplate", elem_b="central_post", contact_tol=0.001)

    ctx.expect_within(top_clamp, mast_frame, axes="x", inner_elem="clamp_frontplate", outer_elem="mast_bottom_crossbar", margin=0.0)
    ctx.expect_contact(top_clamp, mast_frame, elem_a="clamp_frontplate", elem_b="central_post", contact_tol=0.001)

    ctx.expect_gap(base, mast_frame, axis="x", positive_elem="right_bracket_disc", negative_elem="right_hinge_disc", min_gap=0.001, max_gap=0.003)

    return ctx.report()

object_model = build_object_model()