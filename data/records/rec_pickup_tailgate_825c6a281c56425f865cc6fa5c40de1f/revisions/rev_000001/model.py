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
    model = ArticulatedObject(name="pickup_drop_tailgate")

    body_blue = Material("painted_body_blue", color=(0.07, 0.17, 0.32, 1.0))
    inner_blue = Material("stamped_inner_blue", color=(0.10, 0.23, 0.42, 1.0))
    dark_trim = Material("black_textured_trim", color=(0.015, 0.016, 0.018, 1.0))
    rubber = Material("dark_rubber", color=(0.02, 0.02, 0.018, 1.0))
    steel = Material("brushed_steel", color=(0.55, 0.57, 0.58, 1.0))
    work_top = Material("matte_work_surface", color=(0.19, 0.21, 0.20, 1.0))

    # Root is a connected rear-bed stub with a floor strip, sill, and the two
    # short bedside returns that the lower tailgate hinge sits between.
    bed_stubs = model.part("bed_stubs")
    bed_stubs.visual(
        Box((1.86, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.08, 0.01)),
        material=body_blue,
        name="lower_sill",
    )
    bed_stubs.visual(
        Box((1.74, 0.70, 0.055)),
        origin=Origin(xyz=(0.0, 0.43, 0.065)),
        material=inner_blue,
        name="bed_floor_stub",
    )
    for x, name in ((-0.89, "side_stub_0"), (0.89, "side_stub_1")):
        bed_stubs.visual(
            Box((0.14, 0.72, 0.62)),
            origin=Origin(xyz=(x, 0.43, 0.36)),
            material=body_blue,
            name=name,
        )
        bed_stubs.visual(
            Box((0.14, 0.06, 0.18)),
            origin=Origin(xyz=(x, 0.055, 0.075)),
            material=steel,
            name=f"hinge_clevis_{0 if x < 0 else 1}",
        )
        bed_stubs.visual(
            Cylinder(radius=0.024, length=0.10),
            origin=Origin(xyz=(x * 0.92, 0.018, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"corner_pin_{0 if x < 0 else 1}",
        )

    # Main tailgate frame.  Its part frame is the lower hinge axis.  The
    # closed gate rises along local +Z and drops rearward along -Y when opened.
    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((1.70, 0.036, 0.62)),
        origin=Origin(xyz=(0.0, -0.076, 0.33)),
        material=body_blue,
        name="outer_skin",
    )
    tailgate.visual(
        Box((1.70, 0.090, 0.090)),
        origin=Origin(xyz=(0.0, -0.026, 0.065)),
        material=inner_blue,
        name="bottom_inner_rail",
    )
    tailgate.visual(
        Box((1.70, 0.090, 0.080)),
        origin=Origin(xyz=(0.0, -0.026, 0.600)),
        material=inner_blue,
        name="top_inner_rail",
    )
    for x, name in ((-0.805, "side_inner_rail_0"), (0.805, "side_inner_rail_1")):
        tailgate.visual(
            Box((0.09, 0.090, 0.52)),
            origin=Origin(xyz=(x, -0.026, 0.335)),
            material=inner_blue,
            name=name,
        )
    tailgate.visual(
        Cylinder(radius=0.034, length=1.538),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="main_hinge_tube",
    )
    tailgate.visual(
        Box((1.26, 0.090, 0.035)),
        origin=Origin(xyz=(0.0, -0.026, 0.155)),
        material=inner_blue,
        name="work_pocket_lower_lip",
    )
    tailgate.visual(
        Box((1.26, 0.090, 0.035)),
        origin=Origin(xyz=(0.0, -0.026, 0.530)),
        material=inner_blue,
        name="work_pocket_upper_lip",
    )
    for x, name in ((-0.63, "work_pocket_side_0"), (0.63, "work_pocket_side_1")):
        tailgate.visual(
            Box((0.035, 0.090, 0.375)),
            origin=Origin(xyz=(x, -0.026, 0.342)),
            material=inner_blue,
            name=name,
        )
    for x, name in ((-0.43, "work_hinge_clip_0"), (0.43, "work_hinge_clip_1")):
        tailgate.visual(
            Box((0.075, 0.020, 0.070)),
            origin=Origin(xyz=(x, 0.018, 0.178)),
            material=steel,
            name=name,
        )
    tailgate.visual(
        Box((0.34, 0.006, 0.13)),
        origin=Origin(xyz=(0.0, -0.096, 0.470)),
        material=dark_trim,
        name="handle_recess",
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed_stubs,
        child=tailgate,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.57),
    )

    # Fold-down work panel on the inner face.  Its hinge line is inside the
    # stamped pocket; the hinge barrel overlaps the panel itself, while the
    # adjacent tailgate clips keep the panel visibly captured.
    work_panel = model.part("work_panel")
    work_panel.visual(
        Box((1.10, 0.026, 0.340)),
        origin=Origin(xyz=(0.0, 0.043, 0.180)),
        material=work_top,
        name="table_panel",
    )
    work_panel.visual(
        Cylinder(radius=0.018, length=1.16),
        origin=Origin(xyz=(0.0, 0.043, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="work_hinge_barrel",
    )
    work_panel.visual(
        Box((1.00, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.058, 0.345)),
        material=rubber,
        name="top_grip_edge",
    )
    work_panel.visual(
        Box((0.085, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, 0.061, 0.185)),
        material=dark_trim,
        name="finger_pull",
    )

    model.articulation(
        "tailgate_to_work_panel",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=work_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=0.0, upper=1.45),
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Box((0.260, 0.025, 0.075)),
        origin=Origin(xyz=(0.0, -0.0125, 0.0)),
        material=dark_trim,
        name="handle_grip",
    )
    latch_handle.visual(
        Cylinder(radius=0.035, length=0.034),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="handle_spindle",
    )

    model.articulation(
        "tailgate_to_latch_handle",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=latch_handle,
        origin=Origin(xyz=(0.0, -0.099, 0.470)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed_stubs = object_model.get_part("bed_stubs")
    tailgate = object_model.get_part("tailgate")
    work_panel = object_model.get_part("work_panel")
    latch_handle = object_model.get_part("latch_handle")
    gate_joint = object_model.get_articulation("bed_to_tailgate")
    work_joint = object_model.get_articulation("tailgate_to_work_panel")
    handle_joint = object_model.get_articulation("tailgate_to_latch_handle")

    # Captured pins and hinge clips are intentionally seated by a hair so the
    # model reads as mechanically connected rather than floating apart.
    for pin_name in ("corner_pin_0", "corner_pin_1"):
        ctx.allow_overlap(
            bed_stubs,
            tailgate,
            elem_a=pin_name,
            elem_b="main_hinge_tube",
            reason="The tailgate hinge tube is captured on the lower corner pin.",
        )
        ctx.expect_contact(
            bed_stubs,
            tailgate,
            elem_a=pin_name,
            elem_b="main_hinge_tube",
            contact_tol=0.002,
            name=f"{pin_name} contacts main hinge tube",
        )

    for clip_name in ("work_hinge_clip_0", "work_hinge_clip_1"):
        ctx.allow_overlap(
            tailgate,
            work_panel,
            elem_a=clip_name,
            elem_b="work_hinge_barrel",
            reason="The fold-down work panel barrel is locally clipped into the tailgate hinge bracket.",
        )
        ctx.expect_contact(
            tailgate,
            work_panel,
            elem_a=clip_name,
            elem_b="work_hinge_barrel",
            contact_tol=0.002,
            name=f"{clip_name} captures work panel hinge",
        )

    ctx.expect_within(
        work_panel,
        tailgate,
        axes="xz",
        inner_elem="table_panel",
        outer_elem="outer_skin",
        margin=0.0,
        name="work panel stays inside tailgate outline",
    )
    ctx.expect_gap(
        work_panel,
        tailgate,
        axis="y",
        min_gap=0.005,
        max_gap=0.020,
        positive_elem="table_panel",
        negative_elem="work_pocket_lower_lip",
        name="work surface sits proud of inner pocket",
    )
    ctx.expect_contact(
        latch_handle,
        tailgate,
        elem_a="handle_grip",
        elem_b="handle_recess",
        contact_tol=0.001,
        name="latch handle seats on outer recess",
    )

    def _coord(vec, index: int) -> float:
        return float(vec[index])

    closed_gate_aabb = ctx.part_world_aabb(tailgate)
    closed_work_aabb = ctx.part_world_aabb(work_panel)
    closed_handle_aabb = ctx.part_world_aabb(latch_handle)

    with ctx.pose({gate_joint: 1.35}):
        open_gate_aabb = ctx.part_world_aabb(tailgate)
    ctx.check(
        "main tailgate drops rearward and down",
        closed_gate_aabb is not None
        and open_gate_aabb is not None
        and _coord(open_gate_aabb[0], 1) < _coord(closed_gate_aabb[0], 1) - 0.30
        and _coord(open_gate_aabb[1], 2) < _coord(closed_gate_aabb[1], 2) - 0.30,
        details=f"closed={closed_gate_aabb}, open={open_gate_aabb}",
    )

    with ctx.pose({work_joint: 1.20}):
        open_work_aabb = ctx.part_world_aabb(work_panel)
    ctx.check(
        "work panel folds out from inner face",
        closed_work_aabb is not None
        and open_work_aabb is not None
        and _coord(open_work_aabb[1], 1) > _coord(closed_work_aabb[1], 1) + 0.20
        and _coord(open_work_aabb[1], 2) < _coord(closed_work_aabb[1], 2) - 0.20,
        details=f"closed={closed_work_aabb}, open={open_work_aabb}",
    )

    with ctx.pose({handle_joint: 0.70}):
        turned_handle_aabb = ctx.part_world_aabb(latch_handle)
    ctx.check(
        "latch handle rotates on face normal",
        closed_handle_aabb is not None
        and turned_handle_aabb is not None
        and (
            _coord(turned_handle_aabb[1], 2) - _coord(turned_handle_aabb[0], 2)
            > (_coord(closed_handle_aabb[1], 2) - _coord(closed_handle_aabb[0], 2)) + 0.10
        ),
        details=f"closed={closed_handle_aabb}, turned={turned_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
