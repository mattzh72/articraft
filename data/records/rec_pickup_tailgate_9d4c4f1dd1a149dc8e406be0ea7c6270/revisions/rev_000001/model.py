from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pickup_tailgate")

    body_paint = model.material("body_paint", rgba=(0.18, 0.27, 0.52, 1.0))
    liner = model.material("liner", rgba=(0.11, 0.11, 0.12, 1.0))
    hardware = model.material("hardware", rgba=(0.30, 0.31, 0.33, 1.0))
    handle_plastic = model.material("handle_plastic", rgba=(0.08, 0.08, 0.09, 1.0))

    bed_frame = model.part("bed_frame")
    bed_frame.visual(
        Box((1.60, 0.86, 0.04)),
        origin=Origin(xyz=(0.0, 0.52, 0.02)),
        material=liner,
        name="bed_floor",
    )
    bed_frame.visual(
        Box((0.08, 0.95, 0.58)),
        origin=Origin(xyz=(-0.82, 0.475, 0.29)),
        material=body_paint,
        name="left_bedside",
    )
    bed_frame.visual(
        Box((0.08, 0.95, 0.58)),
        origin=Origin(xyz=(0.82, 0.475, 0.29)),
        material=body_paint,
        name="right_bedside",
    )
    bed_frame.visual(
        Box((0.09, 0.10, 0.09)),
        origin=Origin(xyz=(-0.835, 0.045, 0.045)),
        material=hardware,
        name="left_hinge_pedestal",
    )
    bed_frame.visual(
        Box((0.09, 0.10, 0.09)),
        origin=Origin(xyz=(0.835, 0.045, 0.045)),
        material=hardware,
        name="right_hinge_pedestal",
    )
    bed_frame.visual(
        Cylinder(radius=0.021, length=0.09),
        origin=Origin(xyz=(-0.793, 0.018, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="left_hinge_sleeve",
    )
    bed_frame.visual(
        Cylinder(radius=0.021, length=0.09),
        origin=Origin(xyz=(0.793, 0.018, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="right_hinge_sleeve",
    )
    bed_frame.visual(
        Box((0.030, 0.040, 0.060)),
        origin=Origin(xyz=(-0.775, 0.018, 0.505)),
        material=hardware,
        name="left_latch_striker",
    )
    bed_frame.visual(
        Box((0.030, 0.040, 0.060)),
        origin=Origin(xyz=(0.775, 0.018, 0.505)),
        material=hardware,
        name="right_latch_striker",
    )
    bed_frame.inertial = Inertial.from_geometry(
        Box((1.72, 0.95, 0.62)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.475, 0.20)),
    )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((1.51, 0.006, 0.55)),
        origin=Origin(xyz=(0.0, -0.027, 0.275)),
        material=body_paint,
        name="outer_skin",
    )
    tailgate.visual(
        Box((1.38, 0.026, 0.058)),
        origin=Origin(xyz=(0.0, -0.015, 0.031)),
        material=body_paint,
        name="bottom_rail",
    )
    tailgate.visual(
        Box((1.51, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, -0.016, 0.525)),
        material=body_paint,
        name="top_rail",
    )
    tailgate.visual(
        Box((0.055, 0.030, 0.444)),
        origin=Origin(xyz=(-0.727, -0.016, 0.280)),
        material=body_paint,
        name="left_side_rail",
    )
    tailgate.visual(
        Box((0.055, 0.030, 0.444)),
        origin=Origin(xyz=(0.727, -0.016, 0.280)),
        material=body_paint,
        name="right_side_rail",
    )
    tailgate.visual(
        Box((1.404, 0.004, 0.444)),
        origin=Origin(xyz=(0.0, -0.002, 0.280)),
        material=liner,
        name="inner_panel",
    )
    tailgate.visual(
        Box((1.30, 0.018, 0.100)),
        origin=Origin(xyz=(0.0, -0.008, 0.110)),
        material=liner,
        name="lower_reinforcement",
    )
    tailgate.visual(
        Box((0.10, 0.026, 0.145)),
        origin=Origin(xyz=(-0.690, -0.015, 0.0725)),
        material=liner,
        name="left_lower_corner_box",
    )
    tailgate.visual(
        Box((0.10, 0.026, 0.145)),
        origin=Origin(xyz=(0.690, -0.015, 0.0725)),
        material=liner,
        name="right_lower_corner_box",
    )
    tailgate.visual(
        Box((0.065, 0.036, 0.044)),
        origin=Origin(xyz=(-0.698, 0.001, 0.022)),
        material=hardware,
        name="left_hinge_arm",
    )
    tailgate.visual(
        Box((0.065, 0.036, 0.044)),
        origin=Origin(xyz=(0.698, 0.001, 0.022)),
        material=hardware,
        name="right_hinge_arm",
    )
    tailgate.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(-0.698, 0.018, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="left_hinge_knuckle",
    )
    tailgate.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(0.698, 0.018, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="right_hinge_knuckle",
    )
    tailgate.inertial = Inertial.from_geometry(
        Box((1.51, 0.065, 0.55)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Box((0.18, 0.016, 0.050)),
        origin=Origin(xyz=(0.0, -0.011, -0.025)),
        material=handle_plastic,
        name="handle_paddle",
    )
    latch_handle.visual(
        Box((0.13, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, -0.018, -0.039)),
        material=handle_plastic,
        name="handle_grip_ridge",
    )
    latch_handle.visual(
        Cylinder(radius=0.008, length=0.15),
        origin=Origin(xyz=(0.0, -0.006, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="handle_pivot_barrel",
    )
    latch_handle.inertial = Inertial.from_geometry(
        Box((0.18, 0.020, 0.050)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -0.010, -0.025)),
    )

    model.articulation(
        "bed_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=bed_frame,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.018, 0.025)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=1.6,
            lower=0.0,
            upper=1.52,
        ),
    )
    model.articulation(
        "tailgate_to_latch_handle",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=latch_handle,
        origin=Origin(xyz=(0.0, -0.030, 0.505)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=4.0,
            lower=0.0,
            upper=0.60,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bed_frame = object_model.get_part("bed_frame")
    tailgate = object_model.get_part("tailgate")
    latch_handle = object_model.get_part("latch_handle")
    tailgate_hinge = object_model.get_articulation("bed_to_tailgate")
    handle_hinge = object_model.get_articulation("tailgate_to_latch_handle")

    ctx.expect_gap(
        tailgate,
        bed_frame,
        axis="x",
        positive_elem="outer_skin",
        negative_elem="left_bedside",
        min_gap=0.015,
        max_gap=0.040,
        name="tailgate clears left bedside opening",
    )
    ctx.expect_gap(
        bed_frame,
        tailgate,
        axis="x",
        positive_elem="right_bedside",
        negative_elem="outer_skin",
        min_gap=0.015,
        max_gap=0.040,
        name="tailgate clears right bedside opening",
    )
    ctx.expect_gap(
        tailgate,
        latch_handle,
        axis="y",
        positive_elem="outer_skin",
        negative_elem="handle_paddle",
        min_gap=0.002,
        max_gap=0.006,
        name="closed latch handle sits slightly proud of the outer skin",
    )
    ctx.expect_origin_distance(
        tailgate,
        latch_handle,
        axes="x",
        min_dist=0.0,
        max_dist=0.001,
        name="latch handle is centered on the tailgate",
    )

    closed_tailgate_aabb = ctx.part_element_world_aabb(tailgate, elem="outer_skin")
    with ctx.pose({tailgate_hinge: 1.45}):
        open_tailgate_aabb = ctx.part_element_world_aabb(tailgate, elem="outer_skin")

    ctx.check(
        "tailgate drops rearward and downward when opened",
        closed_tailgate_aabb is not None
        and open_tailgate_aabb is not None
        and open_tailgate_aabb[1][2] < closed_tailgate_aabb[1][2] - 0.28
        and open_tailgate_aabb[0][1] < closed_tailgate_aabb[0][1] - 0.30,
        details=f"closed={closed_tailgate_aabb}, open={open_tailgate_aabb}",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(latch_handle, elem="handle_paddle")
    with ctx.pose({handle_hinge: 0.50}):
        opened_handle_aabb = ctx.part_element_world_aabb(latch_handle, elem="handle_paddle")

    ctx.check(
        "latch handle rotates outward from the tailgate skin",
        closed_handle_aabb is not None
        and opened_handle_aabb is not None
        and opened_handle_aabb[0][1] < closed_handle_aabb[0][1] - 0.010,
        details=f"closed={closed_handle_aabb}, open={opened_handle_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
