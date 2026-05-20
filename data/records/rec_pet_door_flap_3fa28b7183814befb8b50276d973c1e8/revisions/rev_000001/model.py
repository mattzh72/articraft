from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pet_door_flap")

    wall_mat = model.material("painted_wall", rgba=(0.76, 0.74, 0.68, 1.0))
    frame_mat = model.material("white_plastic_frame", rgba=(0.94, 0.92, 0.84, 1.0))
    gasket_mat = model.material("black_rubber_gasket", rgba=(0.02, 0.02, 0.018, 1.0))
    hinge_mat = model.material("brushed_hinge_pin", rgba=(0.55, 0.55, 0.52, 1.0))
    flap_mat = model.material("smoky_flexible_flap", rgba=(0.22, 0.16, 0.10, 0.62))
    rib_mat = model.material("darker_flap_edges", rgba=(0.08, 0.06, 0.045, 0.78))

    frame = model.part("frame")

    # Four wall pieces leave a real rectangular opening instead of a solid plate.
    frame.visual(
        Box((0.130, 0.050, 0.760)),
        origin=Origin(xyz=(-0.245, 0.000, 0.380)),
        material=wall_mat,
        name="wall_side_0",
    )
    frame.visual(
        Box((0.130, 0.050, 0.760)),
        origin=Origin(xyz=(0.245, 0.000, 0.380)),
        material=wall_mat,
        name="wall_side_1",
    )
    frame.visual(
        Box((0.364, 0.050, 0.160)),
        origin=Origin(xyz=(0.000, 0.000, 0.680)),
        material=wall_mat,
        name="wall_top",
    )
    frame.visual(
        Box((0.364, 0.050, 0.080)),
        origin=Origin(xyz=(0.000, 0.000, 0.040)),
        material=wall_mat,
        name="wall_bottom",
    )

    # Raised rigid perimeter frame fixed to the wall opening.
    frame.visual(
        Box((0.070, 0.035, 0.660)),
        origin=Origin(xyz=(-0.215, 0.0425, 0.370)),
        material=frame_mat,
        name="side_trim_0",
    )
    frame.visual(
        Box((0.070, 0.035, 0.660)),
        origin=Origin(xyz=(0.215, 0.0425, 0.370)),
        material=frame_mat,
        name="side_trim_1",
    )
    frame.visual(
        Box((0.500, 0.035, 0.070)),
        origin=Origin(xyz=(0.000, 0.0425, 0.635)),
        material=frame_mat,
        name="top_trim",
    )
    frame.visual(
        Box((0.500, 0.035, 0.070)),
        origin=Origin(xyz=(0.000, 0.0425, 0.045)),
        material=frame_mat,
        name="bottom_trim",
    )

    # Thin dark gasket lining the clear aperture, recessed behind the swinging flap.
    frame.visual(
        Box((0.010, 0.006, 0.520)),
        origin=Origin(xyz=(-0.175, 0.056, 0.340)),
        material=gasket_mat,
        name="gasket_side_0",
    )
    frame.visual(
        Box((0.010, 0.006, 0.520)),
        origin=Origin(xyz=(0.175, 0.056, 0.340)),
        material=gasket_mat,
        name="gasket_side_1",
    )
    frame.visual(
        Box((0.360, 0.006, 0.010)),
        origin=Origin(xyz=(0.000, 0.056, 0.595)),
        material=gasket_mat,
        name="gasket_top",
    )
    frame.visual(
        Box((0.360, 0.006, 0.010)),
        origin=Origin(xyz=(0.000, 0.056, 0.085)),
        material=gasket_mat,
        name="gasket_bottom",
    )

    # Fixed hinge knuckles on the upper frame, flanking the moving sleeve.
    for suffix, x in (("0", -0.190), ("1", 0.190)):
        frame.visual(
            Box((0.050, 0.024, 0.050)),
            origin=Origin(xyz=(x, 0.071, 0.585)),
            material=frame_mat,
            name=f"hinge_bracket_{suffix}",
        )
        frame.visual(
            Cylinder(radius=0.018, length=0.050),
            origin=Origin(xyz=(x, 0.085, 0.585), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_mat,
            name=f"hinge_knuckle_{suffix}",
        )

    flap = model.part("flap")
    flap.visual(
        Box((0.310, 0.012, 0.480)),
        origin=Origin(xyz=(0.000, 0.000, -0.258)),
        material=flap_mat,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.018, length=0.330),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=rib_mat,
        name="hinge_sleeve",
    )
    flap.visual(
        Box((0.310, 0.016, 0.035)),
        origin=Origin(xyz=(0.000, 0.000, -0.035)),
        material=rib_mat,
        name="top_clamp",
    )
    flap.visual(
        Box((0.018, 0.014, 0.440)),
        origin=Origin(xyz=(-0.146, 0.002, -0.277)),
        material=rib_mat,
        name="side_edge_0",
    )
    flap.visual(
        Box((0.018, 0.014, 0.440)),
        origin=Origin(xyz=(0.146, 0.002, -0.277)),
        material=rib_mat,
        name="side_edge_1",
    )
    flap.visual(
        Box((0.270, 0.016, 0.025)),
        origin=Origin(xyz=(0.000, 0.002, -0.485)),
        material=rib_mat,
        name="bottom_weight",
    )
    for index, x in enumerate((-0.090, 0.000, 0.090)):
        flap.visual(
            Box((0.010, 0.008, 0.330)),
            origin=Origin(xyz=(x, 0.010, -0.275)),
            material=rib_mat,
            name=f"flex_rib_{index}",
        )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.000, 0.085, 0.585)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.90, upper=1.10),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("top_hinge")

    ctx.check(
        "flap uses a horizontal upper revolute hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE and tuple(hinge.axis) == (1.0, 0.0, 0.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )
    limits = hinge.motion_limits
    ctx.check(
        "flap can swing inward and outward",
        limits is not None and limits.lower is not None and limits.upper is not None and limits.lower < -0.5 and limits.upper > 0.8,
        details=f"limits={limits}",
    )

    ctx.expect_gap(
        frame,
        flap,
        axis="x",
        positive_elem="side_trim_1",
        negative_elem="flap_panel",
        min_gap=0.015,
        name="flap clears the right side of the frame",
    )
    ctx.expect_gap(
        flap,
        frame,
        axis="x",
        positive_elem="flap_panel",
        negative_elem="side_trim_0",
        min_gap=0.015,
        name="flap clears the left side of the frame",
    )
    ctx.expect_gap(
        flap,
        frame,
        axis="z",
        positive_elem="flap_panel",
        negative_elem="bottom_trim",
        min_gap=0.002,
        name="flap hangs just above the bottom trim",
    )
    ctx.expect_overlap(
        flap,
        frame,
        axes="x",
        elem_a="hinge_sleeve",
        elem_b="top_trim",
        min_overlap=0.20,
        name="moving sleeve spans the upper hinge line",
    )

    closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({hinge: 0.80}):
        open_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "positive hinge rotation swings the flap outward and upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.20
        and open_aabb[0][2] > closed_aabb[0][2] + 0.08,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
