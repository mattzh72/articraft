from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


LID_HINGE_LIMIT = math.radians(75.0)


def _rect_ring(
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    height: float,
    *,
    z0: float = 0.0,
    fillet: float = 0.0,
) -> cq.Workplane:
    """Rectangular hollow ring, extruded upward from z0."""
    ring = cq.Workplane("XY").rect(outer_x, outer_y).rect(inner_x, inner_y).extrude(height)
    if fillet > 0.0:
        ring = ring.edges("|Z").fillet(fillet)
    return ring.translate((0.0, 0.0, z0))


def _lid_shell() -> cq.Workplane:
    """Raised hatch lid with front/side overhanging skirt; local hinge axis is y=0,z=0."""
    lid_w = 0.86
    lid_l = 0.62
    slab_t = 0.038
    slab_z = 0.006
    rear_clearance = 0.025
    center_y = rear_clearance + lid_l / 2.0

    slab = (
        cq.Workplane("XY")
        .box(lid_w, lid_l, slab_t)
        .translate((0.0, center_y, slab_z))
        .edges("|Z")
        .fillet(0.030)
    )

    skirt_t = 0.040
    skirt_h = 0.030
    skirt_z = -0.020
    side_l = lid_l - 0.010
    left_skirt = cq.Workplane("XY").box(skirt_t, side_l, skirt_h).translate(
        (-(lid_w - skirt_t) / 2.0, center_y + 0.005, skirt_z)
    )
    right_skirt = cq.Workplane("XY").box(skirt_t, side_l, skirt_h).translate(
        ((lid_w - skirt_t) / 2.0, center_y + 0.005, skirt_z)
    )
    front_skirt = cq.Workplane("XY").box(lid_w, skirt_t, skirt_h).translate(
        (0.0, rear_clearance + lid_l - skirt_t / 2.0, skirt_z)
    )

    crown = (
        cq.Workplane("XY")
        .box(0.58, 0.36, 0.014)
        .translate((0.0, center_y + 0.015, slab_z + slab_t / 2.0 + 0.006))
        .edges("|Z")
        .fillet(0.024)
    )

    return slab.union(left_skirt).union(right_skirt).union(front_skirt).union(crown)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_deck_hatch")

    frame_mat = model.material("powder_coated_aluminum", color=(0.58, 0.62, 0.62, 1.0))
    lid_mat = model.material("warm_white_fiberglass", color=(0.88, 0.90, 0.86, 1.0))
    gasket_mat = model.material("black_rubber_gasket", color=(0.015, 0.014, 0.012, 1.0))
    hinge_mat = model.material("brushed_stainless", color=(0.78, 0.78, 0.74, 1.0))
    latch_mat = model.material("dark_latch_hardware", color=(0.08, 0.085, 0.09, 1.0))

    frame_body = _rect_ring(0.96, 0.72, 0.60, 0.36, 0.035, fillet=0.018).union(
        _rect_ring(0.76, 0.52, 0.58, 0.34, 0.077, z0=0.035, fillet=0.012)
    )
    gasket = _rect_ring(0.68, 0.44, 0.58, 0.34, 0.012, z0=0.112, fillet=0.006)

    curb_frame = model.part("curb_frame")
    curb_frame.visual(
        mesh_from_cadquery(frame_body, "curb_frame_body", tolerance=0.0008),
        material=frame_mat,
        name="curb_body",
    )
    curb_frame.visual(
        mesh_from_cadquery(gasket, "curb_gasket", tolerance=0.0008),
        material=gasket_mat,
        name="gasket_ring",
    )
    curb_frame.visual(
        Box((0.84, 0.045, 0.090)),
        origin=Origin(xyz=(0.0, -0.344, 0.080)),
        material=frame_mat,
        name="rear_hinge_rail",
    )

    hinge_axis_y = -0.325
    hinge_axis_z = 0.140
    barrel_radius = 0.014
    parent_knuckles = [(-0.335, 0.110), (0.0, 0.110), (0.335, 0.110)]
    curb_frame.visual(
        Cylinder(radius=0.006, length=0.82),
        origin=Origin(
            xyz=(0.0, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hinge_mat,
        name="hinge_pin",
    )
    for index, (x_center, length) in enumerate(parent_knuckles):
        curb_frame.visual(
            Cylinder(radius=barrel_radius, length=length),
            origin=Origin(
                xyz=(x_center, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_mat,
            name=f"fixed_knuckle_{index}",
        )
        curb_frame.visual(
            Box((length, 0.044, 0.008)),
            origin=Origin(xyz=(x_center, hinge_axis_y - 0.020, hinge_axis_z - 0.017)),
            material=hinge_mat,
            name=f"fixed_leaf_{index}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "raised_lid_shell", tolerance=0.0008),
        material=lid_mat,
        name="lid_shell",
    )
    for index, x_pos in enumerate((-0.355, 0.355)):
        lid.visual(
            Box((0.095, 0.075, 0.010)),
            origin=Origin(xyz=(x_pos, 0.570, 0.030)),
            material=hinge_mat,
            name=f"latch_pad_{index}",
        )
    child_knuckles = [(-0.170, 0.150), (0.170, 0.150)]
    for index, (x_center, length) in enumerate(child_knuckles):
        lid.visual(
            Cylinder(radius=barrel_radius, length=length),
            origin=Origin(xyz=(x_center, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_mat,
            name=f"moving_knuckle_{index}",
        )
        lid.visual(
            Box((length, 0.044, 0.006)),
            origin=Origin(xyz=(x_center, 0.032, -0.008)),
            material=hinge_mat,
            name=f"moving_leaf_{index}",
        )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=curb_frame,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=LID_HINGE_LIMIT),
    )

    # Two low-profile quarter-turn latch blocks are mounted on the lid near the front corners.
    for index, (x_pos, block_offset_x) in enumerate(((-0.355, 0.030), (0.355, -0.030))):
        latch = model.part(f"latch_{index}")
        latch.visual(
            Cylinder(radius=0.022, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=hinge_mat,
            name="pivot_washer",
        )
        latch.visual(
            Box((0.105, 0.038, 0.018)),
            origin=Origin(xyz=(block_offset_x, 0.0, 0.015)),
            material=latch_mat,
            name="latch_block",
        )
        latch.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.025)),
            material=hinge_mat,
            name="pivot_cap",
        )
        model.articulation(
            f"latch_{index}_pivot",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=latch,
            origin=Origin(xyz=(x_pos, 0.570, 0.035)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=math.radians(90.0)),
        )

    # Keep a named local for readability; the articulation is stored by the model.
    _ = lid_hinge
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("curb_frame")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    for index in range(2):
        ctx.allow_overlap(
            frame,
            lid,
            elem_a="hinge_pin",
            elem_b=f"moving_knuckle_{index}",
            reason="The stainless hinge pin is intentionally captured inside the moving hinge barrel.",
        )
        ctx.expect_within(
            frame,
            lid,
            axes="yz",
            inner_elem="hinge_pin",
            outer_elem=f"moving_knuckle_{index}",
            margin=0.001,
            name=f"hinge pin is centered in moving knuckle {index}",
        )
        ctx.expect_overlap(
            frame,
            lid,
            axes="x",
            elem_a="hinge_pin",
            elem_b=f"moving_knuckle_{index}",
            min_overlap=0.12,
            name=f"hinge pin passes through moving knuckle {index}",
        )

    ctx.check(
        "rear hinge opens to about 75 degrees",
        abs(hinge.motion_limits.upper - LID_HINGE_LIMIT) < math.radians(1.0),
        details=f"upper={hinge.motion_limits.upper}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            frame,
            axes="xy",
            min_overlap=0.50,
            name="closed lid overlaps the curb frame footprint",
        )
        closed_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({hinge: LID_HINGE_LIMIT}):
        open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "front edge lifts high when hatch opens",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.45,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
