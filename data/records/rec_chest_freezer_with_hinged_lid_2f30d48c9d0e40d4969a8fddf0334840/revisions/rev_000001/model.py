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


BODY_W = 1.45
BODY_D = 0.70
BODY_H = 0.68
FOOT_H = 0.035
WALL_T = 0.055
LID_W = 1.51
LID_D = 0.76
LID_T = 0.13
PIN_Y = BODY_D / 2.0 + 0.030
PIN_Z = FOOT_H + BODY_H + 0.055


def _body_shell_shape() -> cq.Workplane:
    """Weatherproof insulated tub: rounded outer cabinet with an open top."""
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .edges("|Z")
        .fillet(0.045)
        .translate((0.0, 0.0, FOOT_H + BODY_H / 2.0))
    )

    cutter = (
        cq.Workplane("XY")
        .box(BODY_W - 2.0 * WALL_T, BODY_D - 2.0 * WALL_T, BODY_H + 0.18)
        .edges("|Z")
        .fillet(0.026)
        .translate((0.0, 0.0, FOOT_H + WALL_T + (BODY_H + 0.18) / 2.0))
    )
    return outer.cut(cutter)


def _rect_frame_shape(width: float, depth: float, tube: float, height: float, z_center: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(width, depth, height)
    inner = cq.Workplane("XY").box(width - 2.0 * tube, depth - 2.0 * tube, height + 0.02)
    return outer.cut(inner).translate((0.0, 0.0, z_center))


def _lid_shape() -> cq.Workplane:
    """Thick insulated lid authored around the hinge-pin child frame."""
    return (
        cq.Workplane("XY")
        .box(LID_W, LID_D, LID_T)
        .edges("|Z")
        .fillet(0.035)
        .edges(">Z")
        .fillet(0.018)
        .translate((0.0, -LID_D / 2.0 - 0.020, LID_T / 2.0 - 0.055))
    )


def _latch_arm_shape() -> cq.Workplane:
    """Flat swing arm in the local YZ plane, with a visible padlock eye."""
    plate = (
        cq.Workplane("YZ")
        .center(0.0, 0.090)
        .rect(0.055, 0.180)
        .extrude(0.010, both=True)
    )
    eye_cut = (
        cq.Workplane("YZ")
        .center(0.0, 0.153)
        .circle(0.012)
        .extrude(0.040, both=True)
    )
    grip_slot = (
        cq.Workplane("YZ")
        .center(0.0, 0.070)
        .rect(0.024, 0.046)
        .extrude(0.040, both=True)
    )
    return plate.cut(eye_cut).cut(grip_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_patio_chest_freezer")

    shell_white = model.material("powder_coated_white", rgba=(0.88, 0.91, 0.91, 1.0))
    liner_white = model.material("cooler_liner_white", rgba=(0.96, 0.98, 1.0, 1.0))
    gasket_black = model.material("black_rubber_gasket", rgba=(0.02, 0.025, 0.025, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.66, 0.69, 0.69, 1.0))
    dark = model.material("dark_plastic", rgba=(0.03, 0.035, 0.04, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "weatherproof_body_shell"),
        material=shell_white,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(
            _rect_frame_shape(BODY_W + 0.018, BODY_D + 0.018, 0.060, 0.006, FOOT_H + BODY_H - 0.003),
            "body_top_gasket",
        ),
        material=gasket_black,
        name="top_gasket",
    )
    body.visual(
        Box((BODY_W - 2.0 * WALL_T - 0.020, 0.010, 0.045)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + WALL_T + 0.005, FOOT_H + BODY_H - 0.025)),
        material=liner_white,
        name="front_liner_lip",
    )
    body.visual(
        Box((0.010, BODY_D - 2.0 * WALL_T, 0.045)),
        origin=Origin(xyz=(-BODY_W / 2.0 + WALL_T + 0.005, 0.0, FOOT_H + BODY_H - 0.025)),
        material=liner_white,
        name="side_liner_lip_0",
    )
    body.visual(
        Box((0.010, BODY_D - 2.0 * WALL_T, 0.045)),
        origin=Origin(xyz=(BODY_W / 2.0 - WALL_T - 0.005, 0.0, FOOT_H + BODY_H - 0.025)),
        material=liner_white,
        name="side_liner_lip_1",
    )
    body.visual(
        Box((BODY_W - 2.0 * WALL_T - 0.020, 0.010, 0.045)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - WALL_T - 0.005, FOOT_H + BODY_H - 0.025)),
        material=liner_white,
        name="rear_liner_lip",
    )

    # Small rubber feet are directly compressed against the cabinet bottom.
    for ix, x in enumerate((-BODY_W / 2.0 + 0.16, BODY_W / 2.0 - 0.16)):
        for iy, y in enumerate((-BODY_D / 2.0 + 0.12, BODY_D / 2.0 - 0.12)):
            body.visual(
                Box((0.12, 0.08, FOOT_H)),
                origin=Origin(xyz=(x, y, FOOT_H / 2.0)),
                material=dark,
                name=f"foot_{ix}_{iy}",
            )

    # Rear hinge hardware: two stainless barrel hinges, split into fixed and lid leaves.
    hinge_centers = (-0.42, 0.42)
    hinge_len = 0.22
    for i, x0 in enumerate(hinge_centers):
        body.visual(
            Box((hinge_len, 0.008, 0.105)),
            origin=Origin(xyz=(x0, BODY_D / 2.0 + 0.018, PIN_Z - 0.0695)),
            material=stainless,
            name=f"rear_hinge_leaf_{i}",
        )
        body.visual(
            Box((hinge_len, 0.030, 0.050)),
            origin=Origin(xyz=(x0, BODY_D / 2.0 + 0.015, FOOT_H + BODY_H - 0.040)),
            material=stainless,
            name=f"rear_hinge_standoff_{i}",
        )
        # Two fixed knuckles at the ends of each hinge.
        for j, dx in enumerate((-0.070, 0.070)):
            body.visual(
                Box((0.060, 0.040, 0.020)),
                origin=Origin(xyz=(x0 + dx, PIN_Y + 0.010, PIN_Z - 0.017)),
                material=stainless,
                name=f"rear_hinge_bridge_{i}_{j}",
            )
            body.visual(
                Cylinder(radius=0.017, length=0.060),
                origin=Origin(
                    xyz=(x0 + dx, PIN_Y, PIN_Z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=stainless,
                name=f"rear_hinge_knuckle_{i}_{j}",
            )

    # Fixed side hinge mounts for the two swing-arm latches.
    latch_y = -0.18
    latch_pivot_z = FOOT_H + BODY_H - 0.105
    latch_offset = 0.065
    for i, side in enumerate((-1.0, 1.0)):
        body.visual(
            Box((0.045, 0.090, 0.060)),
            origin=Origin(xyz=(side * (BODY_W / 2.0 + 0.0225), latch_y, latch_pivot_z)),
            material=stainless,
            name=f"latch_mount_{i}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(), "insulated_lid_shell"),
        material=shell_white,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(
            _rect_frame_shape(LID_W - 0.080, LID_D - 0.080, 0.040, 0.012, -0.049),
            "lid_underside_seal",
        ),
        origin=Origin(xyz=(0.0, -LID_D / 2.0 - 0.020, 0.0)),
        material=gasket_black,
        name="lid_seal",
    )
    lid.visual(
        Box((0.78, 0.026, 0.034)),
        origin=Origin(xyz=(0.0, -LID_D + 0.060, LID_T - 0.050)),
        material=dark,
        name="front_pull_handle",
    )
    for i, x0 in enumerate(hinge_centers):
        lid.visual(
            Box((hinge_len, 0.008, 0.105)),
            origin=Origin(xyz=(x0, 0.038, -0.025)),
            material=stainless,
            name=f"lid_hinge_leaf_{i}",
        )
        lid.visual(
            Box((hinge_len, 0.065, 0.035)),
            origin=Origin(xyz=(x0, 0.010, 0.045)),
            material=stainless,
            name=f"lid_hinge_standoff_{i}",
        )
        lid.visual(
            Box((0.065, 0.030, 0.020)),
            origin=Origin(xyz=(x0, 0.020, 0.0)),
            material=stainless,
            name=f"lid_hinge_bridge_{i}",
        )
        # Center moving knuckle interleaves between the two fixed rear knuckles.
        lid.visual(
            Cylinder(radius=0.0165, length=0.065),
            origin=Origin(xyz=(x0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name=f"lid_hinge_knuckle_{i}",
        )

    for i, side in enumerate((-1.0, 1.0)):
        lid.visual(
            Box((0.020, 0.090, 0.060)),
            origin=Origin(xyz=(side * (LID_W / 2.0 + 0.010), latch_y - PIN_Y, latch_pivot_z - PIN_Z + 0.158)),
            material=stainless,
            name=f"latch_keeper_{i}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, PIN_Y, PIN_Z)),
        # The closed lid extends along local -Y from the rear hinge line.
        # Positive motion about -X lifts the heavy free edge upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=90.0, velocity=0.9),
    )

    latch_mesh = mesh_from_cadquery(_latch_arm_shape(), "swing_latch_arm")
    for i, side in enumerate((-1.0, 1.0)):
        latch = model.part(f"side_latch_{i}")
        latch.visual(
            latch_mesh,
            material=stainless,
            name="latch_arm",
        )
        latch.visual(
            Cylinder(radius=0.018, length=0.040),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name="arm_barrel",
        )
        latch.visual(
            Cylinder(radius=0.015, length=0.006),
            origin=Origin(xyz=(side * 0.006, 0.0, 0.153), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name="lock_hole_shadow",
        )
        model.articulation(
            f"body_to_side_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=latch,
            origin=Origin(xyz=(side * (BODY_W / 2.0 + latch_offset), latch_y, latch_pivot_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=-1.25, upper=0.25, effort=12.0, velocity=2.5),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")

    # Closed lid: it covers the cabinet opening and its seal is seated on the rim.
    ctx.expect_overlap(lid, body, axes="xy", elem_a="lid_shell", elem_b="top_gasket", min_overlap=0.55)
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="top_gasket",
        max_gap=0.004,
        max_penetration=0.0,
        name="lid shell rests on weather gasket",
    )

    closed = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: 1.10}):
        opened = ctx.part_element_world_aabb(lid, elem="lid_shell")
    closed_center_z = None if closed is None else (closed[0][2] + closed[1][2]) / 2.0
    opened_center_z = None if opened is None else (opened[0][2] + opened[1][2]) / 2.0
    ctx.check(
        "lid opens upward about rear barrel hinges",
        closed_center_z is not None and opened_center_z is not None and opened_center_z > closed_center_z + 0.18,
        details=f"closed_center_z={closed_center_z}, opened_center_z={opened_center_z}",
    )

    for i in range(2):
        latch = object_model.get_part(f"side_latch_{i}")
        latch_joint = object_model.get_articulation(f"body_to_side_latch_{i}")
        ctx.expect_contact(
            latch,
            body,
            elem_a="arm_barrel",
            elem_b=f"latch_mount_{i}",
            contact_tol=0.002,
            name=f"side latch {i} barrel bears on stainless mount",
        )
        ctx.expect_overlap(
            latch,
            lid,
            axes="yz",
            elem_a="latch_arm",
            elem_b=f"latch_keeper_{i}",
            min_overlap=0.015,
            name=f"side latch {i} aligns with lid keeper",
        )
        rest_aabb = ctx.part_element_world_aabb(latch, elem="latch_arm")
        with ctx.pose({latch_joint: -0.90}):
            swung_aabb = ctx.part_element_world_aabb(latch, elem="latch_arm")
        rest_center_y = None if rest_aabb is None else (rest_aabb[0][1] + rest_aabb[1][1]) / 2.0
        swung_center_y = None if swung_aabb is None else (swung_aabb[0][1] + swung_aabb[1][1]) / 2.0
        ctx.check(
            f"side latch {i} swings on side revolute hinge",
            rest_center_y is not None and swung_center_y is not None and abs(swung_center_y - rest_center_y) > 0.035,
            details=f"rest_center_y={rest_center_y}, swung_center_y={swung_center_y}",
        )

    return ctx.report()


object_model = build_object_model()
