from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_LENGTH = 0.480
BODY_DEPTH = 0.320
BODY_HEIGHT = 0.058
OPENING_LENGTH = 0.392
OPENING_DEPTH = 0.240
LID_HINGE_Z = 0.063
LEFT_HINGE_X = -0.150
RIGHT_HINGE_X = 0.150
SCAN_TRAVEL = 0.190


def _scanner_body_shell() -> cq.Workplane:
    """Connected tray-like scanner body with a real central platen opening."""

    bottom_thickness = 0.014
    ring = (
        cq.Workplane("XY")
        .rect(BODY_LENGTH, BODY_DEPTH)
        .rect(OPENING_LENGTH, OPENING_DEPTH)
        .extrude(BODY_HEIGHT - bottom_thickness)
        .translate((0.0, 0.0, bottom_thickness))
    )
    bottom = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_DEPTH, bottom_thickness)
        .translate((0.0, 0.0, bottom_thickness / 2.0))
    )
    shell = bottom.union(ring)

    # Rounded outside and inside vertical corners make the slim plastic housing
    # read as an injection-molded scanner rather than a raw rectangular block.
    return shell.edges("|Z").fillet(0.010)


def _rounded_lid_panel() -> cq.Workplane:
    """Lid panel authored in the lid joint frame; hinge line is local y=0."""

    return (
        cq.Workplane("XY")
        .box(0.462, 0.300, 0.014)
        .edges("|Z")
        .fillet(0.010)
        .translate((0.150, -0.150, 0.012))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_flatbed_scanner")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.10, 0.105, 0.11, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.025, 0.025, 0.022, 1.0))
    platen_glass = model.material("pale_blue_glass", rgba=(0.70, 0.90, 1.0, 0.38))
    scan_green = model.material("ccd_green", rgba=(0.04, 0.45, 0.26, 1.0))
    optic_blue = model.material("optic_blue", rgba=(0.10, 0.42, 0.95, 1.0))
    metal = model.material("brushed_metal", rgba=(0.55, 0.56, 0.58, 1.0))
    status_green = model.material("status_green", rgba=(0.05, 0.95, 0.35, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_scanner_body_shell(), "body_shell"),
        material=charcoal,
        name="body_shell",
    )
    body.visual(
        Box((0.400, 0.248, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT + 0.001)),
        material=platen_glass,
        name="platen_glass",
    )
    body.visual(
        Box((0.190, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, -0.143, BODY_HEIGHT + 0.0015)),
        material=matte_black,
        name="front_status_strip",
    )
    body.visual(
        Box((0.010, 0.010, 0.002)),
        origin=Origin(xyz=(-0.075, -0.143, BODY_HEIGHT + 0.004)),
        material=status_green,
        name="status_led",
    )

    # Two steel guide rails carry the internal CCD scan head along the scan axis.
    body.visual(
        Cylinder(radius=0.003, length=0.260),
        origin=Origin(xyz=(-0.182, 0.0, 0.030), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="guide_rail_0",
    )
    body.visual(
        Cylinder(radius=0.003, length=0.260),
        origin=Origin(xyz=(0.182, 0.0, 0.030), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="guide_rail_1",
    )

    # Fixed hinge pads on the long rear edge; the left one carries the main lid
    # joint and the right one carries a mimicked second hinge leaf.
    body.visual(
        Box((0.066, 0.026, 0.006)),
        origin=Origin(xyz=(LEFT_HINGE_X, BODY_DEPTH / 2.0 - 0.005, 0.061)),
        material=matte_black,
        name="rear_hinge_pad_0",
    )
    body.visual(
        Box((0.066, 0.026, 0.006)),
        origin=Origin(xyz=(RIGHT_HINGE_X, BODY_DEPTH / 2.0 - 0.005, 0.061)),
        material=matte_black,
        name="rear_hinge_pad_1",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_rounded_lid_panel(), "lid_panel"),
        material=matte_black,
        name="lid_panel",
    )
    lid.visual(
        Box((0.385, 0.235, 0.003)),
        origin=Origin(xyz=(0.150, -0.150, 0.004)),
        material=Material("soft_white_pad", rgba=(0.88, 0.88, 0.82, 1.0)),
        name="white_backing_pad",
    )
    lid.visual(
        Box((0.055, 0.040, 0.004)),
        origin=Origin(xyz=(0.0, -0.020, 0.003)),
        material=matte_black,
        name="hinge_leaf_0",
    )
    lid.visual(
        Box((0.120, 0.008, 0.004)),
        origin=Origin(xyz=(0.150, -0.290, 0.006)),
        material=dark_rubber,
        name="front_grip",
    )

    hinge_leaf = model.part("hinge_leaf_1")
    hinge_leaf.visual(
        Box((0.055, 0.040, 0.004)),
        origin=Origin(xyz=(0.0, -0.020, 0.003)),
        material=matte_black,
        name="hinge_leaf",
    )

    scan_head = model.part("scan_head")
    scan_head.visual(
        Box((0.370, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=matte_black,
        name="carriage",
    )
    scan_head.visual(
        Box((0.285, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, -0.001, 0.0075)),
        material=optic_blue,
        name="lens_strip",
    )
    scan_head.visual(
        Box((0.318, 0.012, 0.003)),
        origin=Origin(xyz=(0.0, 0.011, 0.0075)),
        material=scan_green,
        name="ccd_board",
    )
    scan_head.visual(
        Box((0.012, 0.024, 0.004)),
        origin=Origin(xyz=(-0.182, 0.0, -0.002)),
        material=dark_rubber,
        name="guide_shoe_0",
    )
    scan_head.visual(
        Box((0.012, 0.024, 0.004)),
        origin=Origin(xyz=(0.182, 0.0, -0.002)),
        material=dark_rubber,
        name="guide_shoe_1",
    )

    main_hinge = model.articulation(
        "lid_hinge_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(LEFT_HINGE_X, BODY_DEPTH / 2.0 + 0.002, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.8, lower=0.0, upper=1.30),
    )
    model.articulation(
        "lid_hinge_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hinge_leaf,
        origin=Origin(xyz=(RIGHT_HINGE_X, BODY_DEPTH / 2.0 + 0.002, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.8, lower=0.0, upper=1.30),
        mimic=Mimic(joint=main_hinge.name),
    )
    model.articulation(
        "scan_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=scan_head,
        origin=Origin(xyz=(0.0, 0.095, 0.037)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.35, lower=0.0, upper=SCAN_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge_leaf = object_model.get_part("hinge_leaf_1")
    scan_head = object_model.get_part("scan_head")
    hinge_0 = object_model.get_articulation("lid_hinge_0")
    hinge_1 = object_model.get_articulation("lid_hinge_1")
    slide = object_model.get_articulation("scan_slide")

    ctx.check(
        "two rear revolute hinges",
        hinge_0.articulation_type == ArticulationType.REVOLUTE
        and hinge_1.articulation_type == ArticulationType.REVOLUTE
        and hinge_0.axis == hinge_1.axis == (-1.0, 0.0, 0.0)
        and abs(hinge_0.origin.xyz[1] - (BODY_DEPTH / 2.0 + 0.002)) < 1e-6
        and abs(hinge_1.origin.xyz[1] - (BODY_DEPTH / 2.0 + 0.002)) < 1e-6,
        details=f"hinge_0={hinge_0}, hinge_1={hinge_1}",
    )
    ctx.check(
        "right hinge mimics lid",
        hinge_1.mimic is not None and hinge_1.mimic.joint == "lid_hinge_0",
        details=f"mimic={hinge_1.mimic}",
    )
    ctx.check(
        "scan head is prismatic along scanner depth",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.axis == (0.0, -1.0, 0.0)
        and slide.motion_limits is not None
        and abs(slide.motion_limits.upper - SCAN_TRAVEL) < 1e-6,
        details=f"slide={slide}",
    )

    with ctx.pose({hinge_0: 0.0, slide: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.20,
            elem_a="lid_panel",
            elem_b="platen_glass",
            name="closed lid covers glass platen",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a="hinge_leaf_0",
            elem_b="rear_hinge_pad_0",
            contact_tol=0.001,
            name="left hinge leaf sits on rear hinge pad",
        )
        ctx.expect_contact(
            hinge_leaf,
            body,
            elem_a="hinge_leaf",
            elem_b="rear_hinge_pad_1",
            contact_tol=0.001,
            name="right hinge leaf sits on rear hinge pad",
        )
        ctx.expect_contact(
            hinge_leaf,
            lid,
            elem_a="hinge_leaf",
            elem_b="lid_panel",
            contact_tol=0.001,
            name="right hinge leaf reaches lid underside",
        )
        ctx.expect_within(
            scan_head,
            body,
            axes="xy",
            inner_elem="carriage",
            outer_elem="platen_glass",
            margin=0.012,
            name="parked scan head lies under glass",
        )
        ctx.expect_contact(
            scan_head,
            body,
            elem_a="guide_shoe_1",
            elem_b="guide_rail_1",
            contact_tol=0.0015,
            name="scan head rides the guide rail",
        )
        closed_lid_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
        parked_head = ctx.part_world_position(scan_head)

    with ctx.pose({hinge_0: 1.15, slide: SCAN_TRAVEL}):
        ctx.expect_within(
            scan_head,
            body,
            axes="xy",
            inner_elem="carriage",
            outer_elem="platen_glass",
            margin=0.012,
            name="extended scan head stays under glass",
        )
        open_lid_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
        extended_head = ctx.part_world_position(scan_head)

    ctx.check(
        "lid opens upward from rear edge",
        closed_lid_panel is not None
        and open_lid_panel is not None
        and open_lid_panel[1][2] > closed_lid_panel[1][2] + 0.12,
        details=f"closed={closed_lid_panel}, open={open_lid_panel}",
    )
    ctx.check(
        "scan head translates toward front",
        parked_head is not None
        and extended_head is not None
        and extended_head[1] < parked_head[1] - 0.15,
        details=f"parked={parked_head}, extended={extended_head}",
    )

    return ctx.report()


object_model = build_object_model()
