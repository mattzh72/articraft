from __future__ import annotations

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


BODY_W = 0.66
BODY_D = 0.48
BODY_H = 0.105
HINGE_Y = 0.256
HINGE_Z = 0.135


def _scanner_body_shell() -> cq.Workplane:
    """One-piece shallow tray with a real open platen well."""

    floor = 0.016
    cavity_w = 0.540
    cavity_d = 0.355
    cavity_y = -0.015

    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H).translate((0, 0, BODY_H / 2))
    cutter = (
        cq.Workplane("XY")
        .box(cavity_w, cavity_d, BODY_H - floor + 0.020)
        .translate((0, cavity_y, floor + (BODY_H - floor + 0.020) / 2))
    )
    shell = outer.cut(cutter)
    # Small scanner-case radii: enough to read as molded plastic without
    # softening the square flatbed geometry.
    return shell.edges("|Z").fillet(0.010)


def _lid_cover() -> cq.Workplane:
    """Lid panel authored in the hinge frame; it extends forward along -Y."""

    lid_w = 0.670
    lid_d = 0.490
    lid_t = 0.028
    cover = cq.Workplane("XY").box(lid_w, lid_d, lid_t).translate((0, -lid_d / 2, -0.006))
    return cover.edges("|Z").fillet(0.014)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="a3_flatbed_scanner")

    warm_gray = model.material("warm_gray_plastic", rgba=(0.74, 0.74, 0.70, 1.0))
    dark_gray = model.material("dark_gray_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.016, 0.017, 1.0))
    glass_blue = model.material("slightly_blue_glass", rgba=(0.62, 0.82, 0.95, 0.38))
    white = model.material("matte_white_backing", rgba=(0.93, 0.93, 0.88, 1.0))
    scan_blue = model.material("scan_lamp_blue", rgba=(0.12, 0.72, 1.0, 1.0))
    metal = model.material("brushed_metal", rgba=(0.55, 0.56, 0.54, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_scanner_body_shell(), "body_shell", tolerance=0.0008),
        material=warm_gray,
        name="body_shell",
    )
    body.visual(
        Box((0.56, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, -0.221, BODY_H + 0.003)),
        material=dark_gray,
        name="front_control_strip",
    )
    for i, x in enumerate((-0.2645, 0.2645)):
        body.visual(
            Box((0.013, 0.355, 0.004)),
            origin=Origin(xyz=(x, -0.015, 0.099)),
            material=black,
            name=f"glass_side_lip_{i}",
        )
    for i, y in enumerate((-0.187, 0.157)):
        body.visual(
            Box((0.540, 0.013, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.099)),
            material=black,
            name=f"glass_end_lip_{i}",
        )
    for x in (-0.245, 0.245):
        body.visual(
            Box((0.012, 0.340, 0.008)),
            origin=Origin(xyz=(x, -0.015, 0.019)),
            material=metal,
            name=f"guide_rail_{0 if x < 0 else 1}",
        )
    # Two exposed rear hinge knuckle brackets. Each bracket has two static
    # cheeks flanking a moving barrel on the lid.
    for i, x in enumerate((-0.220, 0.220)):
        for side in (-1, 1):
            body.visual(
                Box((0.012, 0.030, 0.054)),
                origin=Origin(xyz=(x + side * 0.055, 0.277, 0.119)),
                material=dark_gray,
                name=f"hinge_cheek_{i}_{0 if side < 0 else 1}",
            )
            body.visual(
                Box((0.016, 0.052, 0.012)),
                origin=Origin(xyz=(x + side * 0.055, 0.258, 0.100)),
                material=dark_gray,
                name=f"hinge_foot_{i}_{0 if side < 0 else 1}",
            )

    platen = model.part("platen")
    platen.visual(
        Box((0.520, 0.335, 0.004)),
        origin=Origin(xyz=(0.0, -0.015, 0.103)),
        material=glass_blue,
        name="glass_sheet",
    )
    model.articulation(
        "body_to_platen",
        ArticulationType.FIXED,
        parent=body,
        child=platen,
        origin=Origin(),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_cover(), "lid_cover", tolerance=0.0008),
        material=warm_gray,
        name="cover",
    )
    lid.visual(
        Box((0.565, 0.365, 0.006)),
        origin=Origin(xyz=(0.0, -0.245, -0.0225)),
        material=white,
        name="backing_pad",
    )
    for i, x in enumerate((-0.270, 0.270)):
        lid.visual(
            Box((0.035, 0.018, 0.006)),
            origin=Origin(xyz=(x, -0.475, -0.021)),
            material=black,
            name=f"front_bumper_{i}",
        )
    lid.visual(
        Box((0.610, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, -0.004)),
        material=dark_gray,
        name="rear_spine",
    )
    for i, x in enumerate((-0.220, 0.220)):
        lid.visual(
            Cylinder(radius=0.016, length=0.085),
            origin=Origin(xyz=(x, 0.010, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
            material=dark_gray,
            name=f"hinge_barrel_{i}",
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        # The lid extends from the rear hinge line along local -Y; rotating
        # about -X raises the front edge away from the glass.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=12.0, velocity=1.5),
    )

    scan_head = model.part("scan_head")
    scan_head.visual(
        Box((0.485, 0.036, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_gray,
        name="carriage_body",
    )
    scan_head.visual(
        Box((0.430, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -0.004, 0.012)),
        material=scan_blue,
        name="lamp_window",
    )
    for x in (-0.245, 0.245):
        scan_head.visual(
            Box((0.024, 0.048, 0.024)),
            origin=Origin(xyz=(x, 0.0, -0.022)),
            material=black,
            name=f"slider_shoe_{0 if x < 0 else 1}",
        )
    model.articulation(
        "body_to_scan_head",
        ArticulationType.PRISMATIC,
        parent=body,
        child=scan_head,
        origin=Origin(xyz=(0.0, -0.150, 0.057)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.285, effort=30.0, velocity=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    platen = object_model.get_part("platen")
    scan_head = object_model.get_part("scan_head")
    lid_hinge = object_model.get_articulation("body_to_lid")
    scan_axis = object_model.get_articulation("body_to_scan_head")

    ctx.expect_gap(
        lid,
        platen,
        axis="z",
        positive_elem="backing_pad",
        negative_elem="glass_sheet",
        min_gap=0.001,
        max_gap=0.008,
        name="closed lid backing clears the glass",
    )
    ctx.expect_gap(
        platen,
        scan_head,
        axis="z",
        positive_elem="glass_sheet",
        negative_elem="lamp_window",
        min_gap=0.020,
        max_gap=0.040,
        name="scan lamp runs below the platen glass",
    )
    ctx.expect_within(
        scan_head,
        platen,
        axes="x",
        inner_elem="carriage_body",
        outer_elem="glass_sheet",
        margin=0.020,
        name="scan head spans inside the A3 glass width",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="cover")
    closed_pos = ctx.part_world_position(scan_head)
    with ctx.pose({lid_hinge: 1.15, scan_axis: 0.250}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="cover")
        moved_pos = ctx.part_world_position(scan_head)
        ctx.expect_within(
            scan_head,
            platen,
            axes="xy",
            inner_elem="carriage_body",
            outer_elem="glass_sheet",
            margin=0.030,
            name="scan head stays under the glass at rear travel",
        )
        ctx.expect_gap(
            platen,
            scan_head,
            axis="z",
            positive_elem="glass_sheet",
            negative_elem="lamp_window",
            min_gap=0.020,
            max_gap=0.040,
            name="scan lamp remains below glass at rear travel",
        )

    ctx.check(
        "lid revolute hinge raises the front cover",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.20,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )
    ctx.check(
        "scan head moves along the scan axis",
        closed_pos is not None and moved_pos is not None and moved_pos[1] > closed_pos[1] + 0.20,
        details=f"rest={closed_pos}, moved={moved_pos}",
    )

    return ctx.report()


object_model = build_object_model()
