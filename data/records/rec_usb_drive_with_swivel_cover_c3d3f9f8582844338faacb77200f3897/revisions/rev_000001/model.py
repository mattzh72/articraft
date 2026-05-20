from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _circle_profile(radius: float, segments: int = 56) -> list[tuple[float, float]]:
    return [
        (cos(2.0 * pi * i / segments) * radius, sin(2.0 * pi * i / segments) * radius)
        for i in range(segments)
    ]


def _shift_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_usb_swivel_drive")

    graphite = model.material("painted_graphite_metal", rgba=(0.20, 0.21, 0.22, 1.0))
    satin_steel = model.material("satin_brushed_steel", rgba=(0.72, 0.73, 0.72, 1.0))
    bright_edge = model.material("polished_chamfer", rgba=(0.88, 0.87, 0.82, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.025, 0.027, 0.030, 1.0))
    soft_elastomer = model.material("soft_elastomer", rgba=(0.045, 0.047, 0.052, 1.0))
    usb_blue = model.material("usb_blue_tongue", rgba=(0.03, 0.17, 0.55, 1.0))
    contact_gold = model.material("contact_gold", rgba=(0.95, 0.72, 0.24, 1.0))

    body = model.part("body")

    body_shell = ExtrudeGeometry(
        superellipse_profile(0.048, 0.018, exponent=4.2, segments=72),
        0.008,
        center=True,
    )
    body.visual(
        mesh_from_geometry(body_shell, "body_shell"),
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
        material=graphite,
        name="body_shell",
    )
    body.visual(
        Box((0.032, 0.010, 0.00045)),
        origin=Origin(xyz=(-0.017, 0.0, 0.00418)),
        material=black_polymer,
        name="top_inlay",
    )
    body.visual(
        Box((0.020, 0.0192, 0.00038)),
        origin=Origin(xyz=(-0.031, 0.0, -0.00414)),
        material=soft_elastomer,
        name="grip_pad",
    )
    body.visual(
        Box((0.0012, 0.0188, 0.0084)),
        origin=Origin(xyz=(0.0094, 0.0, 0.0)),
        material=bright_edge,
        name="front_seam",
    )

    # A side-mounted pivot boss, shank, and broad head make the swivel mechanics visible.
    body.visual(
        Cylinder(radius=0.0061, length=0.0012),
        origin=Origin(xyz=(-0.018, 0.012, 0.00455)),
        material=satin_steel,
        name="pivot_boss",
    )
    body.visual(
        Box((0.012, 0.007, 0.0038)),
        origin=Origin(xyz=(-0.018, 0.0099, 0.0021)),
        material=graphite,
        name="pivot_web",
    )
    body.visual(
        Cylinder(radius=0.00225, length=0.00455),
        origin=Origin(xyz=(-0.018, 0.012, 0.00505)),
        material=bright_edge,
        name="pin_shank",
    )
    body.visual(
        Cylinder(radius=0.0046, length=0.0010),
        origin=Origin(xyz=(-0.018, 0.012, 0.00736)),
        material=bright_edge,
        name="pin_head",
    )

    # USB-A connector: four folded stainless walls, a blue polymer tongue, and contacts.
    body.visual(
        Box((0.0146, 0.0122, 0.00055)),
        origin=Origin(xyz=(0.0167, 0.0, 0.00225)),
        material=satin_steel,
        name="connector_top",
    )
    body.visual(
        Box((0.0146, 0.0122, 0.00055)),
        origin=Origin(xyz=(0.0167, 0.0, -0.00225)),
        material=satin_steel,
        name="connector_bottom",
    )
    body.visual(
        Box((0.0146, 0.00055, 0.0045)),
        origin=Origin(xyz=(0.0167, 0.00585, 0.0)),
        material=satin_steel,
        name="connector_side_0",
    )
    body.visual(
        Box((0.0146, 0.00055, 0.0045)),
        origin=Origin(xyz=(0.0167, -0.00585, 0.0)),
        material=satin_steel,
        name="connector_side_1",
    )
    body.visual(
        Box((0.0030, 0.0132, 0.0058)),
        origin=Origin(xyz=(0.0098, 0.0, 0.0)),
        material=satin_steel,
        name="connector_shoulder",
    )
    body.visual(
        Box((0.0123, 0.0062, 0.00115)),
        origin=Origin(xyz=(0.0170, 0.0, -0.00055)),
        material=usb_blue,
        name="connector_tongue",
    )
    for index, y in enumerate((-0.00225, -0.00075, 0.00075, 0.00225)):
        body.visual(
            Box((0.0043, 0.00082, 0.00020)),
            origin=Origin(xyz=(0.0202, y, 0.00008)),
            material=contact_gold,
            name=f"contact_{index}",
        )
    for index, y in enumerate((-0.0033, 0.0033)):
        body.visual(
            Box((0.0037, 0.0012, 0.00018)),
            origin=Origin(xyz=(0.0153, y, 0.00258)),
            material=bright_edge,
            name=f"shell_latch_{index}",
        )

    body.visual(
        Box((0.004, 0.003, 0.0007)),
        origin=Origin(xyz=(0.002, -0.0066, 0.00430)),
        material=soft_elastomer,
        name="closed_bumper",
    )

    cover = model.part("cover")
    cover_outer = _shift_profile(
        rounded_rect_profile(0.070, 0.029, 0.0048, corner_segments=10),
        0.026,
        -0.009,
    )
    cover_hole = _circle_profile(0.0032, segments=64)
    cover_plate = ExtrudeWithHolesGeometry(
        cover_outer,
        [cover_hole],
        0.0016,
        center=True,
    )
    cover.visual(
        mesh_from_geometry(cover_plate, "cover_plate"),
        material=satin_steel,
        name="cover_plate",
    )
    cover.visual(
        Box((0.056, 0.0012, 0.0062)),
        origin=Origin(xyz=(0.032, -0.0230, -0.0023)),
        material=satin_steel,
        name="inner_skirt",
    )
    cover.visual(
        Box((0.052, 0.0012, 0.0062)),
        origin=Origin(xyz=(0.034, 0.0050, -0.0023)),
        material=satin_steel,
        name="outer_skirt",
    )
    cover.visual(
        Box((0.0014, 0.0272, 0.0062)),
        origin=Origin(xyz=(0.0602, -0.0090, -0.0023)),
        material=satin_steel,
        name="cover_nose",
    )
    cover.visual(
        Box((0.024, 0.009, 0.00035)),
        origin=Origin(xyz=(0.034, -0.009, 0.00094)),
        material=bright_edge,
        name="brushed_highlight",
    )

    model.articulation(
        "cover_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(-0.018, 0.012, 0.0060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=3.0, lower=0.0, upper=2.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    pivot = object_model.get_articulation("cover_pivot")

    ctx.allow_overlap(
        body,
        cover,
        elem_a="pin_shank",
        elem_b="cover_plate",
        reason=(
            "The visible pivot shank is intentionally represented as a captured "
            "pin passing through the cover's pivot bore."
        ),
    )

    ctx.expect_gap(
        cover,
        body,
        axis="z",
        positive_elem="cover_plate",
        negative_elem="body_shell",
        min_gap=0.0005,
        max_gap=0.0025,
        name="cover plate clears the painted body",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        elem_a="cover_plate",
        elem_b="connector_top",
        min_overlap=0.008,
        name="closed cover spans the usb connector",
    )
    ctx.expect_gap(
        body,
        cover,
        axis="z",
        positive_elem="pin_head",
        negative_elem="cover_plate",
        min_gap=0.0,
        max_gap=0.0002,
        name="pin head captures the swivel cover without collision",
    )
    ctx.expect_overlap(
        body,
        cover,
        axes="z",
        elem_a="pin_shank",
        elem_b="cover_plate",
        min_overlap=0.001,
        name="pin shank passes through the cover thickness",
    )
    ctx.expect_overlap(
        body,
        cover,
        axes="xy",
        elem_a="pin_head",
        elem_b="cover_plate",
        min_overlap=0.004,
        name="visible pin head is centered over the cover pivot",
    )

    closed_aabb = ctx.part_element_world_aabb(cover, elem="cover_nose")
    with ctx.pose({pivot: 2.2}):
        open_aabb = ctx.part_element_world_aabb(cover, elem="cover_nose")
    ctx.check(
        "swivel cover swings away from connector",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] > closed_aabb[1][1] + 0.020,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
