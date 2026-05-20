from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _annular_plate_geometry(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    *,
    segments: int = 56,
) -> MeshGeometry:
    """Thin washer-like plate in the XZ plane with a real pin clearance hole."""
    geom = MeshGeometry()
    y_front = thickness * 0.5
    y_back = -thickness * 0.5

    outer_front: list[int] = []
    inner_front: list[int] = []
    outer_back: list[int] = []
    inner_back: list[int] = []
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        ca = math.cos(angle)
        sa = math.sin(angle)
        outer_front.append(geom.add_vertex(outer_radius * ca, y_front, outer_radius * sa))
        inner_front.append(geom.add_vertex(inner_radius * ca, y_front, inner_radius * sa))
        outer_back.append(geom.add_vertex(outer_radius * ca, y_back, outer_radius * sa))
        inner_back.append(geom.add_vertex(inner_radius * ca, y_back, inner_radius * sa))

    for i in range(segments):
        j = (i + 1) % segments
        _add_quad(geom, outer_front[i], outer_front[j], inner_front[j], inner_front[i])
        _add_quad(geom, outer_back[j], outer_back[i], inner_back[i], inner_back[j])
        _add_quad(geom, outer_back[i], outer_back[j], outer_front[j], outer_front[i])
        _add_quad(geom, inner_front[i], inner_front[j], inner_back[j], inner_back[i])

    return geom


def _build_cover_geometry() -> MeshGeometry:
    """Single connected mesh for the swivelling U-shaped protective cover."""
    plate_height = 0.012
    plate_thickness = 0.0015
    side_offset_y = 0.0106

    pivot_eye = _annular_plate_geometry(0.0058, 0.0028, plate_thickness)
    side_arm = BoxGeometry((0.0550, plate_thickness, plate_height)).translate(0.0305, 0.0, 0.0)

    cover = MeshGeometry()
    for y in (side_offset_y, -side_offset_y):
        cover.merge(pivot_eye.copy().translate(0.0, y, 0.0))
        cover.merge(side_arm.copy().translate(0.0, y, 0.0))

    # The end bridge ties the side cheeks together beyond the USB plug tip,
    # making the cover a real U-shaped yoke instead of two floating plates.
    bridge = BoxGeometry((0.0045, 0.0242, plate_height)).translate(0.058, 0.0, 0.0)
    cover.merge(bridge)

    return cover


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_drive_swivel_cover")

    body_black = model.material("slightly_soft_black_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    dark_gray = model.material("matte_charcoal_inset", rgba=(0.08, 0.085, 0.09, 1.0))
    chrome = model.material("brushed_usb_shell", rgba=(0.72, 0.73, 0.70, 1.0))
    gold = model.material("gold_contacts", rgba=(1.0, 0.72, 0.18, 1.0))
    cover_metal = model.material("satin_swivel_cover", rgba=(0.62, 0.65, 0.67, 1.0))

    body = model.part("body")

    body_shell = superellipse_side_loft(
        [
            (-0.020, -0.0032, 0.0032, 0.0135),
            (-0.017, -0.0040, 0.0040, 0.0170),
            (0.016, -0.0040, 0.0040, 0.0170),
            (0.020, -0.0033, 0.0033, 0.0140),
        ],
        exponents=3.0,
        segments=56,
        cap=True,
    ).rotate_z(-math.pi / 2.0)
    body.visual(
        mesh_from_geometry(body_shell, "rounded_usb_body"),
        material=body_black,
        name="body_shell",
    )

    # Short molded nose that mechanically seats the USB-A shell in the body.
    body.visual(
        Box((0.006, 0.0145, 0.0062)),
        origin=Origin(xyz=(0.0220, 0.0, 0.0)),
        material=body_black,
        name="connector_nose",
    )

    # A USB-A plug is a thin hollow metal sleeve, not a solid block.
    plug_center_x = 0.0305
    plug_len = 0.018
    body.visual(
        Box((plug_len, 0.0120, 0.00070)),
        origin=Origin(xyz=(plug_center_x, 0.0, 0.00235)),
        material=chrome,
        name="plug_top_wall",
    )
    body.visual(
        Box((plug_len, 0.0120, 0.00070)),
        origin=Origin(xyz=(plug_center_x, 0.0, -0.00235)),
        material=chrome,
        name="plug_bottom_wall",
    )
    body.visual(
        Box((plug_len, 0.00070, 0.0047)),
        origin=Origin(xyz=(plug_center_x, 0.00565, 0.0)),
        material=chrome,
        name="plug_side_wall_0",
    )
    body.visual(
        Box((plug_len, 0.00070, 0.0047)),
        origin=Origin(xyz=(plug_center_x, -0.00565, 0.0)),
        material=chrome,
        name="plug_side_wall_1",
    )
    body.visual(
        Box((0.0070, 0.0068, 0.0011)),
        origin=Origin(xyz=(0.0257, 0.0, -0.0004)),
        material=dark_gray,
        name="tongue_stem",
    )
    body.visual(
        Box((0.012, 0.0068, 0.0011)),
        origin=Origin(xyz=(0.0330, 0.0, -0.0004)),
        material=dark_gray,
        name="plug_tongue",
    )
    for i, y in enumerate((-0.0027, -0.0009, 0.0009, 0.0027)):
        body.visual(
            Box((0.0045, 0.00055, 0.00016)),
            origin=Origin(xyz=(0.0356, y, 0.00020)),
            material=gold,
            name=f"contact_{i}",
        )

    # Side pivot pin fixed to the body.  The rotating cover has matching holes
    # and is centered on the same body midline.
    body.visual(
        Cylinder(radius=0.00145, length=0.0240),
        origin=Origin(xyz=(-0.0120, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="pivot_pin",
    )
    for i, y in enumerate((-0.01175, 0.01175)):
        body.visual(
            Cylinder(radius=0.0045, length=0.0008),
            origin=Origin(xyz=(-0.0120, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"pivot_cap_{i}",
        )

    cover = model.part("cover")
    cover.visual(
        mesh_from_geometry(_build_cover_geometry(), "swivel_cover_yoke"),
        material=cover_metal,
        name="cover_yoke",
    )

    model.articulation(
        "body_to_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(-0.0120, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    swivel = object_model.get_articulation("body_to_cover")

    ctx.check(
        "cover uses a continuous swivel joint",
        swivel.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={swivel.articulation_type}",
    )
    ctx.check(
        "swivel axis is the body side-pin axis",
        tuple(round(v, 6) for v in swivel.axis) == (0.0, 1.0, 0.0),
        details=f"axis={swivel.axis}",
    )

    body_aabb = ctx.part_world_aabb(body)
    cover_aabb = ctx.part_world_aabb(cover)
    if body_aabb is not None and cover_aabb is not None:
        body_mid_y = (body_aabb[0][1] + body_aabb[1][1]) * 0.5
        cover_mid_y = (cover_aabb[0][1] + cover_aabb[1][1]) * 0.5
        ctx.check(
            "cover is centered on the body midline",
            abs(body_mid_y - cover_mid_y) < 0.0005,
            details=f"body_mid_y={body_mid_y:.5f}, cover_mid_y={cover_mid_y:.5f}",
        )
    else:
        ctx.fail("cover is centered on the body midline", "missing body or cover AABB")

    ctx.expect_overlap(
        cover,
        body,
        axes="x",
        min_overlap=0.030,
        elem_a="cover_yoke",
        elem_b="body_shell",
        name="cover cheeks run along the compact body",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="z",
        min_overlap=0.006,
        elem_a="cover_yoke",
        elem_b="body_shell",
        name="cover wraps the body thickness",
    )

    rest_pos = ctx.part_world_position(cover)
    with ctx.pose({swivel: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(cover)
        turned_aabb = ctx.part_world_aabb(cover)
        if body_aabb is not None and turned_aabb is not None:
            turned_mid_y = (turned_aabb[0][1] + turned_aabb[1][1]) * 0.5
            ctx.check(
                "cover stays centered while swivelling",
                abs(turned_mid_y - body_mid_y) < 0.0005,
                details=f"turned_mid_y={turned_mid_y:.5f}, body_mid_y={body_mid_y:.5f}",
            )
    ctx.check(
        "cover rotates about the fixed pivot without translating its origin",
        rest_pos is not None
        and turned_pos is not None
        and all(abs(rest_pos[i] - turned_pos[i]) < 1e-6 for i in range(3)),
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


object_model = build_object_model()
