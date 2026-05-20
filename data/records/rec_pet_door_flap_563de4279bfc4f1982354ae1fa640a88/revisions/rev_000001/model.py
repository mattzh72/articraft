from __future__ import annotations

import math

import cadquery as cq
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
    mesh_from_cadquery,
)


def _rounded_plate(width: float, height: float, thickness: float, radius: float) -> cq.Workplane:
    """A thin rounded rectangle in the X/Z plane, centered on local origin."""

    plate = (
        cq.Workplane("XY")
        # The installed CadQuery surface does not expose roundedRect; a clean
        # rectangular plate with separate rubber/magnet details still reads as a
        # rigid molded flap.
        .rect(width, height)
        .extrude(thickness)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0.0, thickness / 2.0, 0.0))
    )
    return plate


def _box_solid(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _pet_frame_body() -> cq.Workplane:
    """Fixed molded frame: two flanges joined by a short through-wall tunnel."""

    # Width X, depth Y, height Z.  The central void is deliberately open.
    boxes = [
        # Exterior/front flange around the flap.
        ((0.50, 0.018, 0.075), (0.0, -0.051, 0.277)),
        ((0.50, 0.018, 0.075), (0.0, -0.051, -0.277)),
        ((0.080, 0.018, 0.555), (-0.210, -0.051, 0.0)),
        ((0.080, 0.018, 0.555), (0.210, -0.051, 0.0)),
        # Interior flange that carries the cleaning-access trim cover.
        ((0.54, 0.018, 0.078), (0.0, 0.051, 0.295)),
        ((0.54, 0.018, 0.078), (0.0, 0.051, -0.295)),
        ((0.086, 0.018, 0.590), (-0.227, 0.051, 0.0)),
        ((0.086, 0.018, 0.590), (0.227, 0.051, 0.0)),
        # Through-wall sleeve surfaces, visible inside the pet opening.
        ((0.370, 0.088, 0.020), (0.0, 0.0, 0.250)),
        ((0.370, 0.088, 0.020), (0.0, 0.0, -0.250)),
        ((0.020, 0.088, 0.500), (-0.185, 0.0, 0.0)),
        ((0.020, 0.088, 0.500), (0.185, 0.0, 0.0)),
        # A raised hinge rail on the exterior upper edge.
        ((0.410, 0.014, 0.024), (0.0, -0.071, 0.274)),
    ]

    body = _box_solid(*boxes[0])
    for size, center in boxes[1:]:
        body = body.union(_box_solid(size, center))
    return body


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_pet_door")

    frame_plastic = Material("warm_white_plastic", rgba=(0.86, 0.82, 0.72, 1.0))
    darker_plastic = Material("shadow_gray_plastic", rgba=(0.16, 0.16, 0.15, 1.0))
    smoky_clear = Material("smoky_translucent_flap", rgba=(0.28, 0.36, 0.40, 0.46))
    black_rubber = Material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    screw_metal = Material("brushed_screw_caps", rgba=(0.55, 0.55, 0.50, 1.0))

    frame = model.part("outer_frame")
    frame.visual(
        mesh_from_cadquery(_pet_frame_body(), "outer_frame_body", tolerance=0.0012),
        material=frame_plastic,
        name="frame_body",
    )

    # Soft rubber stops/gaskets line the exterior opening without blocking it.
    for name, size, center in (
        ("top_stop", (0.330, 0.006, 0.010), (0.0, -0.062, 0.224)),
        ("bottom_stop", (0.330, 0.006, 0.010), (0.0, -0.062, -0.239)),
        ("side_stop_0", (0.010, 0.006, 0.460), (-0.169, -0.062, 0.0)),
        ("side_stop_1", (0.010, 0.006, 0.460), (0.169, -0.062, 0.0)),
    ):
        frame.visual(Box(size), origin=Origin(xyz=center), material=black_rubber, name=name)

    # Exposed screw caps on both trim flanges.
    for idx, (x, y, z) in enumerate(
        [
            (-0.205, -0.062, 0.250),
            (0.205, -0.062, 0.250),
            (-0.205, -0.062, -0.250),
            (0.205, -0.062, -0.250),
            (-0.225, 0.062, 0.270),
            (0.225, 0.062, 0.270),
            (-0.225, 0.062, -0.270),
            (0.225, 0.062, -0.270),
        ]
    ):
        frame.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=screw_metal,
            name=f"screw_cap_{idx}",
        )

    # Stationary lugs at the top hinge, separated from the moving flap barrel.
    for idx, x in enumerate((-0.190, 0.190)):
        frame.visual(
            Box((0.030, 0.026, 0.040)),
            origin=Origin(xyz=(x, -0.071, 0.246)),
            material=frame_plastic,
            name=f"flap_hinge_lug_{idx}",
        )

    # Alternating fixed knuckles for the interior side hinge.  The moving cover
    # supplies the intervening knuckles, making the hinge line visually legible.
    for idx, (z, length) in enumerate(((-0.235, 0.105), (0.0, 0.120), (0.235, 0.105))):
        frame.visual(
            Box((0.050, 0.008, length)),
            origin=Origin(xyz=(-0.266, 0.063, z)),
            material=frame_plastic,
            name=f"cover_hinge_leaf_{idx}",
        )
        frame.visual(
            Cylinder(radius=0.010, length=length),
            origin=Origin(xyz=(-0.285, 0.075, z)),
            material=frame_plastic,
            name=f"cover_hinge_knuckle_{idx}",
        )

    flap = model.part("flap")
    flap_panel = _rounded_plate(0.322, 0.430, 0.008, 0.030).translate((0.0, 0.0, -0.227))
    flap.visual(
        mesh_from_cadquery(flap_panel, "rigid_flap_panel", tolerance=0.0009),
        material=smoky_clear,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=0.012, length=0.332),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=darker_plastic,
        name="flap_hinge_barrel",
    )
    flap.visual(
        Box((0.260, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.438)),
        material=black_rubber,
        name="bottom_magnet",
    )

    trim_cover = model.part("trim_cover")
    # A hinged interior trim ring: the hinge side starts slightly to the right of
    # the hinge axis so the alternating barrel knuckles do not collide.
    cover_boxes = [
        ((0.520, 0.010, 0.046), (0.290, 0.0, 0.297)),
        ((0.520, 0.010, 0.046), (0.290, 0.0, -0.297)),
        ((0.052, 0.010, 0.594), (0.056, 0.0, 0.0)),
        ((0.052, 0.010, 0.594), (0.524, 0.0, 0.0)),
    ]
    cover_body = _box_solid(*cover_boxes[0])
    for size, center in cover_boxes[1:]:
        cover_body = cover_body.union(_box_solid(size, center))
    trim_cover.visual(
        mesh_from_cadquery(cover_body, "interior_trim_cover", tolerance=0.001),
        material=frame_plastic,
        name="cover_ring",
    )
    for idx, (z, length) in enumerate(((-0.118, 0.100), (0.118, 0.100))):
        trim_cover.visual(
            Cylinder(radius=0.010, length=length),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=frame_plastic,
            name=f"cover_knuckle_{idx}",
        )
        trim_cover.visual(
            Box((0.070, 0.006, length)),
            origin=Origin(xyz=(0.030, 0.0, z)),
            material=frame_plastic,
            name=f"cover_hinge_leaf_{idx}",
        )
    trim_cover.visual(
        Box((0.350, 0.015, 0.018)),
        origin=Origin(xyz=(0.290, -0.0075, -0.265)),
        material=black_rubber,
        name="cover_gasket",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0.0, -0.071, 0.246)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-1.0, upper=1.15),
    )

    model.articulation(
        "frame_to_trim_cover",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=trim_cover,
        origin=Origin(xyz=(-0.285, 0.075, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("outer_frame")
    flap = object_model.get_part("flap")
    trim_cover = object_model.get_part("trim_cover")
    flap_joint = object_model.get_articulation("frame_to_flap")
    cover_joint = object_model.get_articulation("frame_to_trim_cover")

    ctx.expect_gap(
        frame,
        flap,
        axis="y",
        min_gap=0.001,
        max_gap=0.006,
        positive_elem="top_stop",
        negative_elem="flap_panel",
        name="closed flap sits just proud of rubber stop",
    )
    ctx.expect_gap(
        trim_cover,
        frame,
        axis="y",
        min_gap=0.006,
        max_gap=0.025,
        positive_elem="cover_ring",
        negative_elem="frame_body",
        name="interior trim cover closes proud of frame",
    )

    panel_box = ctx.part_element_world_aabb(flap, elem="flap_panel")
    frame_box = ctx.part_element_world_aabb(frame, elem="frame_body")
    if panel_box is not None and frame_box is not None:
        p_min, p_max = panel_box
        f_min, f_max = frame_box
        ctx.check(
            "flap is centered within the pet opening",
            p_min[0] > -0.175
            and p_max[0] < 0.175
            and p_max[2] < 0.240
            and p_min[2] > -0.235,
            details=f"flap_panel={panel_box}, frame_body={frame_box}",
        )
    else:
        ctx.fail("flap is centered within the pet opening", "missing element AABB")

    rest_flap = ctx.part_element_world_aabb(flap, elem="bottom_magnet")
    with ctx.pose({flap_joint: 0.80}):
        open_flap = ctx.part_element_world_aabb(flap, elem="bottom_magnet")
    ctx.check(
        "top-hinged flap swings inward about horizontal axis",
        rest_flap is not None
        and open_flap is not None
        and open_flap[0][1] > rest_flap[0][1] + 0.18,
        details=f"rest={rest_flap}, open={open_flap}",
    )

    rest_cover = ctx.part_element_world_aabb(trim_cover, elem="cover_ring")
    with ctx.pose({cover_joint: 1.20}):
        open_cover = ctx.part_element_world_aabb(trim_cover, elem="cover_ring")
    ctx.check(
        "side-hinged trim cover swings into the interior",
        rest_cover is not None
        and open_cover is not None
        and open_cover[1][1] > rest_cover[1][1] + 0.25,
        details=f"rest={rest_cover}, open={open_cover}",
    )

    return ctx.report()


object_model = build_object_model()
