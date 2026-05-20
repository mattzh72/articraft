from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


def _tapered_digit_body(
    *,
    x0: float,
    x1: float,
    width0: float,
    width1: float,
    height0: float,
    height1: float,
    mesh_name: str,
):
    """Rounded rectangular tapered shell section, with the finger axis along +X."""
    profiles = []
    for t in (0.0, 0.5, 1.0):
        x = x0 + (x1 - x0) * t
        width = width0 + (width1 - width0) * t
        height = height0 + (height1 - height0) * t
        # A slight center bulge keeps the digit from reading as a flat wedge.
        if 0.0 < t < 1.0:
            width *= 1.03
            height *= 1.04
        yz_loop = superellipse_profile(width, height, exponent=3.0, segments=40)
        # LoftGeometry validates sections in the XY projection and lofts along
        # +Z.  Author the rounded section in that plane, then rotate so the
        # loft axis becomes the finger's +X axis.
        profiles.append([(-z, y, x) for y, z in yz_loop])

    geom = LoftGeometry(profiles, cap=True, closed=True)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, mesh_name)


def _add_hinge_barrel(
    part,
    *,
    name: str,
    x: float,
    y: float,
    radius: float,
    length: float,
    material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_digit_segment(
    part,
    *,
    length: float,
    body_width0: float,
    body_width1: float,
    body_height0: float,
    body_height1: float,
    barrel_radius: float,
    central_barrel_length: float,
    connector_width: float,
    side_barrel_y: float | None,
    side_barrel_length: float,
    side_cheek_width: float,
    mesh_name: str,
    shell_material,
    metal_material,
) -> None:
    """Add one mechanically connected digit link in a frame whose origin is its proximal knuckle."""
    _add_hinge_barrel(
        part,
        name="barrel",
        x=0.0,
        y=0.0,
        radius=barrel_radius,
        length=central_barrel_length,
        material=metal_material,
    )

    # The narrow bridge keeps the body physically tied to the central knuckle
    # while leaving clearance for the parent fork barrels on either side.
    part.visual(
        Box((0.022, connector_width, body_height0 * 0.58)),
        origin=Origin(xyz=(barrel_radius + 0.004, 0.0, 0.0)),
        material=shell_material,
        name="barrel_bridge",
    )

    body_start = barrel_radius + 0.004
    if side_barrel_y is None:
        body_end = length
    else:
        body_end = length - barrel_radius - 0.006

    part.visual(
        _tapered_digit_body(
            x0=body_start,
            x1=body_end,
            width0=body_width0,
            width1=body_width1,
            height0=body_height0,
            height1=body_height1,
            mesh_name=mesh_name,
        ),
        material=shell_material,
        name="body",
    )

    if side_barrel_y is not None:
        cheek_length = barrel_radius + 0.034
        cheek_center_x = length - cheek_length / 2.0 - 0.004
        for idx, y in enumerate((-side_barrel_y, side_barrel_y)):
            part.visual(
                Box((cheek_length, side_cheek_width, body_height1 * 0.74)),
                origin=Origin(xyz=(cheek_center_x, y, 0.0)),
                material=shell_material,
                name=f"fork_cheek_{idx}",
            )
            _add_hinge_barrel(
                part,
                name=f"fork_barrel_{idx}",
                x=length,
                y=y,
                radius=barrel_radius,
                length=side_barrel_length,
                material=metal_material,
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="broad_knuckle_finger")

    base_material = model.material("anodized_graphite", rgba=(0.10, 0.11, 0.12, 1.0))
    shell_material = model.material("warm_ivory_polymer", rgba=(0.86, 0.79, 0.66, 1.0))
    metal_material = model.material("brushed_steel", rgba=(0.52, 0.55, 0.58, 1.0))

    proximal_length = 0.085
    middle_length = 0.065
    distal_length = 0.052

    base = model.part("base")
    base.visual(
        Box((0.095, 0.115, 0.035)),
        origin=Origin(xyz=(-0.045, 0.0, -0.037)),
        material=base_material,
        name="palm_block",
    )
    for idx, y in enumerate((-0.0275, 0.0275)):
        base.visual(
            Box((0.030, 0.014, 0.046)),
            origin=Origin(xyz=(-0.003, y, -0.022)),
            material=base_material,
            name=f"base_yoke_{idx}",
        )
        _add_hinge_barrel(
            base,
            name=f"base_barrel_{idx}",
            x=0.0,
            y=y,
            radius=0.015,
            length=0.017,
            material=metal_material,
        )

    proximal = model.part("proximal")
    _add_digit_segment(
        proximal,
        length=proximal_length,
        body_width0=0.064,
        body_width1=0.057,
        body_height0=0.034,
        body_height1=0.029,
        barrel_radius=0.015,
        central_barrel_length=0.038,
        connector_width=0.034,
        side_barrel_y=0.0235,
        side_barrel_length=0.015,
        side_cheek_width=0.014,
        mesh_name="proximal_body",
        shell_material=shell_material,
        metal_material=metal_material,
    )

    middle = model.part("middle")
    _add_digit_segment(
        middle,
        length=middle_length,
        body_width0=0.055,
        body_width1=0.047,
        body_height0=0.029,
        body_height1=0.024,
        barrel_radius=0.013,
        central_barrel_length=0.032,
        connector_width=0.029,
        side_barrel_y=0.020,
        side_barrel_length=0.013,
        side_cheek_width=0.011,
        mesh_name="middle_body",
        shell_material=shell_material,
        metal_material=metal_material,
    )

    distal = model.part("distal")
    _add_digit_segment(
        distal,
        length=distal_length,
        body_width0=0.044,
        body_width1=0.030,
        body_height0=0.024,
        body_height1=0.017,
        barrel_radius=0.011,
        central_barrel_length=0.027,
        connector_width=0.024,
        side_barrel_y=None,
        side_barrel_length=0.0,
        side_cheek_width=0.0,
        mesh_name="distal_body",
        shell_material=shell_material,
        metal_material=metal_material,
    )

    model.articulation(
        "base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=proximal,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.5, velocity=2.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(proximal_length, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(middle_length, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [
        object_model.get_articulation("base_to_proximal"),
        object_model.get_articulation("proximal_to_middle"),
        object_model.get_articulation("middle_to_distal"),
    ]
    ctx.check(
        "three revolute knuckle joints",
        len(joints) == 3 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
    )
    ctx.check(
        "knuckle axes are parallel",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
    )

    proximal = object_model.get_part("proximal")
    middle = object_model.get_part("middle")
    distal = object_model.get_part("distal")

    def _part_extent(part, axis_index: int) -> float | None:
        bounds = ctx.part_world_aabb(part)
        if bounds is None:
            return None
        low, high = bounds
        return float(high[axis_index] - low[axis_index])

    lengths = [_part_extent(part, 0) for part in (proximal, middle, distal)]
    widths = [_part_extent(part, 1) for part in (proximal, middle, distal)]
    ctx.check(
        "digit segments decrease toward the tip",
        all(value is not None for value in lengths + widths)
        and lengths[0] > lengths[1] > lengths[2]
        and widths[0] > widths[1] > widths[2],
        details=f"lengths={lengths}, widths={widths}",
    )

    rest_distal_origin = ctx.part_world_position(distal)
    with ctx.pose(
        {
            joints[0]: 0.95,
            joints[1]: 1.00,
            joints[2]: 0.70,
        }
    ):
        curled_distal_origin = ctx.part_world_position(distal)
    ctx.check(
        "finger curls toward palm side",
        rest_distal_origin is not None
        and curled_distal_origin is not None
        and curled_distal_origin[2] < rest_distal_origin[2] - 0.060,
        details=f"rest={rest_distal_origin}, curled={curled_distal_origin}",
    )

    return ctx.report()


object_model = build_object_model()
