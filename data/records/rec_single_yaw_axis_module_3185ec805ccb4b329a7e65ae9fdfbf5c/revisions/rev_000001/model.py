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
)


BODY_X = 0.68
BODY_Y = 0.46
BODY_Z = 0.20
CARTRIDGE_RADIUS = 0.112
CARTRIDGE_HEIGHT = 0.12
PLATE_RADIUS = 0.19
PLATE_THICKNESS = 0.034
JOINT_Z = BODY_Z + CARTRIDGE_HEIGHT


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _translated_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _rounded_rect_profile(
    width: float, depth: float, radius: float, corner_segments: int = 8
) -> list[tuple[float, float]]:
    """Centered rounded-rectangle loop in the XY plane."""
    points: list[tuple[float, float]] = []
    corner_centers = (
        (width * 0.5 - radius, depth * 0.5 - radius, 0.0, pi * 0.5),
        (-width * 0.5 + radius, depth * 0.5 - radius, pi * 0.5, pi),
        (-width * 0.5 + radius, -depth * 0.5 + radius, pi, pi * 1.5),
        (width * 0.5 - radius, -depth * 0.5 + radius, pi * 1.5, pi * 2.0),
    )
    for cx, cy, a0, a1 in corner_centers:
        for step in range(corner_segments + 1):
            angle = a0 + (a1 - a0) * step / corner_segments
            points.append((cx + radius * cos(angle), cy + radius * sin(angle)))
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_service_yaw_stage")

    paint = model.material("matte_blue_gray", rgba=(0.20, 0.27, 0.31, 1.0))
    dark_paint = model.material("dark_service_panel", rgba=(0.06, 0.07, 0.08, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.012, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.68, 1.0))
    dark_steel = model.material("gunmetal", rgba=(0.22, 0.23, 0.23, 1.0))
    amber = model.material("amber_index", rgba=(0.95, 0.55, 0.10, 1.0))

    body = model.part("body")

    body_shell = ExtrudeGeometry(
        _rounded_rect_profile(BODY_X, BODY_Y, 0.035),
        BODY_Z,
        center=True,
    )
    body.visual(
        mesh_from_geometry(body_shell, "rounded_box_body"),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z * 0.5)),
        material=paint,
        name="box_body",
    )

    # Slightly raised access covers and foot pads make the base read as a
    # grounded service module, not a floating test block.
    body.visual(
        Box((0.52, 0.31, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z + 0.006)),
        material=dark_paint,
        name="top_cover",
    )
    body.visual(
        Box((0.34, 0.010, 0.085)),
        origin=Origin(xyz=(0.0, -BODY_Y * 0.5 - 0.001, 0.105)),
        material=dark_paint,
        name="front_panel",
    )
    body.visual(
        Box((0.13, 0.080, 0.024)),
        origin=Origin(xyz=(0.295, 0.205, 0.012)),
        material=black,
        name="foot_0",
    )
    body.visual(
        Box((0.13, 0.080, 0.024)),
        origin=Origin(xyz=(-0.295, 0.205, 0.012)),
        material=black,
        name="foot_1",
    )
    body.visual(
        Box((0.13, 0.080, 0.024)),
        origin=Origin(xyz=(0.295, -0.205, 0.012)),
        material=black,
        name="foot_2",
    )
    body.visual(
        Box((0.13, 0.080, 0.024)),
        origin=Origin(xyz=(-0.295, -0.205, 0.012)),
        material=black,
        name="foot_3",
    )

    body.visual(
        Cylinder(radius=0.158, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z + 0.015)),
        material=dark_steel,
        name="cartridge_flange",
    )
    body.visual(
        Cylinder(radius=CARTRIDGE_RADIUS, length=CARTRIDGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z + CARTRIDGE_HEIGHT * 0.5)),
        material=steel,
        name="cartridge",
    )

    for index, (x, y) in enumerate(
        ((0.205, 0.135), (-0.205, 0.135), (0.205, -0.135), (-0.205, -0.135))
    ):
        body.visual(
            Cylinder(radius=0.014, length=0.007),
            origin=Origin(xyz=(x, y, BODY_Z + 0.0155)),
            material=steel,
            name=f"cover_bolt_{index}",
        )

    output_plate = model.part("output_plate")
    hole_profile = _circle_profile(0.015, 24)
    bolt_holes = [
        _translated_profile(hole_profile, 0.132 * cos(2.0 * pi * i / 6.0), 0.132 * sin(2.0 * pi * i / 6.0))
        for i in range(6)
    ]
    plate_disk = ExtrudeWithHolesGeometry(
        _circle_profile(PLATE_RADIUS, 96),
        bolt_holes,
        PLATE_THICKNESS,
        center=True,
    )
    output_plate.visual(
        mesh_from_geometry(plate_disk, "round_output_plate"),
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS * 0.5)),
        material=dark_steel,
        name="plate_disk",
    )
    output_plate.visual(
        Cylinder(radius=0.062, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS + 0.009)),
        material=steel,
        name="center_pilot",
    )
    output_plate.visual(
        Cylinder(radius=0.012, length=0.009),
        origin=Origin(xyz=(0.142, 0.0, PLATE_THICKNESS + 0.004)),
        material=amber,
        name="index_pin",
    )

    model.articulation(
        "yaw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=output_plate,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.8, lower=-pi, upper=pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    plate = object_model.get_part("output_plate")
    yaw = object_model.get_articulation("yaw")
    limits = yaw.motion_limits

    ctx.check(
        "single vertical revolute yaw joint",
        len(object_model.articulations) == 1
        and yaw.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in yaw.axis) == (0.0, 0.0, 1.0),
        details=f"articulations={len(object_model.articulations)}, type={yaw.articulation_type}, axis={yaw.axis}",
    )
    ctx.check(
        "yaw joint allows a service half-turn",
        limits is not None and limits.lower is not None and limits.upper is not None and limits.lower <= -pi and limits.upper >= pi,
        details=f"limits={limits}",
    )

    ctx.expect_gap(
        plate,
        body,
        axis="z",
        positive_elem="plate_disk",
        negative_elem="cartridge",
        max_gap=0.001,
        max_penetration=0.0,
        name="output plate sits on exposed cartridge",
    )
    ctx.expect_overlap(
        plate,
        body,
        axes="xy",
        elem_a="plate_disk",
        elem_b="cartridge",
        min_overlap=CARTRIDGE_RADIUS * 1.8,
        name="plate is centered over cartridge footprint",
    )

    def _element_xy_center(part_name: str, elem_name: str) -> tuple[float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        return ((aabb[0][0] + aabb[1][0]) * 0.5, (aabb[0][1] + aabb[1][1]) * 0.5)

    rest_pin = _element_xy_center("output_plate", "index_pin")
    with ctx.pose({yaw: pi / 2.0}):
        turned_pin = _element_xy_center("output_plate", "index_pin")
    ctx.check(
        "output plate rotates about cartridge axis",
        rest_pin is not None
        and turned_pin is not None
        and rest_pin[0] > 0.12
        and abs(rest_pin[1]) < 0.02
        and abs(turned_pin[0]) < 0.02
        and turned_pin[1] > 0.12,
        details=f"rest_pin={rest_pin}, turned_pin={turned_pin}",
    )

    return ctx.report()


object_model = build_object_model()
