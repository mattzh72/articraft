from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _handle_shell() -> object:
    """Slim, hollow utility-knife handle with a blade tunnel and top slider slot."""

    width = 0.024
    outer_side_profile = [
        (-0.080, 0.003),
        (-0.073, 0.000),
        (0.054, 0.000),
        (0.080, 0.006),
        (0.083, 0.013),
        (0.080, 0.020),
        (0.057, 0.026),
        (-0.073, 0.026),
        (-0.080, 0.023),
    ]

    body = (
        cq.Workplane("XZ")
        .polyline(outer_side_profile)
        .close()
        .extrude(width)
        .translate((0.0, width / 2.0, 0.0))
    )

    # Rectangular blade/carriage tunnel, open at the nose but closed at the tail.
    blade_tunnel = cq.Workplane("XY").box(0.146, 0.012, 0.013).translate((0.012, 0.0, 0.011))
    body = body.cut(blade_tunnel)

    # Slot that lets the thumb slider's stalk reach the moving carriage.
    top_slot = cq.Workplane("XY").box(0.090, 0.008, 0.018).translate((-0.005, 0.0, 0.025))
    body = body.cut(top_slot)

    return body


def _xz_plate_mesh(points: list[tuple[float, float]], thickness: float) -> MeshGeometry:
    """Make a thin plate by extruding an X/Z polygon along local Y."""

    geom = MeshGeometry()
    half = thickness / 2.0
    front: list[int] = []
    back: list[int] = []
    for x, z in points:
        front.append(geom.add_vertex(x, half, z))
    for x, z in points:
        back.append(geom.add_vertex(x, -half, z))

    # Cap faces by fan triangulation; the blade outline is intentionally convex.
    for i in range(1, len(points) - 1):
        geom.add_face(front[0], front[i], front[i + 1])
        geom.add_face(back[0], back[i + 1], back[i])

    for i in range(len(points)):
        j = (i + 1) % len(points)
        geom.add_face(front[i], back[i], back[j])
        geom.add_face(front[i], back[j], front[j])

    return geom


def _blade_mesh() -> MeshGeometry:
    return _xz_plate_mesh(
        [
            (0.012, 0.0060),
            (0.047, 0.0060),
            (0.060, 0.0100),
            (0.047, 0.0140),
            (0.012, 0.0140),
        ],
        0.0016,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_utility_knife")

    matte_graphite = model.material("matte_graphite", color=(0.08, 0.085, 0.09, 1.0))
    dark_rubber = model.material("dark_rubber", color=(0.01, 0.012, 0.014, 1.0))
    satin_steel = model.material("satin_steel", color=(0.72, 0.72, 0.68, 1.0))
    blackened_steel = model.material("blackened_steel", color=(0.12, 0.12, 0.12, 1.0))
    safety_red = model.material("safety_red", color=(0.76, 0.05, 0.025, 1.0))

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_shell(), "handle_shell", tolerance=0.0006, angular_tolerance=0.08),
        material=matte_graphite,
        name="handle_shell",
    )

    # Rubberized side grip pads are seated into the shell surface.
    for side_name, y in (("side_grip_0", 0.0127), ("side_grip_1", -0.0127)):
        handle.visual(
            Box((0.055, 0.0022, 0.010)),
            origin=Origin(xyz=(-0.040, y, 0.0125)),
            material=dark_rubber,
            name=side_name,
        )

    # Exposed rivet heads and the latch bushing make the shell read as a rigid carrier.
    for i, x in enumerate((-0.066, 0.056)):
        handle.visual(
            Cylinder(radius=0.0034, length=0.0025),
            origin=Origin(xyz=(x, 0.0130, 0.0200), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_steel,
            name=f"rivet_{i}",
        )

    handle.visual(
        Cylinder(radius=0.0054, length=0.0026),
        origin=Origin(xyz=(-0.047, 0.01325, 0.0170), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blackened_steel,
        name="latch_bushing",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.070, 0.0090, 0.0048)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0100)),
        material=blackened_steel,
        name="carrier_plate",
    )
    for i, y in enumerate((0.0045, -0.0045)):
        carriage.visual(
            Box((0.055, 0.0030, 0.0030)),
            origin=Origin(xyz=(-0.020, y, 0.0110)),
            material=blackened_steel,
            name=f"guide_shoe_{i}",
        )
    carriage.visual(
        mesh_from_geometry(_blade_mesh(), "utility_blade"),
        material=satin_steel,
        name="blade",
    )
    carriage.visual(
        Box((0.010, 0.0055, 0.020)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0205)),
        material=blackened_steel,
        name="slider_stalk",
    )
    carriage.visual(
        Box((0.030, 0.015, 0.006)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0315)),
        material=dark_rubber,
        name="thumb_pad",
    )
    for i, x in enumerate((-0.030, -0.020, -0.010)):
        carriage.visual(
            Box((0.0030, 0.0155, 0.0020)),
            origin=Origin(xyz=(x, 0.0, 0.0355)),
            material=matte_graphite,
            name=f"thumb_ridge_{i}",
        )

    safety_latch = model.part("safety_latch")
    safety_latch.visual(
        Cylinder(radius=0.0047, length=0.0030),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=blackened_steel,
        name="latch_pivot",
    )
    safety_latch.visual(
        Box((0.034, 0.0032, 0.0070)),
        origin=Origin(xyz=(0.018, 0.0022, 0.0)),
        material=safety_red,
        name="latch_tab",
    )
    safety_latch.visual(
        Box((0.006, 0.0035, 0.0090)),
        origin=Origin(xyz=(0.033, 0.0024, 0.0)),
        material=safety_red,
        name="latch_tip",
    )

    model.articulation(
        "handle_to_carriage",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.35, lower=0.0, upper=0.032),
    )

    model.articulation(
        "handle_to_safety_latch",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=safety_latch,
        origin=Origin(xyz=(-0.047, 0.0155, 0.0170)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle")
    carriage = object_model.get_part("carriage")
    safety_latch = object_model.get_part("safety_latch")
    slide = object_model.get_articulation("handle_to_carriage")
    latch_joint = object_model.get_articulation("handle_to_safety_latch")

    ctx.allow_overlap(
        handle,
        safety_latch,
        elem_a="latch_bushing",
        elem_b="latch_pivot",
        reason="The latch pivot disk is intentionally captured on the side-wall bushing.",
    )
    ctx.expect_gap(
        safety_latch,
        handle,
        axis="y",
        positive_elem="latch_pivot",
        negative_elem="latch_bushing",
        max_penetration=0.0007,
        max_gap=0.0010,
        name="latch pivot is seated on the bushing",
    )
    ctx.expect_overlap(
        safety_latch,
        handle,
        axes="xz",
        elem_a="latch_pivot",
        elem_b="latch_bushing",
        min_overlap=0.006,
        name="latch pivot is coaxial with side bushing",
    )

    ctx.expect_gap(
        carriage,
        handle,
        axis="z",
        positive_elem="thumb_pad",
        negative_elem="handle_shell",
        min_gap=0.0005,
        max_gap=0.006,
        name="thumb pad rides above the handle top",
    )

    handle_box = ctx.part_element_world_aabb(handle, elem="handle_shell")
    blade_rest = ctx.part_element_world_aabb(carriage, elem="blade")
    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.032}):
        blade_extended = ctx.part_element_world_aabb(carriage, elem="blade")
        extended_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            carriage,
            handle,
            axis="z",
            positive_elem="thumb_pad",
            negative_elem="handle_shell",
            min_gap=0.0005,
            max_gap=0.006,
            name="extended thumb pad still clears the handle top",
        )

    ctx.check(
        "carriage slides toward the nose",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.025,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    if handle_box is not None and blade_rest is not None and blade_extended is not None:
        handle_nose_x = handle_box[1][0]
        ctx.check(
            "blade is mostly retracted at the lower stop",
            blade_rest[1][0] <= handle_nose_x + 0.002,
            details=f"blade_rest={blade_rest}, handle={handle_box}",
        )
        ctx.check(
            "blade projects from the nose at full travel",
            blade_extended[1][0] >= handle_nose_x + 0.009,
            details=f"blade_extended={blade_extended}, handle={handle_box}",
        )
    else:
        ctx.fail("blade travel can be measured", "Missing handle or blade AABB.")

    with ctx.pose({latch_joint: -0.45}):
        latch_up = ctx.part_element_world_aabb(safety_latch, elem="latch_tip")
    with ctx.pose({latch_joint: 0.45}):
        latch_down = ctx.part_element_world_aabb(safety_latch, elem="latch_tip")
    if latch_up is not None and latch_down is not None:
        up_z = (latch_up[0][2] + latch_up[1][2]) / 2.0
        down_z = (latch_down[0][2] + latch_down[1][2]) / 2.0
        ctx.check(
            "safety latch rotates on its side pivot",
            abs(up_z - down_z) > 0.020,
            details=f"up={latch_up}, down={latch_down}",
        )
    else:
        ctx.fail("safety latch rotation can be measured", "Missing latch tip AABB.")

    return ctx.report()


object_model = build_object_model()
