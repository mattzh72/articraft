from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


ROSE_HALF = 0.17
GRID_HALF = 0.43
GRID_DROP = -0.75
ROSE_JOINT_Z = -0.035
GRID_JOINT_Z = 0.075
TOP_ATTACH = 0.12
WIRE_RADIUS = 0.0035
LOOP_RADIUS = 0.026
PIN_RADIUS = 0.0065
LOOP_SAG = LOOP_RADIUS - PIN_RADIUS - WIRE_RADIUS + 0.003


def _corner_specs() -> list[tuple[int, int]]:
    return [(-1, -1), (1, -1), (1, 1), (-1, 1)]


def _unit(x: float, y: float) -> tuple[float, float]:
    length = math.hypot(x, y)
    return (x / length, y / length)


def _loop_points(
    *,
    radial: tuple[float, float],
    radius: float = LOOP_RADIUS,
    center_z: float = 0.0,
    samples: int = 28,
) -> list[tuple[float, float, float]]:
    """A circular eye loop in the radial/Z plane; its pin axis is tangential."""
    ux, uy = radial
    pts = []
    for i in range(samples):
        a = 2.0 * math.pi * i / samples
        pts.append((ux * radius * math.cos(a), uy * radius * math.cos(a), center_z + radius * math.sin(a)))
    return pts


def _hanger_wire_geometry(dx: float, dy: float, dz: float):
    radial = _unit(dx, dy)
    # Build the upper eye and the sloping hanger wire as one continuous tube.
    loop = _loop_points(radial=radial, center_z=-LOOP_SAG, samples=30)
    bottom_of_loop = (0.0, 0.0, -LOOP_SAG - LOOP_RADIUS)
    lead = (radial[0] * 0.010, radial[1] * 0.010, -LOOP_SAG - LOOP_RADIUS - 0.020)
    lower_lead = (dx, dy, dz - LOOP_SAG + LOOP_RADIUS)
    # Stop at the outer edge of the lower eye so the hanger does not pass
    # through the grid hinge pin that the lower loop rotates around.
    path = [bottom_of_loop] + loop[23:] + loop[:24] + [lead, lower_lead]
    return wire_from_points(
        path,
        radius=WIRE_RADIUS,
        radial_segments=14,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.006,
        corner_segments=6,
    )


def _loop_ring_geometry(sx: int, sy: int):
    radial = _unit(sx, sy)
    return wire_from_points(
        _loop_points(radial=radial, center_z=-LOOP_SAG, samples=36),
        radius=WIRE_RADIUS,
        radial_segments=14,
        closed_path=True,
        cap_ends=False,
        corner_mode="miter",
    )


def _axis_for_corner(sx: int, sy: int) -> tuple[float, float, float]:
    # Tangent to the square corner: perpendicular to the radial hanger plane.
    tx, ty = _unit(-sy, sx)
    return (tx, ty, 0.0)


def _pin_rpy_for_axis(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    yaw = math.atan2(axis[1], axis[0])
    return (0.0, math.pi / 2.0, yaw)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_grid_pendant")

    matte_black = Material("matte_black", color=(0.005, 0.005, 0.004, 1.0))
    dark_wire = Material("blackened_steel_wire", color=(0.01, 0.01, 0.01, 1.0))
    warm_brass = Material("brushed_brass", color=(0.86, 0.62, 0.28, 1.0))
    frosted_glass = Material("frosted_warm_glass", color=(1.0, 0.88, 0.62, 0.58))

    rose = model.part("ceiling_rose")
    rose.visual(
        Box((0.34, 0.34, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=matte_black,
        name="square_canopy",
    )
    rose.visual(
        Box((0.27, 0.27, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=matte_black,
        name="lower_step",
    )
    rose.visual(
        Cylinder(radius=0.006, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, -0.36)),
        material=dark_wire,
        name="power_cable",
    )

    for i, (sx, sy) in enumerate(_corner_specs()):
        x = sx * TOP_ATTACH
        y = sy * TOP_ATTACH
        axis = _axis_for_corner(sx, sy)
        rpy = _pin_rpy_for_axis(axis)
        tx, ty, _ = axis
        for side in (-1.0, 1.0):
            rose.visual(
                Cylinder(radius=0.007, length=0.050),
                origin=Origin(xyz=(x + tx * side * 0.030, y + ty * side * 0.030, -0.024)),
                material=matte_black,
                name=f"upper_lug_{i}_{0 if side < 0 else 1}",
            )
        rose.visual(
            Cylinder(radius=0.0065, length=0.070),
            origin=Origin(xyz=(x, y, ROSE_JOINT_Z), rpy=rpy),
            material=warm_brass,
            name=f"upper_pin_{i}",
        )

    grid = model.part("grid")
    bar_z = 0.0
    # Perimeter frame.
    grid.visual(Box((0.90, 0.044, 0.026)), origin=Origin(xyz=(0.0, GRID_HALF, bar_z)), material=matte_black, name="front_rail")
    grid.visual(Box((0.90, 0.044, 0.026)), origin=Origin(xyz=(0.0, -GRID_HALF, bar_z)), material=matte_black, name="rear_rail")
    grid.visual(Box((0.044, 0.90, 0.026)), origin=Origin(xyz=(GRID_HALF, 0.0, bar_z)), material=matte_black, name="side_rail_0")
    grid.visual(Box((0.044, 0.90, 0.026)), origin=Origin(xyz=(-GRID_HALF, 0.0, bar_z)), material=matte_black, name="side_rail_1")

    # Flat internal lattice, welded into the perimeter.
    for idx, x in enumerate((-0.22, 0.0, 0.22)):
        grid.visual(
            Box((0.018, 0.82, 0.018)),
            origin=Origin(xyz=(x, 0.0, -0.002)),
            material=matte_black,
            name=f"long_bar_{idx}",
        )
    for idx, y in enumerate((-0.22, 0.0, 0.22)):
        grid.visual(
            Box((0.82, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, y, -0.002)),
            material=matte_black,
            name=f"cross_bar_{idx}",
        )

    # Corner hinge pins mounted between small yoke posts welded to the square frame.
    for i, (sx, sy) in enumerate(_corner_specs()):
        axis = _axis_for_corner(sx, sy)
        x = sx * GRID_HALF
        y = sy * GRID_HALF
        tx, ty, _ = axis
        for side in (-1.0, 1.0):
            grid.visual(
                Cylinder(radius=0.007, length=0.070),
                origin=Origin(xyz=(x + tx * side * 0.035, y + ty * side * 0.035, 0.041)),
                material=matte_black,
                name=f"corner_lug_{i}_{0 if side < 0 else 1}",
            )
        grid.visual(
            Cylinder(radius=0.0065, length=0.074),
            origin=Origin(xyz=(x, y, GRID_JOINT_Z), rpy=_pin_rpy_for_axis(axis)),
            material=warm_brass,
            name=f"lower_pin_{i}",
        )

    socket_positions = [(-0.22, -0.22), (0.22, -0.22), (-0.22, 0.22), (0.22, 0.22), (0.0, 0.0)]
    for i, (x, y) in enumerate(socket_positions):
        grid.visual(
            Box((0.078, 0.078, 0.014)),
            origin=Origin(xyz=(x, y, -0.014)),
            material=matte_black,
            name=f"socket_plate_{i}",
        )
        grid.visual(
            Cylinder(radius=0.041, length=0.014),
            origin=Origin(xyz=(x, y, -0.026)),
            material=warm_brass,
            name=f"socket_rim_{i}",
        )
        grid.visual(
            Cylinder(radius=0.032, length=0.068),
            origin=Origin(xyz=(x, y, -0.064)),
            material=warm_brass,
            name=f"socket_body_{i}",
        )
        grid.visual(
            Sphere(radius=0.045),
            origin=Origin(xyz=(x, y, -0.125)),
            material=frosted_glass,
            name=f"bulb_{i}",
        )

    model.articulation(
        "rose_to_grid",
        ArticulationType.FIXED,
        parent=rose,
        child=grid,
        origin=Origin(xyz=(0.0, 0.0, GRID_DROP)),
    )

    for i, (sx, sy) in enumerate(_corner_specs()):
        top = (sx * TOP_ATTACH, sy * TOP_ATTACH, ROSE_JOINT_Z)
        bottom_world = (sx * GRID_HALF, sy * GRID_HALF, GRID_DROP + GRID_JOINT_Z)
        dx = bottom_world[0] - top[0]
        dy = bottom_world[1] - top[1]
        dz = bottom_world[2] - top[2]
        axis = _axis_for_corner(sx, sy)

        hanger = model.part(f"hanger_{i}")
        hanger.visual(
            mesh_from_geometry(_hanger_wire_geometry(dx, dy, dz), f"hanger_wire_{i}"),
            material=dark_wire,
            name="wire",
        )

        loop = model.part(f"lower_loop_{i}")
        loop.visual(
            mesh_from_geometry(_loop_ring_geometry(sx, sy), f"lower_loop_ring_{i}"),
            material=dark_wire,
            name="eye",
        )

        model.articulation(
            f"upper_loop_{i}",
            ArticulationType.REVOLUTE,
            parent=rose,
            child=hanger,
            origin=Origin(xyz=top),
            axis=axis,
            motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=-0.28, upper=0.28),
        )
        model.articulation(
            f"lower_eye_{i}",
            ArticulationType.REVOLUTE,
            parent=hanger,
            child=loop,
            origin=Origin(xyz=(dx, dy, dz)),
            axis=axis,
            motion_limits=MotionLimits(effort=1.2, velocity=1.0, lower=-0.35, upper=0.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rose = object_model.get_part("ceiling_rose")
    grid = object_model.get_part("grid")

    ctx.expect_origin_gap(rose, grid, axis="z", min_gap=0.70, name="grid hangs well below the square ceiling rose")
    ctx.expect_overlap(
        rose,
        grid,
        axes="xy",
        min_overlap=0.30,
        name="grid remains centered under the rose",
    )

    for i in range(4):
        hanger = object_model.get_part(f"hanger_{i}")
        loop = object_model.get_part(f"lower_loop_{i}")
        upper = object_model.get_articulation(f"upper_loop_{i}")
        lower = object_model.get_articulation(f"lower_eye_{i}")

        ctx.allow_overlap(
            rose,
            hanger,
            elem_a=f"upper_pin_{i}",
            elem_b="wire",
            reason="The upper eye loop is intentionally captured on the ceiling-rose hinge pin with slight seated contact.",
        )
        ctx.allow_overlap(
            grid,
            loop,
            elem_a=f"lower_pin_{i}",
            elem_b="eye",
            reason="The lower eye loop is intentionally captured on the grid-corner hinge pin with slight seated contact.",
        )
        ctx.allow_overlap(
            hanger,
            loop,
            elem_a="wire",
            elem_b="eye",
            reason="The sloping hanger wire is crimped into the lower eye loop so the modeled split loop remains physically connected.",
        )
        ctx.expect_overlap(
            hanger,
            rose,
            axes="xyz",
            min_overlap=0.006,
            elem_a="wire",
            elem_b=f"upper_pin_{i}",
            name=f"upper loop {i} surrounds rose pin",
        )
        ctx.expect_overlap(
            loop,
            grid,
            axes="xyz",
            min_overlap=0.006,
            elem_a="eye",
            elem_b=f"lower_pin_{i}",
            name=f"lower loop {i} surrounds grid pin",
        )
        ctx.expect_overlap(
            hanger,
            loop,
            axes="xyz",
            min_overlap=0.004,
            elem_a="wire",
            elem_b="eye",
            name=f"hanger {i} wire meets lower eye",
        )

        rest_pos = ctx.part_world_position(loop)
        with ctx.pose({upper: 0.18, lower: -0.18}):
            moved_pos = ctx.part_world_position(loop)
        ctx.check(
            f"hanger {i} has two working revolute loop joints",
            rest_pos is not None
            and moved_pos is not None
            and abs(moved_pos[2] - rest_pos[2]) > 0.004,
            details=f"rest={rest_pos}, moved={moved_pos}",
        )

    return ctx.report()


object_model = build_object_model()
