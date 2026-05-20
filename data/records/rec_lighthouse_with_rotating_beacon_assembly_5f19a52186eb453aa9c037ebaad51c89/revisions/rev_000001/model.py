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


def _cylinder(z0: float, height: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z0))


def _annular_cylinder(
    z0: float, height: float, outer_radius: float, inner_radius: float
) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    cutter = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height + 0.02)
        .translate((0.0, 0.0, -0.01))
    )
    return outer.cut(cutter).translate((0.0, 0.0, z0))


def _frustum(z0: float, height: float, bottom_radius: float, top_radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(bottom_radius)
        .workplane(offset=height)
        .circle(top_radius)
        .loft(combine=True)
        .translate((0.0, 0.0, z0))
    )


def _annular_sector(
    z0: float,
    height: float,
    outer_radius: float,
    inner_radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 24,
) -> cq.Workplane:
    outer = []
    inner = []
    for i in range(segments + 1):
        a = start_angle + (end_angle - start_angle) * i / segments
        outer.append((outer_radius * math.cos(a), outer_radius * math.sin(a)))
        inner.append((inner_radius * math.cos(a), inner_radius * math.sin(a)))
    points = outer + list(reversed(inner))
    return cq.Workplane("XY").polyline(points).close().extrude(height).translate((0, 0, z0))


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _make_stone_body() -> cq.Workplane:
    """Broad, squat masonry pier with stacked round plinths."""
    return _union_all(
        [
            _cylinder(0.00, 0.28, 1.55),
            _cylinder(0.24, 0.20, 1.32),
            _cylinder(0.40, 0.20, 1.05),
            _frustum(0.55, 1.85, 0.74, 0.56),
            _cylinder(2.34, 0.22, 0.62),
            _cylinder(2.50, 0.18, 0.52),
        ]
    )


def _make_gallery() -> cq.Workplane:
    """Wraparound gallery deck and guard rail, all tied together by posts."""
    shapes: list[cq.Workplane] = [
        _annular_cylinder(2.50, 0.12, 0.98, 0.42),
        _annular_cylinder(2.62, 0.035, 0.92, 0.84),
        _annular_cylinder(3.03, 0.045, 0.92, 0.84),
    ]
    for i in range(16):
        angle = 2.0 * math.pi * i / 16
        x = 0.88 * math.cos(angle)
        y = 0.88 * math.sin(angle)
        shapes.append(cq.Workplane("XY").circle(0.022).extrude(0.44).translate((x, y, 2.61)))
    return _union_all(shapes)


def _make_lantern_frame() -> cq.Workplane:
    """Cylindrical lantern metalwork: curb rings, mullions, overhanging roof."""
    shapes: list[cq.Workplane] = [
        _annular_cylinder(2.62, 0.12, 0.50, 0.36),
        _annular_cylinder(3.42, 0.12, 0.50, 0.36),
        _annular_cylinder(3.52, 0.055, 0.66, 0.37),
        _frustum(3.55, 0.48, 0.58, 0.12),
        _cylinder(4.00, 0.12, 0.13),
        _cylinder(4.10, 0.18, 0.045),
    ]
    for i in range(10):
        angle = 2.0 * math.pi * i / 10
        # Leave a clear front bay for the maintenance door instead of placing
        # mullions through the hinged panel.
        if math.radians(-120.0) <= math.atan2(math.sin(angle), math.cos(angle)) <= math.radians(-60.0):
            continue
        x = 0.425 * math.cos(angle)
        y = 0.425 * math.sin(angle)
        shapes.append(cq.Workplane("XY").circle(0.018).extrude(0.82).translate((x, y, 2.66)))
    return _union_all(shapes)


def _make_beacon() -> cq.Workplane:
    """Rotating beacon drum with a clear spindle bore and retaining collars."""
    shapes: list[cq.Workplane] = [
        _annular_cylinder(-0.15, 0.30, 0.17, 0.055),
        # The top and bottom bearing collars intentionally grip the spindle
        # proxy so the rotating drum is not a floating light cage.
        _annular_cylinder(-0.19, 0.055, 0.105, 0.020),
        _annular_cylinder(0.135, 0.055, 0.105, 0.020),
    ]
    for angle in (0.0, math.pi):
        x = 0.165 * math.cos(angle)
        y = 0.165 * math.sin(angle)
        shapes.append(
            cq.Workplane("XY")
            .box(0.030, 0.155, 0.18)
            .translate((x, y, 0.0))
            .rotate((0, 0, 0), (0, 0, 1), math.degrees(angle))
        )
    return _union_all(shapes)


def _make_curved_door() -> cq.Workplane:
    """Curved lantern maintenance door expressed in its side-hinge frame."""
    wall_radius = 0.430
    hinge_angle = math.radians(-110.0)
    end_angle = math.radians(-70.0)
    hinge_radius = 0.465
    hinge_x = hinge_radius * math.cos(hinge_angle)
    hinge_y = hinge_radius * math.sin(hinge_angle)

    panel = _annular_sector(
        0.0,
        0.58,
        wall_radius + 0.030,
        wall_radius + 0.006,
        hinge_angle,
        end_angle,
        segments=22,
    ).translate((-hinge_x, -hinge_y, 0.0))

    # Hollow hinge barrel around the fixed pin.  The generous bore keeps the pin
    # clear while visually clipping the door to its side hinge.
    barrel = _annular_cylinder(0.0, 0.58, 0.030, 0.014).translate((0.0, 0.0, 0.0))
    straps = _union_all(
        [
            cq.Workplane("XY").box(0.08, 0.018, 0.055).translate((0.065, 0.0, 0.13)),
            cq.Workplane("XY").box(0.08, 0.018, 0.055).translate((0.065, 0.0, 0.45)),
        ]
    )
    return _union_all([panel, barrel, straps])


def _make_door_hinge_pin_mount() -> cq.Workplane:
    """Fixed hinge pin with top and bottom leaves tied into the lantern frame."""
    hinge_angle = math.radians(-110.0)
    hinge_radius = 0.465
    hinge_x = hinge_radius * math.cos(hinge_angle)
    hinge_y = hinge_radius * math.sin(hinge_angle)
    pin = cq.Workplane("XY").circle(0.0075).extrude(0.64).translate((hinge_x, hinge_y, 2.75))

    def leaf(z_center: float, height: float) -> cq.Workplane:
        return (
            cq.Workplane("XY")
            .box(0.12, 0.045, height)
            .translate((0.455, 0.0, z_center))
            .rotate((0, 0, 0), (0, 0, 1), math.degrees(hinge_angle))
        )

    return _union_all([pin, leaf(2.755, 0.045), leaf(3.395, 0.060)])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="squat_pier_lighthouse")

    stone = Material("warm_gray_stone", rgba=(0.58, 0.56, 0.50, 1.0))
    dark_stone = Material("dark_stone_courses", rgba=(0.30, 0.31, 0.31, 1.0))
    weathered_metal = Material("weathered_green_metal", rgba=(0.13, 0.34, 0.31, 1.0))
    black_metal = Material("blackened_metal", rgba=(0.02, 0.025, 0.025, 1.0))
    glass = Material("slightly_green_glass", rgba=(0.58, 0.86, 0.92, 0.35))
    brass = Material("aged_brass", rgba=(0.85, 0.62, 0.24, 1.0))
    lens = Material("warm_beacon_lens", rgba=(1.0, 0.80, 0.23, 0.90))
    door_paint = Material("faded_red_door", rgba=(0.55, 0.08, 0.055, 1.0))

    tower = model.part("tower")
    tower.visual(
        mesh_from_cadquery(_make_stone_body(), "pier_stone_body", tolerance=0.004),
        material=stone,
        name="stone_body",
    )
    # Dark masonry courses make the broad squat pier read as stacked stonework.
    for idx, z in enumerate((0.28, 0.44, 1.02, 1.55, 2.08)):
        radius = 1.34 if idx < 2 else (0.72, 0.67, 0.62)[idx - 2]
        tower.visual(
            Cylinder(radius=radius, length=0.025),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_stone,
            name=f"stone_course_{idx}",
        )
    tower.visual(
        mesh_from_cadquery(_make_gallery(), "wraparound_gallery", tolerance=0.003),
        material=weathered_metal,
        name="wraparound_gallery",
    )
    tower.visual(
        mesh_from_cadquery(_make_lantern_frame(), "lantern_frame_roof", tolerance=0.003),
        material=weathered_metal,
        name="lantern_frame",
    )
    tower.visual(
        mesh_from_cadquery(
            _annular_cylinder(2.70, 0.72, 0.415, 0.385),
            "cylindrical_lantern_glass",
            tolerance=0.003,
        ),
        material=glass,
        name="lantern_glass",
    )
    tower.visual(
        Cylinder(radius=0.025, length=1.12),
        origin=Origin(xyz=(0.0, 0.0, 3.14)),
        material=black_metal,
        name="central_spindle",
    )
    tower.visual(
        mesh_from_cadquery(_make_door_hinge_pin_mount(), "fixed_door_hinge_pin", tolerance=0.0015),
        material=black_metal,
        name="door_hinge_pin",
    )

    beacon = model.part("beacon")
    beacon.visual(
        mesh_from_cadquery(_make_beacon(), "rotating_beacon_drum", tolerance=0.002),
        material=brass,
        name="beacon_shell",
    )
    beacon.visual(
        Box((0.042, 0.20, 0.16)),
        origin=Origin(xyz=(0.185, 0.0, 0.0)),
        material=lens,
        name="front_lens",
    )
    beacon.visual(
        Box((0.042, 0.20, 0.16)),
        origin=Origin(xyz=(-0.185, 0.0, 0.0)),
        material=lens,
        name="rear_lens",
    )

    maintenance_door = model.part("maintenance_door")
    maintenance_door.visual(
        mesh_from_cadquery(_make_curved_door(), "curved_maintenance_door", tolerance=0.002),
        material=door_paint,
        name="door_panel",
    )

    model.articulation(
        "beacon_spin",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 3.13)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=maintenance_door,
        origin=Origin(
            xyz=(
                0.465 * math.cos(math.radians(-110.0)),
                0.465 * math.sin(math.radians(-110.0)),
                2.78,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    beacon = object_model.get_part("beacon")
    door = object_model.get_part("maintenance_door")
    beacon_spin = object_model.get_articulation("beacon_spin")
    door_hinge = object_model.get_articulation("door_hinge")

    ctx.allow_overlap(
        tower,
        beacon,
        elem_a="central_spindle",
        elem_b="beacon_shell",
        reason=(
            "The beacon's bearing collars intentionally capture the central spindle "
            "proxy so the drum stays seated while rotating."
        ),
    )
    ctx.allow_overlap(
        door,
        tower,
        elem_a="door_panel",
        elem_b="door_hinge_pin",
        reason=(
            "The fixed hinge pin is intentionally captured inside the door's "
            "hinge barrel so the maintenance door rotates on a real side axis."
        ),
    )

    ctx.expect_within(
        beacon,
        tower,
        axes="xy",
        inner_elem="beacon_shell",
        outer_elem="lantern_glass",
        margin=0.0,
        name="beacon drum sits inside cylindrical lantern",
    )
    ctx.expect_within(
        tower,
        beacon,
        axes="xy",
        inner_elem="central_spindle",
        outer_elem="beacon_shell",
        margin=0.0,
        name="central spindle passes through beacon bore footprint",
    )
    ctx.expect_overlap(
        beacon,
        tower,
        axes="z",
        elem_a="beacon_shell",
        elem_b="central_spindle",
        min_overlap=0.30,
        name="beacon collars stay vertically clipped to spindle",
    )

    with ctx.pose({beacon_spin: math.pi / 2.0}):
        ctx.expect_within(
            beacon,
            tower,
            axes="xy",
            inner_elem="beacon_shell",
            outer_elem="lantern_glass",
            margin=0.0,
            name="rotated beacon remains inside lantern",
        )
        ctx.expect_overlap(
            beacon,
            tower,
            axes="z",
            elem_a="beacon_shell",
            elem_b="central_spindle",
            min_overlap=0.30,
            name="rotated beacon remains seated on spindle",
        )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.1}):
        open_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "maintenance door swings outward from side hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < closed_aabb[0][1] - 0.05,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )
    ctx.expect_overlap(
        door,
        tower,
        axes="z",
        elem_a="door_panel",
        elem_b="door_hinge_pin",
        min_overlap=0.50,
        name="door hinge barrel remains on fixed pin height",
    )

    return ctx.report()


object_model = build_object_model()
