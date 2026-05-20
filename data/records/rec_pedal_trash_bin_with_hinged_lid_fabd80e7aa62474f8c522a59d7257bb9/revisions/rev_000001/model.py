from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BIN_RADIUS = 0.145
BIN_HEIGHT = 0.450
BODY_WALL = 0.003
BODY_BOTTOM = 0.006

HINGE_X = -0.156
HINGE_Z = 0.466
HINGE_BARREL_RADIUS = 0.0056
HINGE_BARREL_LENGTH = 0.076

LID_RADIUS = 0.149
LID_THICKNESS = 0.003
LID_SKIRT_DEPTH = 0.014
LID_CENTER_X = abs(HINGE_X)

PEDAL_PIVOT_X = 0.168
PEDAL_PIVOT_Z = 0.037
PEDAL_MOUNT_Y = 0.044
PEDAL_BARREL_RADIUS = 0.0041

DAMPER_HINGE_X = -0.166
DAMPER_HINGE_Y = 0.166
DAMPER_HINGE_Z = 0.474


def _box(center: tuple[float, float, float], size: tuple[float, float, float]):
    return cq.Workplane("XY", origin=center).box(*size)


def _cylinder_y(center: tuple[float, float, float], radius: float, length: float):
    x, y, z = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((x, y + length / 2.0, z))
    )


def _make_body_shell():
    outer = cq.Workplane("XY").circle(BIN_RADIUS).extrude(BIN_HEIGHT)
    inner = (
        cq.Workplane("XY", origin=(0.0, 0.0, BODY_BOTTOM))
        .circle(BIN_RADIUS - BODY_WALL)
        .extrude(BIN_HEIGHT - BODY_BOTTOM)
    )
    return outer.cut(inner)


def _make_rear_brace():
    return _box(center=(-0.151, 0.0, 0.440), size=(0.020, 0.112, 0.024))


def _make_rear_ear(ear_y: float):
    ear_size = (0.014, 0.014, 0.050)
    ear = _box(center=(HINGE_X - 0.007, ear_y, 0.458), size=ear_size)
    hole = _cylinder_y(center=(HINGE_X, ear_y, HINGE_Z), radius=0.0057, length=0.020)
    return ear.cut(hole)


def _make_pedal_mount():
    bridge = _box(center=(0.148, 0.0, 0.024), size=(0.018, 0.102, 0.020))

    ear_size = (0.016, 0.012, 0.028)
    left_ear = _box(center=(PEDAL_PIVOT_X, -PEDAL_MOUNT_Y, PEDAL_PIVOT_Z), size=ear_size)
    right_ear = _box(center=(PEDAL_PIVOT_X, PEDAL_MOUNT_Y, PEDAL_PIVOT_Z), size=ear_size)
    left_strut = _box(center=(0.158, -PEDAL_MOUNT_Y, 0.026), size=(0.016, 0.012, 0.018))
    right_strut = _box(center=(0.158, PEDAL_MOUNT_Y, 0.026), size=(0.016, 0.012, 0.018))

    mount = bridge.union(left_ear).union(right_ear).union(left_strut).union(right_strut)
    hole = _cylinder_y(center=(PEDAL_PIVOT_X, -PEDAL_MOUNT_Y, PEDAL_PIVOT_Z), radius=PEDAL_BARREL_RADIUS, length=0.018)
    mount = mount.cut(hole)
    hole = _cylinder_y(center=(PEDAL_PIVOT_X, PEDAL_MOUNT_Y, PEDAL_PIVOT_Z), radius=PEDAL_BARREL_RADIUS, length=0.018)
    return mount.cut(hole)


def _make_lid_shell():
    outer = (
        cq.Workplane("XY", origin=(LID_CENTER_X, 0.0, -LID_SKIRT_DEPTH))
        .circle(LID_RADIUS)
        .workplane(offset=0.012)
        .circle(LID_RADIUS)
        .workplane(offset=0.014)
        .circle(0.138)
        .workplane(offset=0.012)
        .circle(0.104)
        .workplane(offset=0.010)
        .circle(0.030)
        .loft(combine=True)
    )

    inner = (
        cq.Workplane("XY", origin=(LID_CENTER_X, 0.0, -LID_SKIRT_DEPTH - 0.003))
        .circle(LID_RADIUS - LID_THICKNESS)
        .workplane(offset=0.010)
        .circle(LID_RADIUS - LID_THICKNESS)
        .workplane(offset=0.012)
        .circle(0.135)
        .workplane(offset=0.011)
        .circle(0.098)
        .workplane(offset=0.008)
        .circle(0.028)
        .loft(combine=True)
    )

    flange = _box(center=(0.010, 0.0, -0.004), size=(0.018, 0.086, 0.016))
    return outer.cut(inner).union(flange)


def _make_lid_barrel():
    return _cylinder_y(center=(0.0, 0.0, 0.0), radius=HINGE_BARREL_RADIUS, length=HINGE_BARREL_LENGTH)


def _make_pedal():
    blade = _box(center=(0.020, 0.0, 0.040), size=(0.020, 0.104, 0.078))
    tread = _box(center=(0.035, 0.0, 0.010), size=(0.050, 0.114, 0.016))
    spine = _box(center=(0.014, 0.0, 0.028), size=(0.012, 0.082, 0.050))
    neck = _box(center=(0.006, 0.0, 0.008), size=(0.004, 0.070, 0.016))
    barrel = _cylinder_y(center=(0.0, 0.0, 0.0), radius=PEDAL_BARREL_RADIUS, length=0.074)
    return blade.union(tread).union(spine).union(neck).union(barrel)


def _make_damper_mount():
    arm = _box(center=(-0.153, 0.111, 0.444), size=(0.018, 0.110, 0.012))
    block = _box(center=(-0.157, DAMPER_HINGE_Y, 0.459), size=(0.020, 0.024, 0.024))
    hinge_tab = _box(center=(-0.170, DAMPER_HINGE_Y, 0.462), size=(0.008, 0.028, 0.012))
    return arm.union(block).union(hinge_tab)


def _make_damper_cover():
    base = _box(center=(0.026, 0.0, 0.006), size=(0.052, 0.032, 0.018))
    crown = _box(center=(0.038, 0.0, 0.017), size=(0.036, 0.026, 0.012))
    rib = _box(center=(0.010, 0.0, 0.007), size=(0.020, 0.028, 0.010))
    barrel = _cylinder_y(center=(0.0, 0.0, 0.0), radius=0.0040, length=0.028)
    return base.union(crown).union(rib).union(barrel)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_pedal_bin")

    body_metal = model.material("body_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    trim = model.material("trim", rgba=(0.15, 0.15, 0.17, 1.0))
    lid_finish = model.material("lid_finish", rgba=(0.20, 0.21, 0.23, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shell(), "body_shell"),
        material=body_metal,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_make_rear_brace(), "rear_brace"),
        material=trim,
        name="rear_brace",
    )
    body.visual(
        mesh_from_cadquery(_make_rear_ear(-0.050), "rear_ear_0"),
        material=trim,
        name="rear_ear_0",
    )
    body.visual(
        mesh_from_cadquery(_make_rear_ear(0.050), "rear_ear_1"),
        material=trim,
        name="rear_ear_1",
    )
    body.visual(
        mesh_from_cadquery(_make_pedal_mount(), "pedal_mount"),
        material=trim,
        name="pedal_mount",
    )
    body.visual(
        mesh_from_cadquery(_make_damper_mount(), "damper_mount"),
        material=trim,
        name="damper_mount",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_make_lid_shell(), "lid_shell"),
        material=lid_finish,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_make_lid_barrel(), "lid_barrel"),
        material=trim,
        name="lid_barrel",
    )

    pedal = model.part("pedal")
    pedal.visual(
        mesh_from_cadquery(_make_pedal(), "pedal"),
        material=trim,
        name="pedal",
    )

    damper_cover = model.part("damper_cover")
    damper_cover.visual(
        mesh_from_cadquery(_make_damper_cover(), "damper_cover"),
        material=lid_finish,
        name="damper_cover",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=0.0, upper=1.28),
    )
    model.articulation(
        "pedal_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(PEDAL_PIVOT_X, 0.0, PEDAL_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.5, lower=0.0, upper=0.55),
    )
    model.articulation(
        "damper_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=damper_cover,
        origin=Origin(xyz=(DAMPER_HINGE_X, DAMPER_HINGE_Y, DAMPER_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    damper_cover = object_model.get_part("damper_cover")
    lid_hinge = object_model.get_articulation("lid_hinge")
    pedal_hinge = object_model.get_articulation("pedal_hinge")
    damper_cover_hinge = object_model.get_articulation("damper_cover_hinge")

    limits = lid_hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({lid_hinge: limits.lower}):
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                positive_elem="lid_shell",
                negative_elem="body_shell",
                max_gap=0.006,
                max_penetration=0.0,
                name="closed lid sits just above the rim",
            )
            ctx.expect_overlap(
                lid,
                body,
                axes="xy",
                elem_a="lid_shell",
                elem_b="body_shell",
                min_overlap=0.250,
                name="closed lid covers the body opening",
            )
            closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

        with ctx.pose({lid_hinge: limits.upper}):
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                positive_elem="lid_shell",
                negative_elem="body_shell",
                min_gap=0.010,
                name="open lid lifts clear of the rim",
            )
            open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

        ctx.check(
            "lid opens upward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.200
            and open_aabb[0][0] < closed_aabb[0][0] - 0.015,
            details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
        )

    pedal_limits = pedal_hinge.motion_limits
    if pedal_limits is not None and pedal_limits.lower is not None and pedal_limits.upper is not None:
        ctx.expect_origin_gap(
            pedal,
            body,
            axis="x",
            min_gap=0.150,
            name="pedal pivot sits in front of the bin body",
        )
        with ctx.pose({pedal_hinge: pedal_limits.lower}):
            pedal_rest = ctx.part_element_world_aabb(pedal, elem="pedal")
        with ctx.pose({pedal_hinge: pedal_limits.upper}):
            pedal_pressed = ctx.part_element_world_aabb(pedal, elem="pedal")

        ctx.check(
            "pedal tips downward when pressed",
            pedal_rest is not None
            and pedal_pressed is not None
            and pedal_pressed[1][2] < pedal_rest[1][2] - 0.008
            and pedal_pressed[0][2] < pedal_rest[0][2] - 0.020,
            details=f"rest={pedal_rest}, pressed={pedal_pressed}",
        )

    damper_limits = damper_cover_hinge.motion_limits
    if damper_limits is not None and damper_limits.lower is not None and damper_limits.upper is not None:
        ctx.expect_origin_gap(
            damper_cover,
            body,
            axis="y",
            min_gap=0.150,
            name="damper cover sits on the hinge side of the bin",
        )
        with ctx.pose({damper_cover_hinge: damper_limits.lower}):
            damper_closed = ctx.part_element_world_aabb(damper_cover, elem="damper_cover")
        with ctx.pose({damper_cover_hinge: damper_limits.upper}):
            damper_open = ctx.part_element_world_aabb(damper_cover, elem="damper_cover")

        ctx.check(
            "damper cover lifts upward",
            damper_closed is not None
            and damper_open is not None
            and damper_open[1][2] > damper_closed[1][2] + 0.030,
            details=f"closed={damper_closed}, open={damper_open}",
        )

    return ctx.report()


object_model = build_object_model()
