from __future__ import annotations

import math

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

SOCKET_DEPTH = 1.26
SOCKET_OUTER_RADIUS = 0.18
SOCKET_INNER_RADIUS = 0.152
SOCKET_BORE_DEPTH = 1.12
RIM_RADIUS = 0.22
RIM_THICKNESS = 0.08

RAM_RADIUS = 0.138
RAM_LENGTH = 1.32
RAM_Z_CENTER = 0.24
RAM_HIDDEN_INSERTION = (RAM_LENGTH * 0.5) - RAM_Z_CENTER
RAM_TRAVEL = 0.78

CROWN_BASE_Z = 0.90
CROWN_TOP_Z = 0.945
KEY_RECESS_FLOOR_Z = 0.931


def _socket_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .cylinder(SOCKET_DEPTH, SOCKET_OUTER_RADIUS)
        .translate((0.0, 0.0, -(SOCKET_DEPTH * 0.5)))
    )

    rim = (
        cq.Workplane("XY")
        .cylinder(RIM_THICKNESS, RIM_RADIUS)
        .translate((0.0, 0.0, -(RIM_THICKNESS * 0.5)))
    )

    upper_band = (
        cq.Workplane("XY")
        .cylinder(0.18, 0.198)
        .translate((0.0, 0.0, -0.13))
    )

    lower_anchor = (
        cq.Workplane("XY")
        .cylinder(0.12, 0.235)
        .translate((0.0, 0.0, -(SOCKET_DEPTH - 0.06)))
    )

    shell = shell.union(rim).union(upper_band).union(lower_anchor)

    fin_height = 0.34
    for x_sign in (-1.0, 1.0):
        shell = shell.union(
            cq.Workplane("XY")
            .box(0.05, 0.12, fin_height)
            .translate((x_sign * 0.205, 0.0, -(0.20 + (fin_height * 0.5))))
        )
    for y_sign in (-1.0, 1.0):
        shell = shell.union(
            cq.Workplane("XY")
            .box(0.12, 0.05, fin_height)
            .translate((0.0, y_sign * 0.205, -(0.20 + (fin_height * 0.5))))
        )

    shell = shell.union(
        cq.Workplane("XY")
        .box(0.11, 0.070, 0.03)
        .translate((0.0, 0.198, -0.018))
    )

    bore = (
        cq.Workplane("XY")
        .cylinder(SOCKET_BORE_DEPTH, SOCKET_INNER_RADIUS)
        .translate((0.0, 0.0, -(SOCKET_BORE_DEPTH * 0.5)))
    )
    shell = shell.cut(bore)
    return shell


def _crown_plate_shape() -> cq.Workplane:
    crown = (
        cq.Workplane("XY")
        .cylinder(0.034, 0.172)
        .translate((0.0, 0.0, 0.017))
    )
    crown = crown.union(
        cq.Workplane("XY")
        .cylinder(0.011, 0.158)
        .translate((0.0, 0.0, 0.0395))
    )
    crown = crown.union(
        cq.Workplane("XY")
        .cylinder(0.010, 0.142)
        .translate((0.0, 0.0, -0.003))
    )
    crown = crown.cut(
        cq.Workplane("XY")
        .cylinder(CROWN_TOP_Z - KEY_RECESS_FLOOR_Z, 0.036)
        .translate((0.0, 0.0, ((CROWN_TOP_Z - KEY_RECESS_FLOOR_Z) * 0.5) + 0.031))
    )
    return crown


def _key_insert_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .cylinder(0.018, 0.031)
        .translate((0.0, 0.0, 0.009))
    )
    body = body.union(
        cq.Workplane("XY")
        .cylinder(0.006, 0.0335)
        .translate((0.0, 0.0, 0.021))
    )
    body = body.cut(
        cq.Workplane("XY")
        .box(0.040, 0.006, 0.004)
        .translate((0.0, 0.0, 0.0215))
    )
    return body


def _ram_guide_flange_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.168)
        .circle(0.120)
        .extrude(0.012)
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        (lower[0] + upper[0]) * 0.5,
        (lower[1] + upper[1]) * 0.5,
        (lower[2] + upper[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="anti_vehicle_bollard")

    socket_steel = model.material("socket_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    ram_steel = model.material("ram_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    crown_dark = model.material("crown_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    warning_yellow = model.material("warning_yellow", rgba=(0.92, 0.77, 0.12, 1.0))
    brass = model.material("brass", rgba=(0.69, 0.58, 0.28, 1.0))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.12, 1.0))

    socket = model.part("socket")
    socket.visual(
        mesh_from_cadquery(_socket_shell_shape(), "socket_shell"),
        material=socket_steel,
        name="socket_shell",
    )

    ram = model.part("ram")
    ram.visual(
        Cylinder(radius=RAM_RADIUS, length=RAM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, RAM_Z_CENTER)),
        material=ram_steel,
        name="ram_shaft",
    )
    ram.visual(
        mesh_from_cadquery(_crown_plate_shape(), "crown_plate"),
        origin=Origin(xyz=(0.0, 0.0, CROWN_BASE_Z)),
        material=crown_dark,
        name="crown_plate",
    )
    ram.visual(
        mesh_from_cadquery(_ram_guide_flange_shape(), "guide_flange"),
        origin=Origin(),
        material=crown_dark,
        name="guide_flange",
    )
    ram.visual(
        Cylinder(radius=RAM_RADIUS + 0.002, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        material=warning_yellow,
        name="warning_band",
    )

    debris_cover = model.part("debris_cover")
    debris_cover.visual(
        Cylinder(radius=0.010, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="cover_barrel",
    )
    debris_cover.visual(
        Box((0.112, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, -0.035, 0.005)),
        material=crown_dark,
        name="cover_plate",
    )
    debris_cover.visual(
        Box((0.088, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.057, 0.006)),
        material=crown_dark,
        name="cover_lip",
    )

    key_insert = model.part("key_insert")
    key_insert.visual(
        mesh_from_cadquery(_key_insert_body_shape(), "key_insert_body"),
        material=brass,
        name="insert_body",
    )
    key_insert.visual(
        Box((0.020, 0.008, 0.006)),
        origin=Origin(xyz=(0.021, 0.0, 0.018)),
        material=brass,
        name="key_tab",
    )

    model.articulation(
        "socket_to_ram",
        ArticulationType.PRISMATIC,
        parent=socket,
        child=ram,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-RAM_TRAVEL,
            upper=0.0,
            effort=30000.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "socket_to_debris_cover",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=debris_cover,
        origin=Origin(xyz=(0.0, 0.232, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.40,
            effort=10.0,
            velocity=2.0,
        ),
    )
    model.articulation(
        "ram_to_key_insert",
        ArticulationType.REVOLUTE,
        parent=ram,
        child=key_insert,
        origin=Origin(xyz=(0.0, 0.0, KEY_RECESS_FLOOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-0.65,
            upper=0.65,
            effort=4.0,
            velocity=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    socket = object_model.get_part("socket")
    ram = object_model.get_part("ram")
    debris_cover = object_model.get_part("debris_cover")
    key_insert = object_model.get_part("key_insert")

    ram_slide = object_model.get_articulation("socket_to_ram")
    cover_hinge = object_model.get_articulation("socket_to_debris_cover")
    key_joint = object_model.get_articulation("ram_to_key_insert")

    ram_lower = ram_slide.motion_limits.lower if ram_slide.motion_limits is not None else None
    cover_upper = cover_hinge.motion_limits.upper if cover_hinge.motion_limits is not None else None
    key_upper = key_joint.motion_limits.upper if key_joint.motion_limits is not None else None

    ctx.expect_origin_distance(
        ram,
        socket,
        axes="xy",
        min_dist=0.0,
        max_dist=0.001,
        name="ram remains concentric with socket at rest",
    )
    ctx.expect_overlap(
        ram,
        socket,
        axes="z",
        elem_a="ram_shaft",
        elem_b="socket_shell",
        min_overlap=RAM_HIDDEN_INSERTION - 0.01,
        name="raised ram still has deep retained insertion",
    )

    if ram_lower is not None:
        with ctx.pose({ram_slide: ram_lower}):
            ctx.expect_origin_distance(
                ram,
                socket,
                axes="xy",
                min_dist=0.0,
                max_dist=0.001,
                name="ram remains concentric when lowered",
            )
            ctx.expect_overlap(
                ram,
                socket,
                axes="z",
                elem_a="ram_shaft",
                elem_b="socket_shell",
                min_overlap=1.10,
                name="lowered ram remains retained inside the ground socket",
            )

    raised_crown = _aabb_center(ctx.part_element_world_aabb(ram, elem="crown_plate"))
    lowered_crown = None
    if ram_lower is not None:
        with ctx.pose({ram_slide: ram_lower}):
            lowered_crown = _aabb_center(ctx.part_element_world_aabb(ram, elem="crown_plate"))
    ctx.check(
        "ram retracts substantially downward",
        raised_crown is not None
        and lowered_crown is not None
        and raised_crown[2] > lowered_crown[2] + 0.70,
        details=f"raised={raised_crown}, lowered={lowered_crown}",
    )

    closed_cover = _aabb_center(ctx.part_element_world_aabb(debris_cover, elem="cover_plate"))
    open_cover = None
    if cover_upper is not None:
        with ctx.pose({cover_hinge: cover_upper}):
            open_cover = _aabb_center(ctx.part_element_world_aabb(debris_cover, elem="cover_plate"))
    ctx.check(
        "debris cover swings open above the rim",
        closed_cover is not None
        and open_cover is not None
        and open_cover[2] > closed_cover[2] + 0.025,
        details=f"closed={closed_cover}, open={open_cover}",
    )

    ctx.expect_overlap(
        key_insert,
        ram,
        axes="xy",
        elem_a="insert_body",
        elem_b="crown_plate",
        min_overlap=0.05,
        name="key insert stays centered within the crown recess footprint",
    )

    rest_tab = _aabb_center(ctx.part_element_world_aabb(key_insert, elem="key_tab"))
    turned_tab = None
    if key_upper is not None:
        with ctx.pose({key_joint: key_upper}):
            turned_tab = _aabb_center(ctx.part_element_world_aabb(key_insert, elem="key_tab"))
    ctx.check(
        "key insert visibly rotates on the crown",
        rest_tab is not None
        and turned_tab is not None
        and abs(turned_tab[1] - rest_tab[1]) > 0.008,
        details=f"rest={rest_tab}, turned={turned_tab}",
    )

    return ctx.report()


object_model = build_object_model()
