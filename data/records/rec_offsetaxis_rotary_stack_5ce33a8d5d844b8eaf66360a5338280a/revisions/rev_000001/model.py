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


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """A vertical bearing race with a real through-bore."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _faceplate(radius: float, thickness: float, bolt_circle: float, bolt_radius: float) -> cq.Workplane:
    """Small rotary faceplate with four through bolt holes."""
    plate = cq.Workplane("XY").circle(radius).extrude(thickness)
    for index in range(4):
        angle = index * math.pi / 2.0
        x = bolt_circle * math.cos(angle)
        y = bolt_circle * math.sin(angle)
        cutter = (
            cq.Workplane("XY")
            .center(x, y)
            .circle(bolt_radius)
            .extrude(thickness * 3.0)
            .translate((0.0, 0.0, -thickness))
        )
        plate = plate.cut(cutter)
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_axis_rotary_stack")

    dark_cast = model.material("dark_cast_iron", rgba=(0.06, 0.065, 0.07, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.63, 0.66, 0.68, 1.0))
    blue_anodized = model.material("blue_anodized_arm", rgba=(0.05, 0.20, 0.42, 1.0))
    bearing_bronze = model.material("bearing_bronze", rgba=(0.68, 0.48, 0.22, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    red_index = model.material("red_index_mark", rgba=(0.85, 0.06, 0.035, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.34, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_cast,
        name="ground_plinth",
    )
    base.visual(
        mesh_from_cadquery(_annular_cylinder(0.180, 0.085, 0.060), "base_bearing_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=bearing_bronze,
        name="base_bearing_ring",
    )
    for index, (x, y) in enumerate(((0.25, 0.25), (-0.25, 0.25), (-0.25, -0.25), (0.25, -0.25))):
        base.visual(
            Box((0.095, 0.055, 0.018)),
            origin=Origin(xyz=(x, y, 0.009), rpy=(0.0, 0.0, math.atan2(y, x))),
            material=black_rubber,
            name=f"foot_{index}",
        )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.060, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=satin_steel,
        name="lower_spigot",
    )
    turntable.visual(
        Cylinder(radius=0.250, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=satin_steel,
        name="turntable_disk",
    )
    turntable.visual(
        Cylinder(radius=0.112, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=satin_steel,
        name="center_hub",
    )
    turntable.visual(
        Box((0.485, 0.110, 0.070)),
        origin=Origin(xyz=(0.318, 0.0, 0.105)),
        material=blue_anodized,
        name="side_arm",
    )
    turntable.visual(
        Box((0.400, 0.030, 0.090)),
        origin=Origin(xyz=(0.330, 0.047, 0.105), rpy=(0.0, 0.0, 0.0)),
        material=blue_anodized,
        name="front_rib",
    )
    turntable.visual(
        Box((0.400, 0.030, 0.090)),
        origin=Origin(xyz=(0.330, -0.047, 0.105), rpy=(0.0, 0.0, 0.0)),
        material=blue_anodized,
        name="rear_rib",
    )
    turntable.visual(
        mesh_from_cadquery(_annular_cylinder(0.105, 0.055, 0.190), "upper_bearing_column"),
        origin=Origin(xyz=(0.550, 0.0, 0.065)),
        material=bearing_bronze,
        name="upper_bearing_column",
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Cylinder(radius=0.036, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, -0.0325)),
        material=satin_steel,
        name="upper_shaft",
    )
    faceplate.visual(
        Cylinder(radius=0.075, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=bearing_bronze,
        name="thrust_collar",
    )
    faceplate.visual(
        Cylinder(radius=0.060, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=satin_steel,
        name="faceplate_boss",
    )
    faceplate.visual(
        mesh_from_cadquery(_faceplate(0.122, 0.026, 0.080, 0.007), "flange_disk"),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=satin_steel,
        name="flange_disk",
    )
    for index in range(4):
        angle = index * math.pi / 2.0
        faceplate.visual(
            Cylinder(radius=0.014, length=0.007),
            origin=Origin(
                xyz=(0.080 * math.cos(angle), 0.080 * math.sin(angle), 0.0775),
            ),
            material=dark_cast,
            name=f"bolt_head_{index}",
        )
    faceplate.visual(
        Box((0.050, 0.018, 0.009)),
        origin=Origin(xyz=(0.078, 0.0, 0.0815)),
        material=red_index,
        name="index_lug",
    )

    model.articulation(
        "base_bearing",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "offset_bearing",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=faceplate,
        origin=Origin(xyz=(0.550, 0.0, 0.255)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=3.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    faceplate = object_model.get_part("faceplate")
    base_bearing = object_model.get_articulation("base_bearing")
    offset_bearing = object_model.get_articulation("offset_bearing")

    ctx.expect_within(
        turntable,
        base,
        axes="xy",
        inner_elem="lower_spigot",
        outer_elem="base_bearing_ring",
        margin=0.0,
        name="lower spigot is centered in the base bearing bore",
    )
    ctx.expect_overlap(
        turntable,
        base,
        axes="z",
        elem_a="lower_spigot",
        elem_b="base_bearing_ring",
        min_overlap=0.025,
        name="lower spigot remains inserted in base bearing",
    )
    ctx.expect_gap(
        turntable,
        base,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="base_bearing_ring",
        min_gap=0.0,
        max_gap=0.001,
        name="turntable clears the stationary bearing race",
    )

    ctx.expect_within(
        faceplate,
        turntable,
        axes="xy",
        inner_elem="upper_shaft",
        outer_elem="upper_bearing_column",
        margin=0.0,
        name="upper shaft is centered in the offset bearing bore",
    )
    ctx.expect_overlap(
        faceplate,
        turntable,
        axes="z",
        elem_a="upper_shaft",
        elem_b="upper_bearing_column",
        min_overlap=0.070,
        name="upper shaft remains inserted in offset bearing",
    )
    ctx.expect_contact(
        faceplate,
        turntable,
        elem_a="thrust_collar",
        elem_b="upper_bearing_column",
        contact_tol=0.001,
        name="upper thrust collar bears on offset support",
    )

    lower_pos = ctx.part_world_position(turntable)
    upper_pos = ctx.part_world_position(faceplate)
    ctx.check(
        "upper rotary axis is laterally offset from base axis",
        lower_pos is not None
        and upper_pos is not None
        and abs((upper_pos[0] - lower_pos[0]) - 0.550) < 0.005
        and abs(upper_pos[1] - lower_pos[1]) < 0.005,
        details=f"lower={lower_pos}, upper={upper_pos}",
    )

    with ctx.pose({base_bearing: math.pi / 2.0}):
        carried_pos = ctx.part_world_position(faceplate)
    ctx.check(
        "base turntable carries the offset axis around the main axis",
        carried_pos is not None and abs(carried_pos[0]) < 0.020 and carried_pos[1] > 0.520,
        details=f"carried_pos={carried_pos}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    lug_rest = _aabb_center(ctx.part_element_world_aabb(faceplate, elem="index_lug"))
    with ctx.pose({offset_bearing: math.pi / 2.0}):
        lug_rotated = _aabb_center(ctx.part_element_world_aabb(faceplate, elem="index_lug"))
    ctx.check(
        "upper faceplate rotates about its own parallel axis",
        upper_pos is not None
        and lug_rest is not None
        and lug_rotated is not None
        and lug_rest[0] > upper_pos[0] + 0.060
        and lug_rotated[1] > upper_pos[1] + 0.060,
        details=f"upper_axis={upper_pos}, lug_rest={lug_rest}, lug_rotated={lug_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
