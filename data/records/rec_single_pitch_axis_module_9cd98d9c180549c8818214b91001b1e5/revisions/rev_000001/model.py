from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _cheek_geometry() -> ExtrudeWithHolesGeometry:
    # The profile is drawn as local (Y, Z-relative-to-trunnion) and then rotated
    # so the extrusion thickness becomes the world X cheek thickness.
    half_depth = 0.050
    bottom = -0.070
    top = 0.040
    radius = 0.010
    outer = [
        (-half_depth + radius, bottom),
        (half_depth - radius, bottom),
        (half_depth, bottom + radius),
        (half_depth, top - radius),
        (half_depth - radius, top),
        (-half_depth + radius, top),
        (-half_depth, top - radius),
        (-half_depth, bottom + radius),
    ]
    hole = _circle_profile(0.0215, segments=48)
    geom = ExtrudeWithHolesGeometry(outer, [hole], 0.030, center=True)
    geom.rotate_y(math.pi / 2.0).rotate_x(math.pi / 2.0)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_cheek_pitch_trunnion_head")

    cast_iron = model.material("cast_iron", rgba=(0.30, 0.32, 0.34, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.08, 0.09, 0.10, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.68, 0.70, 0.72, 1.0))
    face_black = model.material("face_black", rgba=(0.13, 0.14, 0.15, 1.0))

    base_plate_thickness = 0.018
    yoke_bottom_z = base_plate_thickness - 0.002
    trunnion_z = yoke_bottom_z + 0.090

    base = model.part("base")
    base.visual(
        Box((0.280, 0.180, base_plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, base_plate_thickness * 0.5)),
        material=dark_oxide,
        name="base_plate",
    )

    base.visual(
        Box((0.205, 0.115, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, yoke_bottom_z + 0.010)),
        material=cast_iron,
        name="yoke_foot",
    )

    cheek_mesh = mesh_from_geometry(_cheek_geometry(), "short_trunnion_cheek")
    for cheek_name, x in (("cheek_0", -0.075), ("cheek_1", 0.075)):
        base.visual(
            cheek_mesh,
            origin=Origin(xyz=(x, 0.0, trunnion_z)),
            material=cast_iron,
            name=cheek_name,
        )

    base.visual(
        Box((0.150, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.048, yoke_bottom_z + 0.047)),
        material=cast_iron,
        name="rear_tie_rib",
    )
    base.visual(
        Box((0.150, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.048, yoke_bottom_z + 0.047)),
        material=cast_iron,
        name="front_tie_rib",
    )

    bearing_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.024, tube=0.0048, radial_segments=16, tubular_segments=48),
        "trunnion_bearing_ring",
    )
    for side, x in enumerate((-0.093, 0.093)):
        base.visual(
            bearing_ring_mesh,
            origin=Origin(xyz=(x, 0.0, trunnion_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_steel,
            name=f"bearing_ring_{side}",
        )

    for ix, x in enumerate((-0.115, 0.115)):
        for iy, y in enumerate((-0.065, 0.065)):
            base.visual(
                Cylinder(radius=0.008, length=0.006),
                origin=Origin(xyz=(x, y, base_plate_thickness + 0.002)),
                material=satin_steel,
                name=f"mount_bolt_{ix}_{iy}",
            )

    face = model.part("face")
    face.visual(
        Box((0.100, 0.030, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=face_black,
        name="tilting_body",
    )
    face.visual(
        Cylinder(radius=0.026, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="center_barrel",
    )
    face.visual(
        Cylinder(radius=0.020, length=0.184),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="trunnion_shaft",
    )
    face.visual(
        Box((0.090, 0.008, 0.066)),
        origin=Origin(xyz=(0.0, -0.023, 0.0)),
        material=satin_steel,
        name="mounting_face",
    )

    for ix, x in enumerate((-0.030, 0.030)):
        for iz, z in enumerate((-0.023, 0.023)):
            face.visual(
                Cylinder(radius=0.004, length=0.004),
                origin=Origin(xyz=(x, -0.0285, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark_oxide,
                name=f"face_screw_{ix}_{iz}",
            )

    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=face,
        origin=Origin(xyz=(0.0, 0.0, trunnion_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=-0.60, upper=0.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    face = object_model.get_part("face")
    pitch = object_model.get_articulation("pitch_axis")

    ctx.check(
        "single pitch revolute joint",
        len(object_model.articulations) == 1
        and pitch.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations}",
    )
    ctx.allow_overlap(
        base,
        face,
        elem_a="cheek_0",
        elem_b="trunnion_shaft",
        reason="The rotating trunnion shaft is intentionally captured through the cheek bore.",
    )
    ctx.allow_overlap(
        base,
        face,
        elem_a="cheek_1",
        elem_b="trunnion_shaft",
        reason="The rotating trunnion shaft is intentionally captured through the cheek bore.",
    )
    ctx.expect_within(
        face,
        base,
        axes="x",
        inner_elem="tilting_body",
        outer_elem="yoke_foot",
        margin=0.0,
        name="tilting face sits between cheek sides",
    )
    ctx.expect_gap(
        face,
        base,
        axis="z",
        positive_elem="tilting_body",
        negative_elem="base_plate",
        min_gap=0.020,
        name="moving face clears the grounded plate",
    )
    ctx.expect_overlap(
        face,
        base,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="cheek_0",
        min_overlap=0.020,
        name="shaft enters cheek 0",
    )
    ctx.expect_overlap(
        face,
        base,
        axes="x",
        elem_a="trunnion_shaft",
        elem_b="cheek_1",
        min_overlap=0.020,
        name="shaft enters cheek 1",
    )

    rest_aabb = ctx.part_element_world_aabb(face, elem="mounting_face")
    with ctx.pose({pitch: 0.50}):
        tilted_aabb = ctx.part_element_world_aabb(face, elem="mounting_face")

    rest_depth = None if rest_aabb is None else rest_aabb[1][1] - rest_aabb[0][1]
    tilted_depth = None if tilted_aabb is None else tilted_aabb[1][1] - tilted_aabb[0][1]
    ctx.check(
        "pitch pose visibly tilts the face",
        rest_depth is not None and tilted_depth is not None and tilted_depth > rest_depth + 0.020,
        details=f"rest_y_depth={rest_depth}, tilted_y_depth={tilted_depth}",
    )

    return ctx.report()


object_model = build_object_model()
