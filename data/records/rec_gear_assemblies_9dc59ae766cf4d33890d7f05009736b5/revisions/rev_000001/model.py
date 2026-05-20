from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


SHAFT_Z = 0.12
SHAFT_RADIUS = 0.009
SHAFT_LENGTH = 0.244
BOSS_RADIUS = 0.026
BOSS_LENGTH = 0.026
FRONT_BOSS_Y = 0.135
REAR_BOSS_Y = -0.135
GEAR_MODULE = 0.0045


SHAFT_SPECS = (
    # name, x position, teeth, material name
    ("input_shaft", -0.140, 16, "blued_steel"),
    ("idler_shaft", -0.035, 24, "warm_steel"),
    ("output_shaft", 0.111, 34, "brass"),
)


def _gear_mesh(name: str, teeth: int, width: float):
    """Create a simple extruded involute-like gear silhouette in meters."""
    pitch_radius = GEAR_MODULE * teeth * 0.5
    root_radius = pitch_radius - 1.15 * GEAR_MODULE
    outer_radius = pitch_radius + 1.00 * GEAR_MODULE
    geom = MeshGeometry()
    outline: list[tuple[float, float]] = []
    for i in range(teeth):
        tooth_angle = 2.0 * math.pi * i / teeth
        step = 2.0 * math.pi / teeth
        outline.extend(
            [
                (root_radius * math.cos(tooth_angle - 0.50 * step), root_radius * math.sin(tooth_angle - 0.50 * step)),
                (outer_radius * math.cos(tooth_angle - 0.23 * step), outer_radius * math.sin(tooth_angle - 0.23 * step)),
                (outer_radius * math.cos(tooth_angle + 0.23 * step), outer_radius * math.sin(tooth_angle + 0.23 * step)),
                (root_radius * math.cos(tooth_angle + 0.50 * step), root_radius * math.sin(tooth_angle + 0.50 * step)),
            ]
        )

    bottom: list[int] = []
    top: list[int] = []
    for x, y in outline:
        bottom.append(geom.add_vertex(x, y, -width * 0.5))
    for x, y in outline:
        top.append(geom.add_vertex(x, y, width * 0.5))
    bottom_center = geom.add_vertex(0.0, 0.0, -width * 0.5)
    top_center = geom.add_vertex(0.0, 0.0, width * 0.5)
    n = len(outline)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(bottom[i], bottom[j], top[j])
        geom.add_face(bottom[i], top[j], top[i])
        geom.add_face(bottom_center, bottom[i], bottom[j])
        geom.add_face(top_center, top[j], top[i])
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_reduction_gearbox")

    model.material("cast_aluminum", rgba=(0.58, 0.60, 0.58, 1.0))
    model.material("dark_bearing", rgba=(0.08, 0.085, 0.09, 1.0))
    model.material("shaft_steel", rgba=(0.66, 0.69, 0.70, 1.0))
    model.material("blued_steel", rgba=(0.24, 0.31, 0.38, 1.0))
    model.material("warm_steel", rgba=(0.56, 0.53, 0.48, 1.0))
    model.material("brass", rgba=(0.86, 0.64, 0.25, 1.0))
    model.material("rubber_foot", rgba=(0.03, 0.03, 0.028, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.56, 0.34, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material="cast_aluminum",
        name="floor",
    )
    for y, name in ((0.159, "front_end_plate"), (-0.159, "rear_end_plate")):
        housing.visual(
            Box((0.56, 0.022, 0.205)),
            origin=Origin(xyz=(0.0, y, 0.1025)),
            material="cast_aluminum",
            name=name,
        )
    for x, name in ((0.269, "side_wall_0"), (-0.269, "side_wall_1")):
        housing.visual(
            Box((0.022, 0.34, 0.080)),
            origin=Origin(xyz=(x, 0.0, 0.040)),
            material="cast_aluminum",
            name=name,
        )

    # Four low mounting feet are fused visually into the casting at the corners.
    for ix, x in enumerate((-0.225, 0.225)):
        for iy, y in enumerate((-0.122, 0.122)):
            housing.visual(
                Cylinder(radius=0.024, length=0.014),
                origin=Origin(xyz=(x, y, -0.007)),
                material="rubber_foot",
                name=f"foot_{ix}_{iy}",
            )

    # Solid bearing bosses protrude inward from the two end plates.  The rotating
    # shafts stop at their flat faces, so the support is visible without hiding
    # an overlap in the fixed housing.
    for shaft_name, x, _teeth, _mat in SHAFT_SPECS:
        short = shaft_name.replace("_shaft", "")
        for y, side in ((FRONT_BOSS_Y, "front"), (REAR_BOSS_Y, "rear")):
            housing.visual(
                Cylinder(radius=BOSS_RADIUS, length=BOSS_LENGTH),
                origin=Origin(
                    xyz=(x, y, SHAFT_Z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="dark_bearing",
                name=f"{side}_boss_{short}",
            )

    for shaft_name, x, teeth, gear_material in SHAFT_SPECS:
        shaft = model.part(shaft_name)
        shaft.visual(
            Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="shaft_steel",
            name="shaft_core",
        )
        shaft.visual(
            _gear_mesh(f"{shaft_name}_gear", teeth, 0.048),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=gear_material,
            name=f"{shaft_name.replace('_shaft', '')}_gear",
        )
        shaft.visual(
            Cylinder(radius=0.015, length=0.060),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="shaft_steel",
            name="hub_sleeve",
        )
        model.articulation(
            f"housing_to_{shaft_name}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=shaft,
            origin=Origin(xyz=(x, 0.0, SHAFT_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=12.0,
                lower=-2.0 * math.pi,
                upper=2.0 * math.pi,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    input_shaft = object_model.get_part("input_shaft")
    idler_shaft = object_model.get_part("idler_shaft")
    output_shaft = object_model.get_part("output_shaft")

    shaft_joints = [
        object_model.get_articulation("housing_to_input_shaft"),
        object_model.get_articulation("housing_to_idler_shaft"),
        object_model.get_articulation("housing_to_output_shaft"),
    ]
    ctx.check(
        "three parallel revolute shaft joints",
        len(shaft_joints) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in shaft_joints)
        and all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in shaft_joints),
        details=f"joints={[j.name for j in shaft_joints]}",
    )

    for shaft_name, _x, _teeth, _mat in SHAFT_SPECS:
        shaft = object_model.get_part(shaft_name)
        gear_elem = f"{shaft_name.replace('_shaft', '')}_gear"
        short = shaft_name.replace("_shaft", "")
        ctx.expect_gap(
            shaft,
            housing,
            axis="z",
            min_gap=0.006,
            positive_elem=gear_elem,
            negative_elem="floor",
            name=f"{short} gear clears the open sump floor",
        )
        ctx.expect_contact(
            shaft,
            housing,
            elem_a="shaft_core",
            elem_b=f"front_boss_{short}",
            contact_tol=0.0015,
            name=f"{short} shaft seats in front bearing boss",
        )
        ctx.expect_contact(
            shaft,
            housing,
            elem_a="shaft_core",
            elem_b=f"rear_boss_{short}",
            contact_tol=0.0015,
            name=f"{short} shaft seats in rear bearing boss",
        )

    ctx.expect_gap(
        idler_shaft,
        input_shaft,
        axis="x",
        min_gap=0.002,
        max_gap=0.014,
        positive_elem="idler_gear",
        negative_elem="input_gear",
        name="input and idler gear teeth are close but clear",
    )
    ctx.expect_gap(
        output_shaft,
        idler_shaft,
        axis="x",
        min_gap=0.002,
        max_gap=0.014,
        positive_elem="output_gear",
        negative_elem="idler_gear",
        name="idler and output gear teeth are close but clear",
    )

    rest_positions = [ctx.part_world_position(object_model.get_part(spec[0])) for spec in SHAFT_SPECS]
    with ctx.pose({shaft_joints[0]: math.pi / 2.0, shaft_joints[1]: -math.pi / 3.0, shaft_joints[2]: math.pi / 4.0}):
        rotated_positions = [
            ctx.part_world_position(object_model.get_part(spec[0])) for spec in SHAFT_SPECS
        ]
    ctx.check(
        "rotating shafts spin in place on bearing axes",
        rest_positions == rotated_positions and all(pos is not None for pos in rest_positions),
        details=f"rest={rest_positions}, rotated={rotated_positions}",
    )

    return ctx.report()


object_model = build_object_model()
