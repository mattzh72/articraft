from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _annular_cylinder(
    *,
    inner_radius: float,
    outer_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 96,
) -> MeshGeometry:
    """Create a connected annular cylinder mesh in the local XY plane."""
    geom = MeshGeometry()
    for i in range(segments):
        angle = 2.0 * math.pi * i / segments
        c = math.cos(angle)
        s = math.sin(angle)
        geom.add_vertex(outer_radius * c, outer_radius * s, z_min)
        geom.add_vertex(outer_radius * c, outer_radius * s, z_max)
        geom.add_vertex(inner_radius * c, inner_radius * s, z_min)
        geom.add_vertex(inner_radius * c, inner_radius * s, z_max)

    for i in range(segments):
        j = (i + 1) % segments
        ob_i, ot_i, ib_i, it_i = 4 * i, 4 * i + 1, 4 * i + 2, 4 * i + 3
        ob_j, ot_j, ib_j, it_j = 4 * j, 4 * j + 1, 4 * j + 2, 4 * j + 3

        # Outer cylindrical wall.
        geom.add_face(ob_i, ob_j, ot_j)
        geom.add_face(ob_i, ot_j, ot_i)
        # Inner bore wall.
        geom.add_face(ib_i, it_j, ib_j)
        geom.add_face(ib_i, it_i, it_j)
        # Top and bottom annular faces.
        geom.add_face(ot_i, ot_j, it_j)
        geom.add_face(ot_i, it_j, it_i)
        geom.add_face(ob_i, ib_j, ob_j)
        geom.add_face(ob_i, ib_i, ib_j)

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_wall_thermostat")

    warm_white = model.material("warm_white", color=(0.88, 0.87, 0.82, 1.0))
    satin_white = model.material("satin_white", color=(0.96, 0.95, 0.90, 1.0))
    datum_gray = model.material("datum_gray", color=(0.18, 0.19, 0.18, 1.0))
    shadow_gray = model.material("shadow_gray", color=(0.08, 0.085, 0.08, 1.0))
    stainless = model.material("brushed_stainless", color=(0.63, 0.64, 0.61, 1.0))
    amber = model.material("calibration_amber", color=(0.92, 0.56, 0.18, 1.0))

    body = model.part("body")
    body_profile = rounded_rect_profile(0.132, 0.096, 0.013, corner_segments=10)
    body.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(body_profile, 0.014),
            "rounded_wall_plate",
        ),
        material=warm_white,
        name="rounded_wall_plate",
    )
    body.visual(
        mesh_from_geometry(
            _annular_cylinder(
                inner_radius=0.041,
                outer_radius=0.052,
                z_min=0.0140,
                z_max=0.0152,
            ),
            "datum_ring",
        ),
        material=satin_white,
        name="datum_ring",
    )

    # A small bottom datum shelf makes the thermostat read as a calibrated
    # instrument rather than a generic consumer knob plate.
    body.visual(
        Box((0.070, 0.004, 0.0014)),
        origin=Origin(xyz=(0.0, -0.044, 0.0146)),
        material=datum_gray,
        name="bottom_datum_bar",
    )

    # Static index marks embedded slightly into the face so every mark is
    # mechanically connected to the molded wall plate.
    for i in range(25):
        angle = math.radians(-120.0 + i * 10.0)
        is_major = i % 6 == 0
        is_mid = i % 3 == 0
        length = 0.0070 if is_major else (0.0050 if is_mid else 0.0032)
        width = 0.00125 if is_major else 0.00085
        r = 0.0475
        body.visual(
            Box((length, width, 0.0009)),
            origin=Origin(
                xyz=(r * math.sin(angle), r * math.cos(angle), 0.01445),
                rpy=(0.0, 0.0, -angle),
            ),
            material=datum_gray,
            name=f"index_mark_{i}",
        )

    # Raised calibration screw seats on the body: the moving screw heads sit on
    # these bosses and rotate around explicit coaxial joints.
    for suffix, x in (("0", -0.044), ("1", 0.044)):
        body.visual(
            Cylinder(radius=0.0085, length=0.0018),
            origin=Origin(xyz=(x, -0.032, 0.0149)),
            material=satin_white,
            name=f"screw_boss_{suffix}",
        )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=0.0058, length=0.0130),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=stainless,
        name="center_shaft",
    )
    shaft.visual(
        Cylinder(radius=0.0115, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0145)),
        material=stainless,
        name="retainer_cap",
    )
    shaft.visual(
        Box((0.014, 0.0022, 0.0005)),
        origin=Origin(xyz=(0.0, 0.0, 0.01625)),
        material=shadow_gray,
        name="retainer_slot",
    )

    model.articulation(
        "body_to_shaft",
        ArticulationType.FIXED,
        parent=body,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            _annular_cylinder(
                inner_radius=0.0072,
                outer_radius=0.0370,
                z_min=0.0020,
                z_max=0.0120,
            ),
            "rotary_dial_annulus",
        ),
        material=satin_white,
        name="rotary_annulus",
    )
    dial.visual(
        mesh_from_geometry(
            _annular_cylinder(
                inner_radius=0.0072,
                outer_radius=0.0132,
                z_min=0.0118,
                z_max=0.0130,
                segments=72,
            ),
            "dial_bore_chamfer",
        ),
        material=warm_white,
        name="bore_chamfer",
    )
    dial.visual(
        Box((0.0030, 0.0220, 0.0008)),
        origin=Origin(xyz=(0.0, 0.0230, 0.0123)),
        material=shadow_gray,
        name="datum_pointer",
    )
    dial.visual(
        Box((0.0180, 0.0045, 0.0009)),
        origin=Origin(xyz=(0.0, -0.024, 0.01235)),
        material=datum_gray,
        name="thumb_flat",
    )
    for i in range(18):
        angle = 2.0 * math.pi * i / 18.0
        r = 0.0375
        dial.visual(
            Box((0.0040, 0.0054, 0.0080)),
            origin=Origin(
                xyz=(r * math.cos(angle), r * math.sin(angle), 0.0070),
                rpy=(0.0, 0.0, angle),
            ),
            material=satin_white,
            name=f"grip_rib_{i}",
        )

    model.articulation(
        "shaft_to_dial",
        ArticulationType.REVOLUTE,
        parent=shaft,
        child=dial,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=math.radians(-130.0),
            upper=math.radians(130.0),
            effort=0.18,
            velocity=1.2,
        ),
        motion_properties=MotionProperties(damping=0.025, friction=0.015),
    )

    for suffix, x in (("0", -0.044), ("1", 0.044)):
        screw = model.part(f"trim_screw_{suffix}")
        screw.visual(
            Cylinder(radius=0.0062, length=0.0030),
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            material=stainless,
            name="screw_head",
        )
        screw.visual(
            Box((0.0105, 0.0016, 0.0006)),
            origin=Origin(xyz=(0.0, 0.0, 0.0033)),
            material=shadow_gray,
            name="screw_slot",
        )
        screw.visual(
            Box((0.0025, 0.0016, 0.0007)),
            origin=Origin(xyz=(0.0048, 0.0, 0.00335)),
            material=amber,
            name="calibration_witness",
        )
        model.articulation(
            f"body_to_trim_screw_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=screw,
            origin=Origin(xyz=(x, -0.032, 0.0158)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.04, velocity=2.0),
            motion_properties=MotionProperties(damping=0.01, friction=0.005),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    shaft = object_model.get_part("shaft")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("shaft_to_dial")

    ctx.expect_within(
        dial,
        body,
        axes="xy",
        margin=0.0,
        name="dial footprint stays inside wall plate",
    )
    ctx.expect_origin_distance(
        dial,
        shaft,
        axes="xy",
        max_dist=0.0001,
        name="dial axis is coaxial with shaft",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="z",
        min_gap=0.0006,
        max_gap=0.0030,
        negative_elem="datum_ring",
        name="controlled dial-to-datum-ring air gap",
    )
    ctx.expect_gap(
        shaft,
        dial,
        axis="z",
        min_gap=0.0006,
        max_gap=0.0030,
        positive_elem="retainer_cap",
        negative_elem="rotary_annulus",
        name="retainer cap clears dial front face",
    )
    ctx.expect_overlap(
        shaft,
        dial,
        axes="xy",
        min_overlap=0.006,
        elem_a="retainer_cap",
        elem_b="rotary_annulus",
        name="retainer overlaps dial bore footprint for capture",
    )

    pointer_rest = ctx.part_element_world_aabb(dial, elem="datum_pointer")
    with ctx.pose({dial_joint: math.radians(90.0)}):
        pointer_rotated = ctx.part_element_world_aabb(dial, elem="datum_pointer")
        ctx.expect_origin_distance(
            dial,
            shaft,
            axes="xy",
            max_dist=0.0001,
            name="dial remains coaxial while rotated",
        )
    ctx.check(
        "datum pointer visibly rotates about center",
        pointer_rest is not None
        and pointer_rotated is not None
        and abs(pointer_rest[0][0] - pointer_rotated[0][0]) > 0.010,
        details=f"rest={pointer_rest}, rotated={pointer_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
