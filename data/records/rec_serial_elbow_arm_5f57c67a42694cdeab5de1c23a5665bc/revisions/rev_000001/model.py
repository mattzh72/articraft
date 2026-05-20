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
    mesh_from_geometry,
)


def _capsule_profile(
    x0: float,
    x1: float,
    radius: float,
    *,
    segments: int = 16,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    """2-D stadium profile in the X/Z drawing plane.

    The mesh extruder works in local XY; here the second coordinate is the
    link's vertical Z coordinate before the mesh is rotated into place.
    """

    pts: list[tuple[float, float]] = []
    for i in range(segments + 1):
        angle = -math.pi / 2.0 + math.pi * i / segments
        pts.append((x1 + radius * math.cos(angle), radius * math.sin(angle)))
    for i in range(segments + 1):
        angle = math.pi / 2.0 + math.pi * i / segments
        pts.append((x0 + radius * math.cos(angle), radius * math.sin(angle)))
    if clockwise:
        pts.reverse()
    return pts


def _circle_profile(
    cx: float,
    cz: float,
    radius: float,
    *,
    segments: int = 32,
    clockwise: bool = True,
) -> list[tuple[float, float]]:
    pts = [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cz + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]
    if clockwise:
        pts.reverse()
    return pts


def _link_plate_mesh(
    length: float,
    outer_radius: float,
    pivot_hole_radius: float,
    thickness: float,
    name: str,
    *,
    slot_radius: float | None = None,
) -> object:
    holes = [
        _circle_profile(0.0, 0.0, pivot_hole_radius),
        _circle_profile(length, 0.0, pivot_hole_radius),
    ]
    if slot_radius is not None:
        holes.append(_capsule_profile(length * 0.34, length * 0.66, slot_radius, clockwise=True))
    geom = ExtrudeWithHolesGeometry(
        _capsule_profile(0.0, length, outer_radius),
        holes,
        thickness,
        center=True,
    )
    # ExtrudeWithHolesGeometry extrudes along local Z.  Rotate so the extruded
    # thickness becomes the arm's local Y thickness and the profile remains in
    # the X/Z swing plane.
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_elbow_arm")

    plate_paint = model.material("dark_powder_coat", rgba=(0.09, 0.10, 0.11, 1.0))
    shoulder_casting = model.material("cast_aluminum", rgba=(0.58, 0.61, 0.64, 1.0))
    link_finish = model.material("brushed_steel", rgba=(0.72, 0.73, 0.70, 1.0))
    pin_finish = model.material("black_oxide_pins", rgba=(0.02, 0.02, 0.018, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        Box((0.28, 0.035, 0.76)),
        origin=Origin(xyz=(0.0, -0.200, 0.0)),
        material=plate_paint,
        name="wall_plate",
    )
    side_plate.visual(
        Box((0.20, 0.155, 0.18)),
        origin=Origin(xyz=(0.0, -0.110, 0.0)),
        material=shoulder_casting,
        name="shoulder_block",
    )
    # Side ribs make the projecting block read as a grounded wall casting.
    for x in (-0.085, 0.085):
        side_plate.visual(
            Box((0.030, 0.150, 0.245)),
            origin=Origin(xyz=(x, -0.108, 0.0)),
            material=shoulder_casting,
            name=f"side_rib_{'neg' if x < 0.0 else 'pos'}",
        )
    side_plate.visual(
        Cylinder(radius=0.068, length=0.049),
        origin=Origin(xyz=(0.0, -0.0105, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_finish,
        name="shoulder_boss",
    )
    for i, (x, z) in enumerate(((-0.095, -0.275), (0.095, -0.275), (-0.095, 0.275), (0.095, 0.275))):
        side_plate.visual(
            Cylinder(radius=0.016, length=0.012),
            origin=Origin(xyz=(x, -0.178, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=pin_finish,
            name=f"wall_bolt_{i}",
        )

    upper_length = 0.78
    upper_link = model.part("upper_link")
    upper_link.visual(
        _link_plate_mesh(upper_length, 0.055, 0.022, 0.032, "upper_link_plate", slot_radius=0.017),
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
        material=link_finish,
        name="upper_plate",
    )
    upper_link.visual(
        Cylinder(radius=0.033, length=0.014),
        origin=Origin(xyz=(0.0, 0.052, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_finish,
        name="shoulder_cap",
    )
    upper_link.visual(
        Cylinder(radius=0.030, length=0.024),
        origin=Origin(xyz=(upper_length, 0.055, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_finish,
        name="elbow_spacer",
    )

    distal_length = 0.46
    distal_link = model.part("distal_link")
    distal_link.visual(
        _link_plate_mesh(distal_length, 0.045, 0.019, 0.030, "distal_link_plate", slot_radius=0.013),
        origin=Origin(xyz=(0.0, 0.082, 0.0)),
        material=link_finish,
        name="distal_plate",
    )
    distal_link.visual(
        Cylinder(radius=0.029, length=0.012),
        origin=Origin(xyz=(0.0, 0.103, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_finish,
        name="elbow_cap",
    )
    distal_link.visual(
        Cylinder(radius=0.023, length=0.012),
        origin=Origin(xyz=(distal_length, 0.103, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pin_finish,
        name="distal_mount_cap",
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=side_plate,
        child=upper_link,
        origin=Origin(),
        # The upper link extends along local +X from the shoulder.  -Y makes
        # positive joint motion lift the elbow upward in the wall-side view.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.4,
            lower=math.radians(-70.0),
            upper=math.radians(95.0),
        ),
    )

    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=distal_link,
        origin=Origin(xyz=(upper_length, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.8,
            lower=math.radians(-130.0),
            upper=math.radians(130.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    side_plate = object_model.get_part("side_plate")
    upper_link = object_model.get_part("upper_link")
    distal_link = object_model.get_part("distal_link")
    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")

    ctx.expect_gap(
        upper_link,
        side_plate,
        axis="y",
        max_gap=0.002,
        positive_elem="upper_plate",
        negative_elem="shoulder_boss",
        max_penetration=0.000001,
        name="upper link seats on shoulder boss",
    )
    ctx.expect_gap(
        distal_link,
        upper_link,
        axis="y",
        max_gap=0.002,
        positive_elem="distal_plate",
        negative_elem="elbow_spacer",
        max_penetration=0.000001,
        name="distal link seats on elbow spacer",
    )
    ctx.expect_overlap(
        upper_link,
        side_plate,
        axes="xz",
        min_overlap=0.040,
        elem_a="upper_plate",
        elem_b="shoulder_boss",
        name="shoulder pivot is coaxial with upper link eye",
    )
    ctx.expect_overlap(
        distal_link,
        upper_link,
        axes="xz",
        min_overlap=0.035,
        elem_a="distal_plate",
        elem_b="elbow_spacer",
        name="elbow pivot is coaxial with distal link eye",
    )

    rest_upper_aabb = ctx.part_element_world_aabb(upper_link, elem="upper_plate")
    with ctx.pose({shoulder_joint: math.radians(55.0), elbow_joint: math.radians(70.0)}):
        raised_upper_aabb = ctx.part_element_world_aabb(upper_link, elem="upper_plate")
        bent_distal_aabb = ctx.part_element_world_aabb(distal_link, elem="distal_plate")

    if rest_upper_aabb is not None and raised_upper_aabb is not None:
        rest_upper_z = (rest_upper_aabb[0][2] + rest_upper_aabb[1][2]) * 0.5
        raised_upper_z = (raised_upper_aabb[0][2] + raised_upper_aabb[1][2]) * 0.5
    else:
        rest_upper_z = raised_upper_z = None
    ctx.check(
        "positive shoulder motion raises the upper link",
        rest_upper_z is not None and raised_upper_z is not None and raised_upper_z > rest_upper_z + 0.20,
        details=f"rest_z={rest_upper_z}, raised_z={raised_upper_z}",
    )

    if raised_upper_aabb is not None and bent_distal_aabb is not None:
        raised_upper_max_x = raised_upper_aabb[1][0]
        bent_distal_max_x = bent_distal_aabb[1][0]
    else:
        raised_upper_max_x = bent_distal_max_x = None
    ctx.check(
        "positive elbow motion folds the distal link relative to the upper link",
        raised_upper_max_x is not None
        and bent_distal_max_x is not None
        and bent_distal_max_x < raised_upper_max_x + 0.28,
        details=f"upper_max_x={raised_upper_max_x}, distal_max_x={bent_distal_max_x}",
    )

    return ctx.report()


object_model = build_object_model()
