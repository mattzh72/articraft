from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MeshGeometry,
    Mimic,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


GEAR_THICKNESS = 0.006
GEAR_CENTER_Z = 0.012
BODY_THICKNESS = 0.006
SUN_TEETH = 24
RING_TEETH = 18
MODULE = 0.002
SUN_PITCH_RADIUS = MODULE * SUN_TEETH / 2.0
RING_PITCH_RADIUS = MODULE * RING_TEETH / 2.0
SUN_OUTER_RADIUS = SUN_PITCH_RADIUS + MODULE
RING_OUTER_RADIUS = RING_PITCH_RADIUS + MODULE
# A tiny tip clearance prevents tooth-solid collisions in the current pose while
# keeping the external gears visually tangent and mechanically coupled by mimic.
GEAR_CENTER_RADIUS = SUN_OUTER_RADIUS + RING_OUTER_RADIUS + 0.0004


def _circle_profile(radius: float, *, segments: int, reverse: bool = False) -> list[tuple[float, float]]:
    points = [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]
    if reverse:
        points.reverse()
    return points


def _spur_ring_profile(
    teeth: int,
    pitch_radius: float,
    *,
    addendum: float,
    dedendum: float,
    phase: float = 0.0,
) -> list[tuple[float, float]]:
    """Approximate an external spur gear as a connected extruded toothed ring."""

    root_radius = pitch_radius - dedendum
    outer_radius = pitch_radius + addendum
    pitch = 2.0 * math.pi / teeth
    profile: list[tuple[float, float]] = []

    for tooth_index in range(teeth):
        center = phase + tooth_index * pitch
        for fraction, radius in (
            (-0.50, root_radius),
            (-0.30, root_radius),
            (-0.17, outer_radius),
            (0.17, outer_radius),
            (0.30, root_radius),
        ):
            angle = center + fraction * pitch
            profile.append((radius * math.cos(angle), radius * math.sin(angle)))
    return profile


def _mesh_from_annular_profiles(
    outer_profile: list[tuple[float, float]],
    *,
    inner_radius: float,
    height: float,
    name: str,
):
    geom = MeshGeometry()
    bottom_outer: list[int] = []
    bottom_inner: list[int] = []
    top_outer: list[int] = []
    top_inner: list[int] = []
    half_height = height / 2.0

    for x, y in outer_profile:
        angle = math.atan2(y, x)
        ix = inner_radius * math.cos(angle)
        iy = inner_radius * math.sin(angle)
        bottom_outer.append(geom.add_vertex(x, y, -half_height))
        bottom_inner.append(geom.add_vertex(ix, iy, -half_height))
        top_outer.append(geom.add_vertex(x, y, half_height))
        top_inner.append(geom.add_vertex(ix, iy, half_height))

    count = len(outer_profile)
    for index in range(count):
        next_index = (index + 1) % count

        bo0 = bottom_outer[index]
        bo1 = bottom_outer[next_index]
        bi0 = bottom_inner[index]
        bi1 = bottom_inner[next_index]
        to0 = top_outer[index]
        to1 = top_outer[next_index]
        ti0 = top_inner[index]
        ti1 = top_inner[next_index]

        # Outer tooth/root wall.
        geom.add_face(bo0, bo1, to1)
        geom.add_face(bo0, to1, to0)
        # Inner bore wall.
        geom.add_face(bi0, ti1, bi1)
        geom.add_face(bi0, ti0, ti1)
        # Top annular face.
        geom.add_face(to0, to1, ti1)
        geom.add_face(to0, ti1, ti0)
        # Bottom annular face.
        geom.add_face(bo0, bi1, bo1)
        geom.add_face(bo0, bi0, bi1)

    return mesh_from_geometry(geom, name)


def _annular_disc_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    segments: int,
    name: str,
):
    return _mesh_from_annular_profiles(
        _circle_profile(outer_radius, segments=segments),
        inner_radius=inner_radius,
        height=height,
        name=name,
    )


def _spur_gear_mesh(
    *,
    teeth: int,
    pitch_radius: float,
    bore_radius: float,
    phase: float,
    name: str,
):
    outer_profile = _spur_ring_profile(
        teeth,
        pitch_radius,
        addendum=MODULE,
        dedendum=1.15 * MODULE,
        phase=phase,
    )
    return _mesh_from_annular_profiles(
        outer_profile,
        inner_radius=bore_radius,
        height=GEAR_THICKNESS,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gear_ring_fidget")

    body_plastic = model.material("matte_graphite", rgba=(0.055, 0.060, 0.068, 1.0))
    axle_metal = model.material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    sun_material = model.material("warm_sun_gear", rgba=(0.96, 0.63, 0.13, 1.0))
    ring_material = model.material("teal_ring_gears", rgba=(0.02, 0.58, 0.66, 1.0))
    pocket_material = model.material("recess_shadow", rgba=(0.030, 0.034, 0.040, 1.0))

    body = model.part("disc_body")
    body.visual(
        Cylinder(radius=0.070, length=BODY_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BODY_THICKNESS / 2.0)),
        material=body_plastic,
        name="disc_plate",
    )
    body.visual(
        Cylinder(radius=0.029, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, BODY_THICKNESS + 0.0008)),
        material=pocket_material,
        name="sun_pocket",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, BODY_THICKNESS + 0.0015)),
        material=axle_metal,
        name="sun_lower_washer",
    )
    body.visual(
        Cylinder(radius=0.0032, length=0.0115),
        origin=Origin(xyz=(0.0, 0.0, BODY_THICKNESS + 0.00575)),
        material=axle_metal,
        name="sun_axle",
    )
    body.visual(
        Cylinder(radius=0.0060, length=0.0024),
        origin=Origin(xyz=(0.0, 0.0, GEAR_CENTER_Z + GEAR_THICKNESS / 2.0 + 0.0036)),
        material=axle_metal,
        name="sun_axle_cap",
    )

    outer_centers: list[tuple[float, float, float]] = []
    for index in range(3):
        angle = 2.0 * math.pi * index / 3.0
        x = GEAR_CENTER_RADIUS * math.cos(angle)
        y = GEAR_CENTER_RADIUS * math.sin(angle)
        outer_centers.append((x, y, angle))
        body.visual(
            Cylinder(radius=0.024, length=0.0016),
            origin=Origin(xyz=(x, y, BODY_THICKNESS + 0.0008)),
            material=pocket_material,
            name=f"ring_pocket_{index}",
        )
        body.visual(
            Cylinder(radius=0.0115, length=0.0030),
            origin=Origin(xyz=(x, y, BODY_THICKNESS + 0.0015)),
            material=axle_metal,
            name=f"ring_lower_washer_{index}",
        )
        body.visual(
            Cylinder(radius=0.0040, length=0.0115),
            origin=Origin(xyz=(x, y, BODY_THICKNESS + 0.00575)),
            material=axle_metal,
            name=f"ring_axle_{index}",
        )
        body.visual(
            Cylinder(radius=0.0130, length=0.0024),
            origin=Origin(xyz=(x, y, GEAR_CENTER_Z + GEAR_THICKNESS / 2.0 + 0.0036)),
            material=axle_metal,
            name=f"ring_axle_cap_{index}",
        )

    sun_mesh = _spur_gear_mesh(
        teeth=SUN_TEETH,
        pitch_radius=SUN_PITCH_RADIUS,
        bore_radius=0.0042,
        phase=0.0,
        name="sun_spur_gear",
    )
    ring_meshes = [
        _spur_gear_mesh(
            teeth=RING_TEETH,
            pitch_radius=RING_PITCH_RADIUS,
            bore_radius=0.0105,
            phase=angle + math.pi - math.pi / RING_TEETH,
            name=f"outer_spur_ring_{index}",
        )
        for index, (_, _, angle) in enumerate(outer_centers)
    ]

    sun_gear = model.part("sun_gear")
    sun_gear.visual(
        sun_mesh,
        origin=Origin(xyz=(0.0, 0.0, GEAR_CENTER_Z)),
        material=sun_material,
        name="gear_teeth",
    )
    sun_gear.visual(
        Cylinder(radius=0.0105, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, GEAR_CENTER_Z + GEAR_THICKNESS / 2.0 + 0.00075)),
        material=sun_material,
        name="raised_hub",
    )
    sun_gear.inertial = Inertial.from_geometry(
        Cylinder(radius=SUN_OUTER_RADIUS, length=GEAR_THICKNESS),
        mass=0.020,
        origin=Origin(xyz=(0.0, 0.0, GEAR_CENTER_Z)),
    )

    sun_joint = model.articulation(
        "body_to_sun_gear",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=sun_gear,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.12, velocity=18.0),
        motion_properties=MotionProperties(damping=0.004, friction=0.001),
    )

    for index, ((x, y, _angle), ring_mesh) in enumerate(zip(outer_centers, ring_meshes)):
        ring = model.part(f"ring_{index}")
        ring.visual(
            ring_mesh,
            origin=Origin(xyz=(0.0, 0.0, GEAR_CENTER_Z)),
            material=ring_material,
            name="gear_teeth",
        )
        ring.visual(
            _annular_disc_mesh(
                outer_radius=0.0142,
                inner_radius=0.0107,
                height=0.0014,
                segments=64,
                name=f"outer_spur_ring_bearing_rim_{index}",
            ),
            origin=Origin(xyz=(0.0, 0.0, GEAR_CENTER_Z + GEAR_THICKNESS / 2.0 + 0.0007)),
            material=ring_material,
            name="bearing_rim",
        )
        ring.inertial = Inertial.from_geometry(
            Cylinder(radius=RING_OUTER_RADIUS, length=GEAR_THICKNESS),
            mass=0.018,
            origin=Origin(xyz=(0.0, 0.0, GEAR_CENTER_Z)),
        )
        model.articulation(
            f"body_to_ring_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=ring,
            origin=Origin(xyz=(x, y, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.10, velocity=18.0),
            mimic=Mimic(
                joint=sun_joint.name,
                multiplier=-(SUN_TEETH / RING_TEETH),
                offset=0.0,
            ),
            motion_properties=MotionProperties(damping=0.004, friction=0.001),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("disc_body")
    sun = object_model.get_part("sun_gear")
    sun_joint = object_model.get_articulation("body_to_sun_gear")

    ctx.check(
        "central sun gear spins freely",
        sun_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={sun_joint.articulation_type}",
    )
    ctx.expect_gap(
        sun,
        body,
        axis="z",
        positive_elem="gear_teeth",
        negative_elem="disc_plate",
        min_gap=0.002,
        max_gap=0.004,
        name="sun gear clears the flat disc",
    )

    expected_ratio = -(SUN_TEETH / RING_TEETH)
    for index in range(3):
        ring = object_model.get_part(f"ring_{index}")
        joint = object_model.get_articulation(f"body_to_ring_{index}")
        mimic = joint.mimic
        ctx.check(
            f"ring {index} is geared to the sun",
            mimic is not None
            and mimic.joint == "body_to_sun_gear"
            and abs(mimic.multiplier - expected_ratio) < 1.0e-9,
            details=f"mimic={mimic}",
        )
        ctx.expect_origin_distance(
            ring,
            body,
            axes="xy",
            min_dist=GEAR_CENTER_RADIUS - 0.0005,
            max_dist=GEAR_CENTER_RADIUS + 0.0005,
            name=f"ring {index} axle sits on the disc perimeter",
        )
        ctx.expect_gap(
            ring,
            body,
            axis="z",
            positive_elem="gear_teeth",
            negative_elem="disc_plate",
            min_gap=0.002,
            max_gap=0.004,
            name=f"ring {index} gear clears the flat disc",
        )
        ctx.expect_contact(
            ring,
            sun,
            elem_a="gear_teeth",
            elem_b="gear_teeth",
            contact_tol=0.003,
            name=f"ring {index} teeth sit in mesh with sun",
        )

    return ctx.report()


object_model = build_object_model()
