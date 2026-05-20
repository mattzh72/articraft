from __future__ import annotations

import math

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


REAR_FLANGE_LEN = 0.004
APERTURE_SUPPORT_Z = REAR_FLANGE_LEN
APERTURE_SUPPORT_LEN = 0.012
APERTURE_RING_LEN = 0.010
APERTURE_CENTER_Z = APERTURE_SUPPORT_Z + APERTURE_SUPPORT_LEN * 0.5

REAR_FIXED_BAND_Z = APERTURE_SUPPORT_Z + APERTURE_SUPPORT_LEN
REAR_FIXED_BAND_LEN = 0.006

FOCUS_SUPPORT_Z = REAR_FIXED_BAND_Z + REAR_FIXED_BAND_LEN
FOCUS_SUPPORT_LEN = 0.036
FOCUS_RING_LEN = 0.034
FOCUS_CENTER_Z = FOCUS_SUPPORT_Z + FOCUS_SUPPORT_LEN * 0.5

FRONT_BARREL_Z = FOCUS_SUPPORT_Z + FOCUS_SUPPORT_LEN
FRONT_BARREL_LEN = 0.014

HOOD_MOUNT_Z = FRONT_BARREL_Z + FRONT_BARREL_LEN
HOOD_MOUNT_LEN = 0.008
HOOD_JOINT_Z = HOOD_MOUNT_Z + HOOD_MOUNT_LEN * 0.5

BARREL_TOTAL_LEN = HOOD_MOUNT_Z + HOOD_MOUNT_LEN

FOCUS_OUTER_RADIUS = 0.0345
FOCUS_INNER_RADIUS = 0.0310
APERTURE_OUTER_RADIUS = 0.0312
APERTURE_INNER_RADIUS = 0.0284
HOOD_MOUNT_OUTER_RADIUS = 0.0324


def _annulus(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    z0: float = 0.0,
    centered: bool = False,
):
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    inner = cq.Workplane("XY").circle(inner_radius).extrude(length + 0.002).translate((0.0, 0.0, -0.001))
    shape = outer.cut(inner)
    if centered:
        return shape.translate((0.0, 0.0, -length * 0.5))
    return shape.translate((0.0, 0.0, z0))


def _radial_box(
    *,
    radius: float,
    radial_depth: float,
    tangential_width: float,
    length: float,
    z_center: float,
    angle_deg: float,
):
    box = cq.Workplane("XY").box(
        radial_depth,
        tangential_width,
        length,
        centered=(True, True, True),
    )
    box = box.translate((radius - radial_depth * 0.5 + 0.0001, 0.0, z_center))
    return box.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)


def _build_rear_band_shape():
    rear_band = _annulus(0.0300, 0.0190, REAR_FIXED_BAND_LEN, z0=REAR_FIXED_BAND_Z)
    distance_window = _radial_box(
        radius=0.0300,
        radial_depth=0.0016,
        tangential_width=0.010,
        length=0.0045,
        z_center=0.0195,
        angle_deg=90.0,
    )
    rear_band = rear_band.cut(distance_window)

    index_boss = _radial_box(
        radius=0.0320,
        radial_depth=0.0022,
        tangential_width=0.0024,
        length=0.006,
        z_center=REAR_FIXED_BAND_Z + REAR_FIXED_BAND_LEN * 0.5,
        angle_deg=90.0,
    )
    return rear_band.union(index_boss)


def _build_front_barrel_shape():
    front_barrel = _annulus(0.0298, 0.0168, FRONT_BARREL_LEN, z0=FRONT_BARREL_Z)
    front_name_ring = _annulus(0.0310, 0.0276, 0.003, z0=0.069)
    return front_barrel.union(front_name_ring)


def _build_ribbed_ring_shape(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    groove_count: int,
    groove_depth: float,
    groove_width: float,
):
    ring = _annulus(outer_radius, inner_radius, length, centered=True)
    for index in range(groove_count):
        angle_deg = 360.0 * index / groove_count
        ring = ring.cut(
            _radial_box(
                radius=outer_radius,
                radial_depth=groove_depth,
                tangential_width=groove_width,
                length=length + 0.002,
                z_center=0.0,
                angle_deg=angle_deg,
            )
        )
    return ring


def _build_focus_ring_shape():
    ring = _build_ribbed_ring_shape(
        outer_radius=FOCUS_OUTER_RADIUS,
        inner_radius=FOCUS_INNER_RADIUS,
        length=FOCUS_RING_LEN,
        groove_count=44,
        groove_depth=0.0022,
        groove_width=0.0013,
    )
    front_land = _annulus(FOCUS_OUTER_RADIUS, FOCUS_OUTER_RADIUS - 0.0012, 0.0024, z0=0.0158, centered=True)
    rear_land = _annulus(FOCUS_OUTER_RADIUS, FOCUS_OUTER_RADIUS - 0.0012, 0.0024, z0=-0.0158, centered=True)
    return ring.union(front_land).union(rear_land)


def _build_aperture_ring_shape():
    ring = _build_ribbed_ring_shape(
        outer_radius=APERTURE_OUTER_RADIUS,
        inner_radius=APERTURE_INNER_RADIUS,
        length=APERTURE_RING_LEN,
        groove_count=30,
        groove_depth=0.0012,
        groove_width=0.0010,
    )
    detent_land = _annulus(APERTURE_OUTER_RADIUS, APERTURE_OUTER_RADIUS - 0.0010, 0.0018, z0=0.0, centered=True)
    return ring.union(detent_land)


def _build_hood_shape():
    hood = _annulus(0.0400, HOOD_MOUNT_OUTER_RADIUS, 0.016, z0=-0.004)
    hood = hood.union(_annulus(0.0415, 0.0348, 0.016, z0=0.012))
    hood = hood.union(_annulus(0.0445, 0.0395, 0.018, z0=0.028))

    for angle_deg in (0.0, 120.0, 240.0):
        twist_tab = _radial_box(
            radius=0.0458,
            radial_depth=0.0060,
            tangential_width=0.0130,
            length=0.005,
            z_center=0.004,
            angle_deg=angle_deg,
        )
        hood = hood.union(twist_tab)

    return hood


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_focus_prime_lens")

    anodized_black = model.material("anodized_black", rgba=(0.11, 0.11, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    satin_black = model.material("satin_black", rgba=(0.14, 0.14, 0.15, 1.0))
    mount_steel = model.material("mount_steel", rgba=(0.70, 0.71, 0.74, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_annulus(0.0310, 0.0210, REAR_FLANGE_LEN, z0=0.0), "mount_shell"),
        material=mount_steel,
        name="mount_shell",
    )
    barrel.visual(
        mesh_from_cadquery(_annulus(0.0284, 0.0196, APERTURE_SUPPORT_LEN, z0=APERTURE_SUPPORT_Z), "aperture_drum"),
        material=anodized_black,
        name="aperture_drum",
    )
    barrel.visual(
        mesh_from_cadquery(_build_rear_band_shape(), "rear_band"),
        material=anodized_black,
        name="rear_band",
    )
    barrel.visual(
        mesh_from_cadquery(_annulus(0.0310, 0.0183, FOCUS_SUPPORT_LEN, z0=FOCUS_SUPPORT_Z), "focus_drum"),
        material=anodized_black,
        name="focus_drum",
    )
    barrel.visual(
        mesh_from_cadquery(_build_front_barrel_shape(), "front_barrel"),
        material=anodized_black,
        name="front_barrel",
    )
    barrel.visual(
        mesh_from_cadquery(_annulus(HOOD_MOUNT_OUTER_RADIUS, 0.0182, HOOD_MOUNT_LEN, z0=HOOD_MOUNT_Z), "hood_mount"),
        material=anodized_black,
        name="hood_mount",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_cadquery(_build_focus_ring_shape(), "focus_ring_mesh"),
        material=rubber_black,
        name="focus_sleeve",
    )

    aperture_ring = model.part("aperture_ring")
    aperture_ring.visual(
        mesh_from_cadquery(_build_aperture_ring_shape(), "aperture_ring_mesh"),
        material=satin_black,
        name="aperture_sleeve",
    )

    hood = model.part("hood")
    hood.visual(
        mesh_from_cadquery(_build_hood_shape(), "hood_mesh"),
        material=satin_black,
        name="hood_collar",
    )

    model.articulation(
        "aperture_rotation",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=aperture_ring,
        origin=Origin(xyz=(0.0, 0.0, APERTURE_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=7.0),
    )
    model.articulation(
        "focus_rotation",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, FOCUS_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )
    model.articulation(
        "hood_twist",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=hood,
        origin=Origin(xyz=(0.0, 0.0, HOOD_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(24.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel = object_model.get_part("barrel")
    focus_ring = object_model.get_part("focus_ring")
    aperture_ring = object_model.get_part("aperture_ring")
    hood = object_model.get_part("hood")

    focus_rotation = object_model.get_articulation("focus_rotation")
    aperture_rotation = object_model.get_articulation("aperture_rotation")
    hood_twist = object_model.get_articulation("hood_twist")

    ctx.check(
        "focus ring is continuous",
        focus_rotation.articulation_type == ArticulationType.CONTINUOUS,
        details=str(focus_rotation.articulation_type),
    )
    ctx.check(
        "aperture ring is continuous",
        aperture_rotation.articulation_type == ArticulationType.CONTINUOUS,
        details=str(aperture_rotation.articulation_type),
    )
    hood_limits = hood_twist.motion_limits
    ctx.allow_overlap(
        focus_ring,
        barrel,
        elem_a="focus_sleeve",
        elem_b="focus_drum",
        reason="The focus ring is intentionally represented as a concentric sleeve riding on the lens barrel drum.",
    )
    ctx.allow_overlap(
        aperture_ring,
        barrel,
        elem_a="aperture_sleeve",
        elem_b="aperture_drum",
        reason="The aperture ring is intentionally represented as a nested sleeve around the rear barrel drum.",
    )
    ctx.allow_overlap(
        hood,
        barrel,
        elem_a="hood_collar",
        elem_b="hood_mount",
        reason="The removable hood is intentionally represented as a bayonet collar nested over the front mounting seat.",
    )

    ctx.check(
        "hood twist stays in a short bayonet range",
        hood_limits is not None
        and hood_limits.lower is not None
        and hood_limits.upper is not None
        and 0.15 <= hood_limits.upper <= 0.60
        and abs(hood_limits.lower) <= 1e-9,
        details=str(hood_limits),
    )

    ctx.expect_origin_distance(
        focus_ring,
        barrel,
        axes="xy",
        max_dist=1e-6,
        name="focus ring stays coaxial with barrel",
    )
    ctx.expect_origin_distance(
        aperture_ring,
        barrel,
        axes="xy",
        max_dist=1e-6,
        name="aperture ring stays coaxial with barrel",
    )
    ctx.expect_origin_distance(
        hood,
        barrel,
        axes="xy",
        max_dist=1e-6,
        name="hood stays coaxial with barrel",
    )

    ctx.expect_within(
        barrel,
        focus_ring,
        axes="xy",
        inner_elem="focus_drum",
        outer_elem="focus_sleeve",
        name="focus sleeve wraps around the fixed barrel drum",
    )
    ctx.expect_overlap(
        focus_ring,
        barrel,
        axes="z",
        elem_a="focus_sleeve",
        elem_b="focus_drum",
        min_overlap=0.033,
        name="focus ring remains over its support drum",
    )
    ctx.expect_within(
        barrel,
        aperture_ring,
        axes="xy",
        inner_elem="aperture_drum",
        outer_elem="aperture_sleeve",
        name="aperture sleeve wraps around the rear barrel drum",
    )
    ctx.expect_overlap(
        aperture_ring,
        barrel,
        axes="z",
        elem_a="aperture_sleeve",
        elem_b="aperture_drum",
        min_overlap=0.009,
        name="aperture ring remains seated near the mount",
    )

    ctx.expect_within(
        barrel,
        hood,
        axes="xy",
        inner_elem="hood_mount",
        outer_elem="hood_collar",
        name="hood collar stays centered around the front bayonet seat",
    )
    ctx.expect_overlap(
        hood,
        barrel,
        axes="z",
        elem_a="hood_collar",
        elem_b="hood_mount",
        min_overlap=0.006,
        name="hood collar retains insertion on the bayonet seat",
    )

    with ctx.pose({hood_twist: hood_limits.upper}):
        ctx.expect_within(
            barrel,
            hood,
            axes="xy",
            inner_elem="hood_mount",
            outer_elem="hood_collar",
            name="twisted hood stays centered on the front bayonet seat",
        )
        ctx.expect_overlap(
            hood,
            barrel,
            axes="z",
            elem_a="hood_collar",
            elem_b="hood_mount",
            min_overlap=0.006,
            name="twisted hood still retains bayonet engagement",
        )

    return ctx.report()


object_model = build_object_model()
