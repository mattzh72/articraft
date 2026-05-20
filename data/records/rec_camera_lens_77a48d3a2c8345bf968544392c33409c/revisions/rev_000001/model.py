from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


LENS_LENGTH = 0.092
ZOOM_RING_LENGTH = 0.022
FOCUS_RING_LENGTH = 0.015


def _x_cylinder(radius: float, length: float, *, x_center: float, y: float, z: float):
    geom = CylinderGeometry(radius, length)
    geom.rotate_y(math.pi * 0.5)
    geom.translate(x_center, y, z)
    return geom


def _polar_box(
    radial_depth: float,
    tangential_width: float,
    height: float,
    *,
    radius: float,
    angle_deg: float,
    z_center: float,
):
    geom = BoxGeometry((radial_depth, tangential_width, height))
    geom.translate(radius + (radial_depth * 0.5), 0.0, z_center)
    geom.rotate_z(math.radians(angle_deg))
    return geom


def _ribbed_profile(
    *,
    sleeve_radius: float,
    rib_radius: float,
    length: float,
    rib_count: int,
    rib_width: float,
    end_margin: float,
):
    profile = [(sleeve_radius, 0.0)]
    if rib_count == 1:
        starts = [(length - rib_width) * 0.5]
    else:
        usable = max(length - (2.0 * end_margin) - rib_width, 0.0)
        step = usable / (rib_count - 1)
        starts = [end_margin + (step * idx) for idx in range(rib_count)]
    for start in starts:
        if start > profile[-1][1]:
            profile.append((sleeve_radius, start))
        profile.append((rib_radius, start))
        profile.append((rib_radius, start + rib_width))
        profile.append((sleeve_radius, start + rib_width))
    if profile[-1][1] < length:
        profile.append((sleeve_radius, length))
    return profile


def _build_barrel_shell():
    outer_profile = [
        (0.0318, 0.0000),
        (0.0318, 0.0035),
        (0.0330, 0.0035),
        (0.0330, 0.0260),
        (0.0310, 0.0260),
        (0.0310, 0.0540),
        (0.0292, 0.0540),
        (0.0292, 0.0740),
        (0.0335, 0.0740),
        (0.0335, 0.0890),
        (0.0352, 0.0890),
        (0.0352, LENS_LENGTH),
    ]
    inner_profile = [
        (0.0230, 0.0000),
        (0.0230, 0.0035),
        (0.0270, 0.0035),
        (0.0270, 0.0740),
        (0.0288, 0.0740),
        (0.0288, LENS_LENGTH),
    ]
    shell = LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=80)

    for angle_deg in (0.0, 120.0, 240.0):
        shell.merge(
            _polar_box(
                0.0032,
                0.0150,
                0.0042,
                radius=0.0323,
                angle_deg=angle_deg,
                z_center=0.0815,
            )
        )

    hinge_block = BoxGeometry((0.016, 0.0038, 0.006))
    hinge_block.translate(0.0, 0.0336, 0.084)
    shell.merge(hinge_block)
    shell.merge(_x_cylinder(0.00175, 0.0030, x_center=-0.0050, y=0.0350, z=0.084))
    shell.merge(_x_cylinder(0.00175, 0.0030, x_center=0.0050, y=0.0350, z=0.084))
    return shell


def _build_mount_ring():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0318, 0.0000),
            (0.0318, 0.0048),
            (0.0306, 0.0048),
            (0.0306, 0.0058),
        ],
        [
            (0.0228, 0.0000),
            (0.0228, 0.0048),
            (0.0238, 0.0048),
            (0.0238, 0.0058),
        ],
        segments=72,
    )


def _build_grip_ring(
    *,
    contact_radius: float,
    outer_radius: float,
    length: float,
    bar_count: int,
    tangential_width: float,
):
    frame_inner_radius = contact_radius + 0.0009
    tube_radius = (outer_radius - frame_inner_radius) * 0.5
    torus_major = frame_inner_radius + tube_radius
    bar_length = length - (tube_radius * 1.6)
    bar_center_radius = frame_inner_radius + tube_radius

    ring = TorusGeometry(torus_major, tube_radius, radial_segments=18, tubular_segments=56)
    front_ridge = ring.copy().translate(0.0, 0.0, -((length * 0.5) - tube_radius))
    rear_ridge = ring.copy().translate(0.0, 0.0, (length * 0.5) - tube_radius)

    frame = front_ridge
    frame.merge(rear_ridge)

    for index in range(bar_count):
        angle = (2.0 * math.pi * index) / bar_count
        bar = BoxGeometry((outer_radius - frame_inner_radius, tangential_width, bar_length))
        bar.translate(bar_center_radius, 0.0, 0.0)
        bar.rotate_z(angle)
        frame.merge(bar)

    pad_depth = (frame_inner_radius - contact_radius) + 0.0003
    pad_center_radius = contact_radius + (pad_depth * 0.5)
    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0
        pad = BoxGeometry((pad_depth, tangential_width * 1.2, length * 0.62))
        pad.translate(pad_center_radius, 0.0, 0.0)
        pad.rotate_z(angle)
        frame.merge(pad)

    return frame


def _build_zoom_ring():
    return _build_grip_ring(
        contact_radius=0.0310,
        outer_radius=0.0361,
        length=ZOOM_RING_LENGTH,
        bar_count=24,
        tangential_width=0.0030,
    )


def _build_focus_ring():
    return _build_grip_ring(
        contact_radius=0.0292,
        outer_radius=0.0346,
        length=FOCUS_RING_LENGTH,
        bar_count=20,
        tangential_width=0.0027,
    )


def _build_latch_flap():
    panel_width = 0.014
    panel_thickness = 0.0022
    panel_height = 0.009
    hinge_radius = 0.0015

    flap = BoxGeometry((panel_width, panel_thickness, panel_height))
    flap.translate(0.0, -0.0004, -(hinge_radius + (panel_height * 0.5)))
    flap.merge(_x_cylinder(hinge_radius, 0.0066, x_center=0.0, y=0.0, z=0.0))
    thumb_nub = BoxGeometry((0.005, 0.0018, 0.0022))
    thumb_nub.translate(0.0, 0.0009, -0.0102)
    flap.merge(thumb_nub)
    return flap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_zoom_lens")

    body_black = model.material("body_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.64, 0.66, 0.69, 1.0))
    latch_black = model.material("latch_black", rgba=(0.12, 0.12, 0.13, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_build_mount_ring(), "lens_mount_ring"),
        material=mount_metal,
        name="mount_ring",
    )
    barrel.visual(
        mesh_from_geometry(_build_barrel_shell(), "lens_barrel_shell"),
        material=body_black,
        name="barrel_shell",
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        mesh_from_geometry(_build_zoom_ring(), "lens_zoom_ring"),
        material=rubber_black,
        name="zoom_grip",
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        mesh_from_geometry(_build_focus_ring(), "lens_focus_ring"),
        material=rubber_black,
        name="focus_grip",
    )

    latch_flap = model.part("latch_flap")
    latch_flap.visual(
        mesh_from_geometry(_build_latch_flap(), "lens_latch_flap"),
        material=latch_black,
        name="flap_shell",
    )

    model.articulation(
        "zoom_rotation",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=zoom_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )
    model.articulation(
        "focus_rotation",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0635)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=10.0),
    )
    model.articulation(
        "latch_hinge",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=latch_flap,
        origin=Origin(xyz=(0.0, 0.0350, 0.084)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(38.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel = object_model.get_part("barrel")
    zoom_ring = object_model.get_part("zoom_ring")
    focus_ring = object_model.get_part("focus_ring")
    latch_flap = object_model.get_part("latch_flap")

    latch_hinge = object_model.get_articulation("latch_hinge")

    ctx.expect_overlap(
        zoom_ring,
        barrel,
        axes="xy",
        min_overlap=0.060,
        name="zoom ring stays coaxial with the barrel",
    )
    ctx.expect_overlap(
        focus_ring,
        barrel,
        axes="xy",
        min_overlap=0.056,
        name="focus ring stays coaxial with the barrel",
    )
    ctx.expect_gap(
        focus_ring,
        zoom_ring,
        axis="z",
        min_gap=0.004,
        max_gap=0.010,
        name="focus ring sits forward of the zoom ring",
    )
    ctx.expect_origin_distance(
        latch_flap,
        barrel,
        axes="y",
        min_dist=0.034,
        max_dist=0.0365,
        name="latch hinge is mounted on the side of the front bayonet",
    )

    closed_aabb = ctx.part_world_aabb(latch_flap)
    with ctx.pose({latch_hinge: math.radians(32.0)}):
        open_aabb = ctx.part_world_aabb(latch_flap)

    ctx.check(
        "latch flap opens outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][1] > closed_aabb[1][1] + 0.003,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
