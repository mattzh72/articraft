from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _grip_outer_profile(
    *,
    length: float,
    outer_radius: float,
    groove_depth: float,
    groove_count: int,
) -> list[tuple[float, float]]:
    z0 = -0.5 * length
    edge = min(0.0018, 0.18 * length)
    usable = max(length - (2.0 * edge), 0.001)
    pitch = usable / groove_count
    profile = [
        (outer_radius - 0.0005, z0),
        (outer_radius, z0 + edge),
    ]
    for index in range(groove_count):
        base = z0 + edge + (index * pitch)
        profile.extend(
            [
                (outer_radius, base + 0.22 * pitch),
                (outer_radius - groove_depth, base + 0.50 * pitch),
                (outer_radius, base + 0.78 * pitch),
            ]
        )
    profile.append((outer_radius - 0.0005, z0 + length))
    return profile


def _lathed_shell(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 80,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_outer_barrel_mesh() -> MeshGeometry:
    return _lathed_shell(
        [
            (0.0340, 0.000),
            (0.0365, 0.006),
            (0.0405, 0.014),
            (0.0405, 0.044),
            (0.0428, 0.044),
            (0.0428, 0.076),
            (0.0395, 0.076),
            (0.0395, 0.090),
            (0.0388, 0.090),
            (0.0388, 0.106),
        ],
        [
            (0.0180, 0.000),
            (0.0292, 0.010),
            (0.0354, 0.014),
            (0.0354, 0.094),
            (0.0360, 0.106),
        ],
    )


def _build_zoom_ring_mesh() -> MeshGeometry:
    length = 0.032
    return _lathed_shell(
        _grip_outer_profile(
            length=length,
            outer_radius=0.0468,
            groove_depth=0.0015,
            groove_count=9,
        ),
        [
            (0.0452, -0.5 * length),
            (0.0452, 0.5 * length),
        ],
    )


def _radial_pattern(base_geometry: MeshGeometry, *, count: int = 4) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geometry.copy().rotate_z(index * math.tau / count))
    return patterned


def _build_zoom_ring_bushings_mesh() -> MeshGeometry:
    parent_radius = 0.0428
    bushing_radius = 0.00135
    bushing = CylinderGeometry(radius=bushing_radius, height=0.020, radial_segments=16).translate(
        parent_radius + bushing_radius,
        0.0,
        0.0,
    )
    return _radial_pattern(bushing)


def _build_inner_barrel_shell_mesh() -> MeshGeometry:
    shell = _lathed_shell(
        [
            (0.0334, 0.000),
            (0.0334, 0.026),
            (0.0356, 0.026),
            (0.0356, 0.050),
            (0.0368, 0.050),
            (0.0368, 0.058),
        ],
        [
            (0.0290, 0.000),
            (0.0288, 0.040),
            (0.0278, 0.040),
            (0.0280, 0.058),
        ],
    )
    shell.merge(
        CylinderGeometry(radius=0.020, height=0.096, radial_segments=28).translate(0.0, 0.0, -0.002)
    )

    front_spoke = BoxGeometry((0.0092, 0.0032, 0.012)).translate(0.0244, 0.0, 0.022)
    rear_spoke = BoxGeometry((0.0128, 0.0032, 0.012)).translate(0.0264, 0.0, -0.018)
    shell.merge(_radial_pattern(front_spoke))
    shell.merge(_radial_pattern(rear_spoke))
    return shell


def _build_inner_barrel_guide_bushings_mesh() -> MeshGeometry:
    bore_radius = 0.0360
    bushing_radius = 0.00160
    bushing = CylinderGeometry(radius=bushing_radius, height=0.038, radial_segments=16).translate(
        bore_radius - bushing_radius,
        0.0,
        -0.018,
    )
    return _radial_pattern(bushing)


def _build_focus_ring_mesh() -> MeshGeometry:
    length = 0.022
    return _lathed_shell(
        _grip_outer_profile(
            length=length,
            outer_radius=0.0408,
            groove_depth=0.0012,
            groove_count=7,
        ),
        [
            (0.0387, -0.5 * length),
            (0.0387, 0.5 * length),
        ],
    )


def _build_focus_ring_bushings_mesh() -> MeshGeometry:
    parent_radius = 0.0368
    bushing_radius = 0.00105
    bushing = CylinderGeometry(radius=bushing_radius, height=0.015, radial_segments=14).translate(
        parent_radius + bushing_radius,
        0.0,
        0.0,
    )
    return _radial_pattern(bushing)


def _build_bayonet_mount_mesh() -> MeshGeometry:
    flange = _lathed_shell(
        [
            (0.0290, -0.0045),
            (0.0312, -0.0030),
            (0.0312, 0.0000),
        ],
        [
            (0.0165, -0.0045),
            (0.0165, 0.0000),
        ],
    )

    lug_template = BoxGeometry((0.0058, 0.0105, 0.0018)).translate(0.0302, 0.0, -0.0012)
    for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        flange.merge(lug_template.copy().rotate_z(angle))

    return flange


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standard_zoom_lens")

    barrel_black = model.material("barrel_black", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    anodized_metal = model.material("anodized_metal", rgba=(0.62, 0.64, 0.67, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.18, 0.26, 0.34, 0.60))

    outer_barrel = model.part("outer_barrel")
    outer_barrel.visual(
        _save_mesh("outer_barrel_shell", _build_outer_barrel_mesh()),
        material=barrel_black,
        name="outer_barrel_shell",
    )
    outer_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.043, length=0.106),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.053)),
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        _save_mesh("zoom_ring_shell", _build_zoom_ring_mesh()),
        material=rubber_black,
        name="zoom_ring_shell",
    )
    zoom_ring.visual(
        _save_mesh("zoom_ring_bushings", _build_zoom_ring_bushings_mesh()),
        material=barrel_black,
        name="zoom_ring_bushings",
    )
    zoom_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.047, length=0.032),
        mass=0.11,
        origin=Origin(),
    )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(
        _save_mesh("inner_barrel_shell", _build_inner_barrel_shell_mesh()),
        material=barrel_black,
        name="inner_barrel_shell",
    )
    inner_barrel_pad_specs = (
        ("focus_guide_pad_px", (0.0022, 0.0060, 0.014), Origin(xyz=(0.0354, 0.0, 0.037))),
        ("focus_guide_pad_nx", (0.0022, 0.0060, 0.014), Origin(xyz=(-0.0354, 0.0, 0.037))),
        ("focus_guide_pad_py", (0.0060, 0.0022, 0.014), Origin(xyz=(0.0, 0.0354, 0.037))),
        ("focus_guide_pad_ny", (0.0060, 0.0022, 0.014), Origin(xyz=(0.0, -0.0354, 0.037))),
    )
    for name, size, origin in inner_barrel_pad_specs:
        inner_barrel.visual(
            Box(size),
            origin=origin,
            material=anodized_metal,
            name=name,
        )
    inner_barrel.visual(
        _save_mesh("inner_barrel_guide_bushings", _build_inner_barrel_guide_bushings_mesh()),
        material=anodized_metal,
        name="inner_barrel_guide_bushings",
    )
    inner_barrel.visual(
        Cylinder(radius=0.0280, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=glass_dark,
        name="front_element",
    )
    inner_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.090),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _save_mesh("focus_ring_shell", _build_focus_ring_mesh()),
        material=rubber_black,
        name="focus_ring_shell",
    )
    focus_ring_pad_specs = (
        ("drive_pad_px", (0.0022, 0.0060, 0.014), Origin(xyz=(0.0376, 0.0, 0.0))),
        ("drive_pad_nx", (0.0022, 0.0060, 0.014), Origin(xyz=(-0.0376, 0.0, 0.0))),
        ("drive_pad_py", (0.0060, 0.0022, 0.014), Origin(xyz=(0.0, 0.0376, 0.0))),
        ("drive_pad_ny", (0.0060, 0.0022, 0.014), Origin(xyz=(0.0, -0.0376, 0.0))),
    )
    for name, size, origin in focus_ring_pad_specs:
        focus_ring.visual(
            Box(size),
            origin=origin,
            material=barrel_black,
            name=name,
        )
    focus_ring.visual(
        _save_mesh("focus_ring_bushings", _build_focus_ring_bushings_mesh()),
        material=barrel_black,
        name="focus_ring_bushings",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.022),
        mass=0.06,
        origin=Origin(),
    )

    bayonet_mount = model.part("bayonet_mount")
    bayonet_mount.visual(
        _save_mesh("bayonet_mount_plate", _build_bayonet_mount_mesh()),
        material=anodized_metal,
        name="bayonet_mount_plate",
    )
    bayonet_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.005),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
    )

    model.articulation(
        "outer_barrel_to_zoom_ring",
        ArticulationType.REVOLUTE,
        parent=outer_barrel,
        child=zoom_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "outer_barrel_to_inner_barrel",
        ArticulationType.PRISMATIC,
        parent=outer_barrel,
        child=inner_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.10,
            lower=0.0,
            upper=0.024,
        ),
    )
    model.articulation(
        "inner_barrel_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=inner_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.5,
            lower=-1.10,
            upper=1.10,
        ),
    )
    model.articulation(
        "outer_barrel_to_bayonet_mount",
        ArticulationType.FIXED,
        parent=outer_barrel,
        child=bayonet_mount,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_barrel = object_model.get_part("outer_barrel")
    zoom_ring = object_model.get_part("zoom_ring")
    inner_barrel = object_model.get_part("inner_barrel")
    focus_ring = object_model.get_part("focus_ring")
    bayonet_mount = object_model.get_part("bayonet_mount")

    zoom_joint = object_model.get_articulation("outer_barrel_to_zoom_ring")
    slide_joint = object_model.get_articulation("outer_barrel_to_inner_barrel")
    focus_joint = object_model.get_articulation("inner_barrel_to_focus_ring")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "zoom ring revolves on optical axis",
        zoom_joint.articulation_type == ArticulationType.REVOLUTE and zoom_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={zoom_joint.articulation_type}, axis={zoom_joint.axis}",
    )
    ctx.check(
        "inner barrel slides on optical axis",
        slide_joint.articulation_type == ArticulationType.PRISMATIC and slide_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={slide_joint.articulation_type}, axis={slide_joint.axis}",
    )
    ctx.check(
        "focus ring revolves on optical axis",
        focus_joint.articulation_type == ArticulationType.REVOLUTE and focus_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={focus_joint.articulation_type}, axis={focus_joint.axis}",
    )

    ctx.expect_contact(zoom_ring, outer_barrel, name="zoom ring is supported by the outer barrel")
    ctx.expect_contact(inner_barrel, outer_barrel, name="inner barrel rides in the outer barrel sleeve")
    ctx.expect_contact(focus_ring, inner_barrel, name="focus ring is supported by the inner barrel")
    ctx.expect_contact(bayonet_mount, outer_barrel, name="bayonet mount seats against the rear barrel")

    ctx.expect_within(
        inner_barrel,
        outer_barrel,
        axes="xy",
        margin=0.004,
        name="inner barrel stays centered within the outer barrel",
    )
    ctx.expect_overlap(
        inner_barrel,
        outer_barrel,
        axes="z",
        min_overlap=0.040,
        name="collapsed inner barrel retains deep insertion",
    )
    ctx.expect_origin_gap(
        focus_ring,
        zoom_ring,
        axis="z",
        min_gap=0.050,
        name="focus ring sits forward of the zoom ring",
    )

    slide_upper = slide_joint.motion_limits.upper or 0.0
    zoom_upper = zoom_joint.motion_limits.upper or 0.0
    rest_inner_pos = ctx.part_world_position(inner_barrel)
    rest_focus_pos = ctx.part_world_position(focus_ring)
    with ctx.pose({slide_joint: slide_upper, zoom_joint: zoom_upper}):
        ctx.expect_within(
            inner_barrel,
            outer_barrel,
            axes="xy",
            margin=0.004,
            name="extended inner barrel stays centered within the outer barrel",
        )
        ctx.expect_overlap(
            inner_barrel,
            outer_barrel,
            axes="z",
            min_overlap=0.018,
            name="extended inner barrel still retains insertion",
        )
        extended_inner_pos = ctx.part_world_position(inner_barrel)
        extended_focus_pos = ctx.part_world_position(focus_ring)

    ctx.check(
        "zoom action extends the front barrel forward",
        rest_inner_pos is not None
        and extended_inner_pos is not None
        and extended_inner_pos[2] > rest_inner_pos[2] + 0.020,
        details=f"rest={rest_inner_pos}, extended={extended_inner_pos}",
    )
    ctx.check(
        "focus ring rides forward with the extending inner barrel",
        rest_focus_pos is not None
        and extended_focus_pos is not None
        and extended_inner_pos is not None
        and rest_inner_pos is not None
        and abs((extended_focus_pos[2] - rest_focus_pos[2]) - (extended_inner_pos[2] - rest_inner_pos[2])) < 0.003,
        details=(
            f"focus_rest={rest_focus_pos}, focus_extended={extended_focus_pos}, "
            f"inner_rest={rest_inner_pos}, inner_extended={extended_inner_pos}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
