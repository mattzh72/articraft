from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(geometry: MeshGeometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float,
    radial_segments: int = 56,
) -> MeshGeometry:
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.002,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)


def _helix_points(
    *,
    radius: float,
    z_start: float,
    turns: float,
    pitch: float,
    phase: float = 0.0,
    samples_per_turn: int = 44,
) -> list[tuple[float, float, float]]:
    steps = max(12, int(samples_per_turn * turns))
    points: list[tuple[float, float, float]] = []
    for step in range(steps + 1):
        t = turns * step / steps
        angle = phase + (math.tau * t)
        points.append(
            (
                radius * math.cos(angle),
                radius * math.sin(angle),
                z_start + (pitch * t),
            )
        )
    return points


def _thread_bundle(
    *,
    radius: float,
    z_start: float,
    turns: float,
    pitch: float,
    starts: int,
    bead_radius: float,
    radial_segments: int = 14,
) -> MeshGeometry:
    bundle = MeshGeometry()
    for start_index in range(starts):
        phase = start_index * math.tau / starts
        bundle.merge(
            wire_from_points(
                _helix_points(
                    radius=radius,
                    z_start=z_start,
                    turns=turns,
                    pitch=pitch,
                    phase=phase,
                ),
                radius=bead_radius,
                radial_segments=radial_segments,
                cap_ends=True,
                corner_mode="miter",
            )
        )
    return bundle


def _build_bottle_shell_mesh() -> MeshGeometry:
    outer_profile = [
        (0.0310, 0.000),
        (0.0365, 0.008),
        (0.0400, 0.020),
        (0.0415, 0.050),
        (0.0435, 0.056),
        (0.0435, 0.062),
        (0.0410, 0.092),
        (0.0395, 0.138),
        (0.0405, 0.164),
        (0.0405, 0.174),
        (0.0370, 0.184),
        (0.0320, 0.193),
        (0.0245, 0.199),
        (0.0210, 0.206),
        (0.0200, 0.214),
        (0.0195, 0.224),
    ]
    inner_profile = [
        (0.0000, 0.009),
        (0.0280, 0.011),
        (0.0360, 0.024),
        (0.0375, 0.058),
        (0.0360, 0.092),
        (0.0340, 0.138),
        (0.0325, 0.174),
        (0.0290, 0.188),
        (0.0200, 0.198),
        (0.0160, 0.206),
        (0.0150, 0.224),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=84,
        start_cap="flat",
        end_cap="flat",
    )


def _build_cap_shell_mesh() -> MeshGeometry:
    outer_shell = CylinderGeometry(radius=0.0275, height=0.026, radial_segments=64).translate(
        0.0,
        0.0,
        0.013,
    )
    inner_cavity = CylinderGeometry(radius=0.0242, height=0.0222, radial_segments=64).translate(
        0.0,
        0.0,
        0.0111,
    )
    shell = boolean_difference(outer_shell, inner_cavity)
    shell.merge(
        CylinderGeometry(radius=0.0248, height=0.0032, radial_segments=56).translate(
            0.0,
            0.0,
            0.0244,
        )
    )
    shell.merge(
        _ring_band(
            outer_radius=0.0278,
            inner_radius=0.0248,
            height=0.004,
            z_center=0.0205,
            radial_segments=56,
        )
    )
    return shell


def _guard_ring_meshes() -> tuple[MeshGeometry, MeshGeometry]:
    lower_ring = _ring_band(
        outer_radius=0.0530,
        inner_radius=0.0428,
        height=0.014,
        z_center=0.057,
        radial_segments=56,
    )
    upper_ring = _ring_band(
        outer_radius=0.0510,
        inner_radius=0.0398,
        height=0.012,
        z_center=0.166,
        radial_segments=56,
    )
    return lower_ring, upper_ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_screwcap_bottle", assets=ASSETS)

    body_polymer = model.material("body_polymer", rgba=(0.30, 0.35, 0.26, 1.0))
    cap_polymer = model.material("cap_polymer", rgba=(0.12, 0.13, 0.14, 1.0))
    cage_finish = model.material("cage_finish", rgba=(0.24, 0.25, 0.27, 1.0))
    fastener_finish = model.material("fastener_finish", rgba=(0.50, 0.52, 0.56, 1.0))
    seal_finish = model.material("seal_finish", rgba=(0.82, 0.83, 0.84, 1.0))

    bottle = model.part("bottle_body")
    bottle_shell_mesh = _build_bottle_shell_mesh()
    bottle.visual(
        _save_mesh(bottle_shell_mesh, "bottle_shell.obj"),
        material=body_polymer,
        name="body_shell",
    )
    bottle.visual(
        _save_mesh(
            _ring_band(
                outer_radius=0.0248,
                inner_radius=0.0202,
                height=0.0044,
                z_center=0.200,
                radial_segments=48,
            ),
            "neck_support_ring.obj",
        ),
        material=body_polymer,
        name="neck_support_ring",
    )
    for start_index, base_angle in enumerate((0.0, math.tau / 3.0, 2.0 * math.tau / 3.0)):
        for segment_index, (angle_offset, z_center) in enumerate(((0.18, 0.2054), (0.48, 0.2094))):
            angle = base_angle + angle_offset
            bottle.visual(
                Box((0.0030, 0.0110, 0.0032)),
                origin=Origin(
                    xyz=(0.0213 * math.cos(angle), 0.0213 * math.sin(angle), z_center),
                    rpy=(0.0, 0.0, angle),
                ),
                material=body_polymer,
                name=f"neck_thread_{start_index}_{segment_index}",
            )

    lower_guard_mesh, upper_guard_mesh = _guard_ring_meshes()
    bottle.visual(
        _save_mesh(lower_guard_mesh, "lower_guard_ring.obj"),
        material=cage_finish,
        name="lower_guard_ring",
    )
    bottle.visual(
        _save_mesh(upper_guard_mesh, "upper_guard_ring.obj"),
        material=cage_finish,
        name="upper_guard_ring",
    )

    rail_radius = 0.0470
    rail_z = 0.1115
    for name, angle in (
        ("east", 0.0),
        ("north", math.pi / 2.0),
        ("west", math.pi),
        ("south", 3.0 * math.pi / 2.0),
    ):
        bottle.visual(
            Box((0.008, 0.014, 0.103)),
            origin=Origin(
                xyz=(rail_radius * math.cos(angle), rail_radius * math.sin(angle), rail_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=cage_finish,
            name=f"guard_rail_{name}",
        )

    for name, angle in (
        ("east", 0.0),
        ("north", math.pi / 2.0),
        ("west", math.pi),
        ("south", 3.0 * math.pi / 2.0),
    ):
        boss_radius = 0.0480
        fastener_z = 0.166
        bottle.visual(
            Box((0.008, 0.014, 0.010)),
            origin=Origin(
                xyz=(boss_radius * math.cos(angle), boss_radius * math.sin(angle), fastener_z),
                rpy=(0.0, 0.0, angle),
            ),
            material=cage_finish,
            name=f"fastener_boss_{name}",
        )
        bottle.visual(
            Cylinder(radius=0.0022, length=0.006),
            origin=Origin(
                xyz=(0.0526 * math.cos(angle), 0.0526 * math.sin(angle), fastener_z),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=fastener_finish,
            name=f"fastener_shaft_{name}",
        )
        bottle.visual(
            Cylinder(radius=0.0046, length=0.002),
            origin=Origin(
                xyz=(0.0566 * math.cos(angle), 0.0566 * math.sin(angle), fastener_z),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=fastener_finish,
            name=f"fastener_head_{name}",
        )

    bottle.visual(
        Box((0.006, 0.032, 0.086)),
        origin=Origin(xyz=(0.0385, 0.0, 0.116)),
        material=body_polymer,
        name="grip_pad_east",
    )
    bottle.visual(
        Box((0.006, 0.032, 0.086)),
        origin=Origin(xyz=(-0.0385, 0.0, 0.116)),
        material=body_polymer,
        name="grip_pad_west",
    )
    bottle.inertial = Inertial.from_geometry(
        Box((0.110, 0.110, 0.228)),
        mass=0.48,
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
    )

    cap = model.part("screw_cap")
    cap.visual(
        _save_mesh(_build_cap_shell_mesh(), "cap_shell.obj"),
        material=cap_polymer,
        name="cap_skirt",
    )
    cap.visual(
        Cylinder(radius=0.0148, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
        material=seal_finish,
        name="seal_ring",
    )
    for start_index, base_angle in enumerate((0.0, math.tau / 3.0, 2.0 * math.tau / 3.0)):
        for segment_index, (angle_offset, z_center) in enumerate(((0.82, 0.0038), (1.12, 0.0078))):
            angle = base_angle + angle_offset
            cap.visual(
                Box((0.0036, 0.0110, 0.0032)),
                origin=Origin(
                    xyz=(0.0237 * math.cos(angle), 0.0237 * math.sin(angle), z_center),
                    rpy=(0.0, 0.0, angle),
                ),
                material=cap_polymer,
                name=f"cap_thread_{start_index}_{segment_index}",
            )
    for index in range(12):
        angle = index * math.tau / 12.0
        cap.visual(
            Box((0.004, 0.007, 0.015)),
            origin=Origin(
                xyz=(0.0290 * math.cos(angle), 0.0290 * math.sin(angle), 0.0085),
                rpy=(0.0, 0.0, angle),
            ),
            material=cap_polymer,
            name=f"grip_rib_{index}",
        )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.024),
        mass=0.07,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    model.articulation(
        "body_to_cap",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.2022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bottle = object_model.get_part("bottle_body")
    cap = object_model.get_part("screw_cap")
    cap_joint = object_model.get_articulation("body_to_cap")

    body_shell = bottle.get_visual("body_shell")
    neck_thread = bottle.get_visual("neck_thread_0_1")
    neck_support_ring = bottle.get_visual("neck_support_ring")
    upper_guard_ring = bottle.get_visual("upper_guard_ring")
    lower_guard_ring = bottle.get_visual("lower_guard_ring")
    fastener_head_north = bottle.get_visual("fastener_head_north")
    fastener_head_east = bottle.get_visual("fastener_head_east")

    cap_skirt = cap.get_visual("cap_skirt")
    cap_thread = cap.get_visual("cap_thread_0_0")
    seal_ring = cap.get_visual("seal_ring")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_origin_distance(cap, bottle, axes="xy", max_dist=0.001)
    ctx.expect_within(bottle, cap, axes="xy", inner_elem=neck_thread, outer_elem=cap_skirt, margin=0.0)
    ctx.expect_overlap(cap, bottle, axes="xy", elem_a=cap_thread, elem_b=neck_thread, min_overlap=0.003)
    ctx.expect_gap(
        cap,
        bottle,
        axis="z",
        positive_elem=cap_skirt,
        negative_elem=neck_support_ring,
        min_gap=0.0,
        max_gap=0.002,
        max_penetration=0.0,
    )

    bottle_aabb = ctx.part_world_aabb(bottle)
    cap_aabb = ctx.part_world_aabb(cap)
    upper_guard_aabb = ctx.part_element_world_aabb(bottle, elem=upper_guard_ring)
    lower_guard_aabb = ctx.part_element_world_aabb(bottle, elem=lower_guard_ring)
    fastener_north_aabb = ctx.part_element_world_aabb(bottle, elem=fastener_head_north)
    fastener_east_aabb = ctx.part_element_world_aabb(bottle, elem=fastener_head_east)
    cap_rest = ctx.part_world_position(cap)

    assert bottle_aabb is not None
    assert cap_aabb is not None
    assert upper_guard_aabb is not None
    assert lower_guard_aabb is not None
    assert fastener_north_aabb is not None
    assert fastener_east_aabb is not None
    assert cap_rest is not None

    bottle_dx = bottle_aabb[1][0] - bottle_aabb[0][0]
    bottle_dz = bottle_aabb[1][2] - bottle_aabb[0][2]
    cap_dx = cap_aabb[1][0] - cap_aabb[0][0]
    cap_dz = cap_aabb[1][2] - cap_aabb[0][2]
    ctx.check(
        "utility_bottle_proportions",
        0.085 <= bottle_dx <= 0.120 and 0.220 <= bottle_dz <= 0.235,
        f"unexpected bottle dimensions dx={bottle_dx:.4f}, dz={bottle_dz:.4f}",
    )
    ctx.check(
        "cap_is_substantial",
        0.050 <= cap_dx <= 0.065 and 0.022 <= cap_dz <= 0.028,
        f"unexpected cap dimensions dx={cap_dx:.4f}, dz={cap_dz:.4f}",
    )
    ctx.check(
        "guard_rings_frame_body",
        lower_guard_aabb[0][2] < 0.065 and upper_guard_aabb[0][2] > 0.158,
        "guard rings are not bracketing the lower body and shoulder as intended",
    )
    ctx.check(
        "exposed_fasteners_present",
        fastener_north_aabb[1][2] > 0.165 and fastener_east_aabb[1][0] > 0.054,
        "expected exposed utility fasteners on the upper reinforcement ring",
    )
    ctx.check(
        "cap_joint_axis_is_vertical",
        getattr(cap_joint, "axis", None) == (0.0, 0.0, 1.0),
        f"unexpected cap joint axis {getattr(cap_joint, 'axis', None)}",
    )

    with ctx.pose({cap_joint: math.pi * 0.75}):
        cap_rotated = ctx.part_world_position(cap)
        assert cap_rotated is not None
        ctx.expect_origin_distance(cap, bottle, axes="xy", max_dist=0.001)
        ctx.expect_gap(
            cap,
            bottle,
            axis="z",
            positive_elem=cap_skirt,
            negative_elem=neck_support_ring,
            min_gap=0.0,
            max_gap=0.002,
            max_penetration=0.0,
        )
        ctx.check(
            "cap_rotation_is_coaxial",
            abs(cap_rotated[0] - cap_rest[0]) <= 1e-6
            and abs(cap_rotated[1] - cap_rest[1]) <= 1e-6
            and abs(cap_rotated[2] - cap_rest[2]) <= 1e-6,
            f"cap origin drifted from {cap_rest} to {cap_rotated}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
