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
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _annular_band(
    outer_radius: float,
    inner_radius: float,
    length: float,
    *,
    segments: int = 72,
) -> MeshGeometry:
    half = 0.5 * length
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _helix_points(
    radius: float,
    z_start: float,
    height: float,
    turns: float,
    *,
    phase: float = 0.0,
    samples_per_turn: int = 28,
) -> list[tuple[float, float, float]]:
    sample_count = max(8, int(math.ceil(turns * samples_per_turn)))
    points: list[tuple[float, float, float]] = []
    for index in range(sample_count + 1):
        angle = phase + (turns * math.tau * index / sample_count)
        z_pos = z_start + (height * index / sample_count)
        points.append((radius * math.cos(angle), radius * math.sin(angle), z_pos))
    return points


def _neck_thread_mesh() -> MeshGeometry:
    thread_geometries = []
    for phase in (0.0, math.pi):
        thread_geometries.append(
            wire_from_points(
                _helix_points(
                    radius=0.02135,
                    z_start=0.304,
                    height=0.022,
                    turns=1.35,
                    phase=phase,
                ),
                radius=0.00105,
                radial_segments=12,
                cap_ends=True,
                corner_mode="miter",
            )
        )
    return _merge_geometries(thread_geometries)


def _cap_thread_mesh() -> MeshGeometry:
    thread_geometries = []
    for phase in (0.0, math.pi):
        thread_geometries.append(
            wire_from_points(
                _helix_points(
                    radius=0.0249,
                    z_start=0.010,
                    height=0.022,
                    turns=1.35,
                    phase=phase,
                ),
                radius=0.0017,
                radial_segments=12,
                cap_ends=True,
                corner_mode="miter",
            )
        )
    return _merge_geometries(thread_geometries)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_screwcap_bottle", assets=ASSETS)

    body_polymer = model.material("body_polymer", rgba=(0.23, 0.31, 0.28, 1.0))
    cap_polymer = model.material("cap_polymer", rgba=(0.12, 0.12, 0.13, 1.0))
    guard_paint = model.material("guard_paint", rgba=(0.29, 0.31, 0.34, 1.0))
    fastener_steel = model.material("fastener_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    wear_strip = model.material("wear_strip", rgba=(0.18, 0.19, 0.20, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_shell = LatheGeometry.from_shell_profiles(
        [
            (0.048, 0.000),
            (0.050, 0.010),
            (0.048, 0.030),
            (0.047, 0.120),
            (0.046, 0.205),
            (0.042, 0.248),
            (0.033, 0.279),
            (0.025, 0.292),
            (0.0212, 0.301),
            (0.0212, 0.338),
            (0.0223, 0.344),
        ],
        [
            (0.000, 0.000),
            (0.020, 0.010),
            (0.034, 0.018),
            (0.039, 0.040),
            (0.040, 0.120),
            (0.039, 0.205),
            (0.034, 0.246),
            (0.022, 0.279),
            (0.0172, 0.300),
            (0.0158, 0.338),
            (0.0164, 0.344),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )
    bottle_body.visual(
        _save_mesh("utility_bottle_shell.obj", bottle_shell),
        material=body_polymer,
        name="bottle_shell",
    )
    bottle_body.visual(
        _save_mesh("utility_base_guard.obj", _annular_band(0.051, 0.041, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=wear_strip,
        name="base_guard",
    )
    bottle_body.visual(
        _save_mesh("utility_grip_band_lower.obj", _annular_band(0.0495, 0.0440, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
        material=guard_paint,
    )
    bottle_body.visual(
        _save_mesh("utility_grip_band_upper.obj", _annular_band(0.0490, 0.0435, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.168)),
        material=guard_paint,
    )
    bottle_body.visual(
        _save_mesh("utility_cap_stop_ring.obj", _annular_band(0.0280, 0.0170, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.287)),
        material=guard_paint,
        name="cap_stop_ring",
    )
    bottle_body.visual(
        _save_mesh("utility_neck_finish.obj", _annular_band(0.0206, 0.0158, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.319)),
        material=body_polymer,
        name="neck_finish",
    )
    bottle_body.visual(
        _save_mesh("utility_neck_lip.obj", _annular_band(0.0223, 0.0164, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.341)),
        material=body_polymer,
        name="neck_lip",
    )
    bottle_body.visual(
        _save_mesh("utility_neck_threads.obj", _neck_thread_mesh()),
        material=guard_paint,
        name="neck_threads",
    )
    bottle_body.visual(
        Box((0.012, 0.032, 0.150)),
        origin=Origin(xyz=(0.041, 0.0, 0.145)),
        material=guard_paint,
    )
    bottle_body.visual(
        Box((0.012, 0.032, 0.150)),
        origin=Origin(xyz=(-0.041, 0.0, 0.145)),
        material=guard_paint,
    )
    bottle_body.visual(
        Box((0.060, 0.010, 0.126)),
        origin=Origin(xyz=(0.0, 0.043, 0.152)),
        material=guard_paint,
        name="front_plate",
    )
    bottle_body.visual(
        Box((0.060, 0.010, 0.126)),
        origin=Origin(xyz=(0.0, -0.043, 0.152)),
        material=guard_paint,
        name="rear_plate",
    )
    bottle_body.visual(
        Box((0.048, 0.016, 0.060)),
        origin=Origin(xyz=(0.0, 0.036, 0.246)),
        material=guard_paint,
    )
    bottle_body.visual(
        Box((0.048, 0.016, 0.060)),
        origin=Origin(xyz=(0.0, -0.036, 0.246)),
        material=guard_paint,
    )
    for y_sign in (1.0, -1.0):
        for x_pos in (-0.018, 0.018):
            for z_pos in (0.112, 0.192):
                bottle_body.visual(
                    Cylinder(radius=0.0046, length=0.004),
                    origin=Origin(
                        xyz=(x_pos, y_sign * 0.050, z_pos),
                        rpy=(math.pi / 2.0, 0.0, 0.0),
                    ),
                    material=fastener_steel,
                )
    bottle_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.344),
        mass=0.82,
        origin=Origin(xyz=(0.0, 0.0, 0.172)),
    )

    screw_cap = model.part("screw_cap")
    cap_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0298, 0.000),
            (0.0298, 0.047),
            (0.0290, 0.054),
            (0.0282, 0.058),
        ],
        [
            (0.0265, 0.000),
            (0.0265, 0.051),
            (0.0254, 0.055),
            (0.0000, 0.058),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )
    screw_cap.visual(
        _save_mesh("utility_cap_shell.obj", cap_shell),
        material=cap_polymer,
        name="cap_shell",
    )
    screw_cap.visual(
        _save_mesh("utility_cap_internal_threads.obj", _cap_thread_mesh()),
        material=guard_paint,
        name="cap_internal_threads",
    )
    screw_cap.visual(
        Cylinder(radius=0.0215, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=cap_polymer,
        name="cap_top_plug",
    )
    screw_cap.visual(
        Box((0.028, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=guard_paint,
    )
    screw_cap.visual(
        Box((0.006, 0.028, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=guard_paint,
    )
    for index in range(12):
        angle = index * math.tau / 12.0
        screw_cap.visual(
            Box((0.0046, 0.010, 0.030)),
            origin=Origin(
                xyz=(0.0294 * math.cos(angle), 0.0294 * math.sin(angle), 0.024),
                rpy=(0.0, 0.0, angle),
            ),
            material=cap_polymer,
        )
    screw_cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.031, length=0.058),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
    )

    model.articulation(
        "cap_thread",
        ArticulationType.CONTINUOUS,
        parent=bottle_body,
        child=screw_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.294)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bottle_body = object_model.get_part("bottle_body")
    screw_cap = object_model.get_part("screw_cap")
    cap_thread = object_model.get_articulation("cap_thread")

    bottle_shell = bottle_body.get_visual("bottle_shell")
    front_plate = bottle_body.get_visual("front_plate")
    rear_plate = bottle_body.get_visual("rear_plate")
    cap_stop_ring = bottle_body.get_visual("cap_stop_ring")
    neck_finish = bottle_body.get_visual("neck_finish")
    neck_lip = bottle_body.get_visual("neck_lip")
    neck_threads = bottle_body.get_visual("neck_threads")

    cap_shell = screw_cap.get_visual("cap_shell")
    cap_internal_threads = screw_cap.get_visual("cap_internal_threads")
    cap_top_plug = screw_cap.get_visual("cap_top_plug")

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

    ctx.expect_origin_distance(
        screw_cap,
        bottle_body,
        axes="xy",
        max_dist=0.0005,
        name="cap axis aligned to bottle axis",
    )
    ctx.expect_contact(
        screw_cap,
        bottle_body,
        elem_a=cap_shell,
        elem_b=cap_stop_ring,
        name="cap bears on molded stop ring",
    )
    ctx.expect_gap(
        screw_cap,
        bottle_body,
        axis="z",
        max_gap=0.0008,
        max_penetration=0.0,
        positive_elem=cap_shell,
        negative_elem=cap_stop_ring,
        name="cap lower edge seats cleanly on ring",
    )
    ctx.expect_within(
        bottle_body,
        screw_cap,
        axes="xy",
        margin=0.0,
        inner_elem=neck_finish,
        outer_elem=cap_shell,
        name="neck finish stays within cap skirt",
    )
    ctx.expect_overlap(
        screw_cap,
        bottle_body,
        axes="xy",
        min_overlap=0.040,
        elem_a=cap_shell,
        elem_b=neck_threads,
        name="cap footprint matches thread footprint",
    )
    ctx.expect_overlap(
        screw_cap,
        bottle_body,
        axes="xy",
        min_overlap=0.042,
        elem_a=cap_internal_threads,
        elem_b=neck_threads,
        name="internal and external thread envelopes match",
    )
    ctx.expect_gap(
        screw_cap,
        bottle_body,
        axis="z",
        min_gap=0.0010,
        max_gap=0.0045,
        positive_elem=cap_top_plug,
        negative_elem=neck_lip,
        name="cap top clears bottle lip by practical margin",
    )

    bottle_aabb = ctx.part_world_aabb(bottle_body)
    cap_aabb = ctx.part_world_aabb(screw_cap)
    bottle_shell_aabb = ctx.part_element_world_aabb(bottle_body, elem=bottle_shell)
    front_plate_aabb = ctx.part_element_world_aabb(bottle_body, elem=front_plate)
    rear_plate_aabb = ctx.part_element_world_aabb(bottle_body, elem=rear_plate)
    thread_aabb = ctx.part_element_world_aabb(bottle_body, elem=neck_threads)
    cap_thread_aabb = ctx.part_element_world_aabb(screw_cap, elem=cap_internal_threads)

    assert bottle_aabb is not None
    assert cap_aabb is not None
    assert bottle_shell_aabb is not None
    assert front_plate_aabb is not None
    assert rear_plate_aabb is not None
    assert thread_aabb is not None
    assert cap_thread_aabb is not None

    overall_height = max(bottle_aabb[1][2], cap_aabb[1][2]) - min(bottle_aabb[0][2], cap_aabb[0][2])
    shell_height = bottle_shell_aabb[1][2] - bottle_shell_aabb[0][2]
    front_plate_height = front_plate_aabb[1][2] - front_plate_aabb[0][2]
    rear_plate_height = rear_plate_aabb[1][2] - rear_plate_aabb[0][2]
    thread_height = thread_aabb[1][2] - thread_aabb[0][2]
    cap_thread_height = cap_thread_aabb[1][2] - cap_thread_aabb[0][2]

    ctx.check(
        "overall bottle height practical",
        0.34 <= overall_height <= 0.37,
        f"overall height was {overall_height:.4f} m",
    )
    ctx.check(
        "molded body height substantial",
        0.33 <= shell_height <= 0.35,
        f"body shell height was {shell_height:.4f} m",
    )
    ctx.check(
        "thread engagement depth practical",
        0.020 <= thread_height <= 0.026 and 0.020 <= cap_thread_height <= 0.026,
        (
            f"body/cap thread heights were "
            f"{thread_height:.4f} m and {cap_thread_height:.4f} m"
        ),
    )
    ctx.check(
        "reinforcement plates are substantial",
        front_plate_height >= 0.11 and rear_plate_height >= 0.11,
        (
            f"front/rear plate heights were "
            f"{front_plate_height:.4f} m and {rear_plate_height:.4f} m"
        ),
    )

    with ctx.pose({cap_thread: math.tau * 0.33}):
        ctx.expect_origin_distance(
            screw_cap,
            bottle_body,
            axes="xy",
            max_dist=0.0005,
            name="rotated cap stays coaxial",
        )
        ctx.expect_contact(
            screw_cap,
            bottle_body,
            elem_a=cap_shell,
            elem_b=cap_stop_ring,
            name="rotated cap still bears on stop ring",
        )
        ctx.expect_within(
            bottle_body,
            screw_cap,
            axes="xy",
            margin=0.0,
            inner_elem=neck_finish,
            outer_elem=cap_shell,
            name="rotated cap still envelopes neck finish",
        )
        ctx.expect_overlap(
            screw_cap,
            bottle_body,
            axes="xy",
            min_overlap=0.042,
            elem_a=cap_internal_threads,
            elem_b=neck_threads,
            name="rotated thread envelopes stay coaxial",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
