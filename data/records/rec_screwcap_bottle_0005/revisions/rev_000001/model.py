from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Cylinder,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
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


def _helix_points(
    radius: float,
    z_start: float,
    turns: float,
    pitch: float,
    *,
    phase: float = 0.0,
    samples_per_turn: int = 72,
) -> list[tuple[float, float, float]]:
    total_samples = max(12, int(math.ceil(turns * samples_per_turn)))
    points: list[tuple[float, float, float]] = []
    for index in range(total_samples + 1):
        t = index / total_samples
        angle = phase + (t * turns * math.tau)
        points.append(
            (
                radius * math.cos(angle),
                radius * math.sin(angle),
                z_start + (t * turns * pitch),
            )
        )
    return points


def _thread_geometry(
    *,
    radius: float,
    z_start: float,
    turns: float,
    pitch: float,
    thread_radius: float,
    starts: int = 1,
) -> MeshGeometry:
    thread_starts: list[MeshGeometry] = []
    for start_index in range(starts):
        phase = (math.tau * start_index) / starts
        thread_starts.append(
            wire_from_points(
                _helix_points(
                    radius,
                    z_start,
                    turns,
                    pitch,
                    phase=phase,
                    samples_per_turn=84,
                ),
                radius=thread_radius,
                radial_segments=12,
                cap_ends=True,
                corner_mode="miter",
                up_hint=(1.0, 0.0, 0.0),
            )
        )
    return _merge_geometries(thread_starts)


def _band_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    segments: int = 72,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z0), (outer_radius, z1)],
        [(inner_radius, z0), (inner_radius, z1)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_screwcap_bottle", assets=ASSETS)

    bottle_matte = model.material("bottle_matte", rgba=(0.76, 0.80, 0.75, 0.88))
    bottle_satin = model.material("bottle_satin", rgba=(0.88, 0.90, 0.86, 0.96))
    cap_satin = model.material("cap_satin", rgba=(0.24, 0.25, 0.27, 1.0))
    cap_matte = model.material("cap_matte", rgba=(0.14, 0.15, 0.16, 1.0))
    seal_cream = model.material("seal_cream", rgba=(0.94, 0.93, 0.89, 1.0))

    cap_base_z = 0.209

    bottle = model.part("bottle")
    bottle.visual(
        _save_mesh(
            "bottle_body_shell.obj",
            LatheGeometry.from_shell_profiles(
                [
                    (0.0000, 0.0000),
                    (0.0210, 0.0025),
                    (0.0335, 0.0075),
                    (0.0382, 0.0160),
                    (0.0385, 0.0660),
                    (0.0380, 0.1420),
                    (0.0345, 0.1720),
                    (0.0285, 0.1900),
                    (0.0230, 0.2045),
                    (0.0205, 0.2145),
                    (0.0192, 0.2285),
                    (0.0184, 0.2365),
                    (0.0176, 0.2390),
                ],
                [
                    (0.0100, 0.0080),
                    (0.0165, 0.0190),
                    (0.0322, 0.0290),
                    (0.0338, 0.1430),
                    (0.0305, 0.1710),
                    (0.0260, 0.1885),
                    (0.0210, 0.2040),
                    (0.0160, 0.2180),
                    (0.0132, 0.2305),
                    (0.0111, 0.2390),
                ],
                segments=96,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=bottle_matte,
        name="body_shell",
    )
    bottle.visual(
        _save_mesh(
            "bottle_neck_thread.obj",
            _thread_geometry(
                radius=0.0194,
                z_start=0.2132,
                turns=1.75,
                pitch=0.0081,
                thread_radius=0.00085,
            ),
        ),
        material=bottle_satin,
        name="neck_thread",
    )
    bottle.visual(
        _save_mesh(
            "bottle_label_band.obj",
            _band_shell(
                outer_radius=0.0392,
                inner_radius=0.0381,
                z0=0.076,
                z1=0.136,
            ),
        ),
        material=bottle_satin,
        name="label_band",
    )
    bottle.visual(
        _save_mesh(
            "bottle_support_ring.obj",
            _band_shell(
                outer_radius=0.0216,
                inner_radius=0.0190,
                z0=0.2056,
                z1=0.2082,
            ),
        ),
        material=bottle_satin,
        name="support_ring",
    )
    bottle.visual(
        _save_mesh(
            "bottle_lip_land.obj",
            _band_shell(
                outer_radius=0.0174,
                inner_radius=0.0111,
                z0=0.2360,
                z1=0.2390,
            ),
        ),
        material=bottle_satin,
        name="lip_land",
    )
    bottle.visual(
        _save_mesh(
            "bottle_heel_ring.obj",
            TorusGeometry(
                radius=0.0345,
                tube=0.0016,
                radial_segments=18,
                tubular_segments=72,
            ).translate(0.0, 0.0, 0.0120),
        ),
        material=bottle_satin,
        name="heel_ring",
    )
    bottle.visual(
        _save_mesh(
            "bottle_shoulder_bead.obj",
            TorusGeometry(
                radius=0.0307,
                tube=0.0013,
                radial_segments=18,
                tubular_segments=72,
            ).translate(0.0, 0.0, 0.1835),
        ),
        material=bottle_satin,
        name="shoulder_bead",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.239),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.1195)),
    )

    cap = model.part("cap")
    cap.visual(
        _save_mesh(
            "cap_shell.obj",
            LatheGeometry.from_shell_profiles(
                [
                    (0.0244, 0.0000),
                    (0.0247, 0.0040),
                    (0.0247, 0.0300),
                    (0.0238, 0.0360),
                    (0.0195, 0.0395),
                    (0.0000, 0.0400),
                ],
                [
                    (0.0224, 0.0000),
                    (0.0224, 0.0315),
                    (0.0185, 0.0348),
                    (0.0136, 0.0355),
                ],
                segments=96,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=cap_satin,
        name="cap_shell",
    )
    cap.visual(
        _save_mesh(
            "cap_thread.obj",
            _thread_geometry(
                radius=0.0217,
                z_start=0.0045,
                turns=1.70,
                pitch=0.0081,
                thread_radius=0.00085,
            ),
        ),
        material=cap_matte,
        name="cap_thread",
    )
    cap.visual(
        _save_mesh(
            "cap_grip_sleeve.obj",
            _band_shell(
                outer_radius=0.0251,
                inner_radius=0.0236,
                z0=0.0040,
                z1=0.0290,
            ),
        ),
        material=cap_matte,
        name="grip_sleeve",
    )
    cap.visual(
        Cylinder(radius=0.0160, length=0.0090),
        origin=Origin(xyz=(0.0, 0.0, 0.0345)),
        material=seal_cream,
        name="liner_land",
    )
    cap.visual(
        Cylinder(radius=0.0160, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0394)),
        material=cap_matte,
        name="top_insert",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.025, length=0.040),
        mass=0.024,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, cap_base_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("cap")
    cap_spin = object_model.get_articulation("cap_spin")

    body_shell = bottle.get_visual("body_shell")
    neck_thread = bottle.get_visual("neck_thread")
    support_ring = bottle.get_visual("support_ring")
    lip_land = bottle.get_visual("lip_land")

    cap_shell = cap.get_visual("cap_shell")
    cap_thread = cap.get_visual("cap_thread")
    liner_land = cap.get_visual("liner_land")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=12, name="cap_rotation_clearance")

    ctx.expect_origin_distance(
        cap,
        bottle,
        axes="xy",
        max_dist=0.0005,
        name="cap_is_coaxial_with_bottle",
    )
    ctx.expect_origin_gap(
        cap,
        bottle,
        axis="z",
        min_gap=0.2085,
        max_gap=0.2095,
        name="cap_seating_height",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="xy",
        min_overlap=0.038,
        elem_a=cap_shell,
        elem_b=neck_thread,
        name="cap_covers_threaded_finish",
    )
    ctx.expect_within(
        bottle,
        cap,
        axes="xy",
        margin=0.0,
        inner_elem=neck_thread,
        outer_elem=cap_shell,
        name="thread_stays_within_cap_footprint",
    )
    ctx.expect_contact(
        cap,
        bottle,
        elem_a=liner_land,
        elem_b=lip_land,
        contact_tol=0.0002,
        name="liner_seats_on_lip_land",
    )
    ctx.expect_gap(
        cap,
        bottle,
        axis="z",
        positive_elem=cap_shell,
        negative_elem=support_ring,
        min_gap=0.0005,
        max_gap=0.0012,
        name="clean_cap_break_above_support_ring",
    )

    with ctx.pose({cap_spin: math.tau / 3.0}):
        ctx.expect_origin_distance(
            cap,
            bottle,
            axes="xy",
            max_dist=0.0005,
            name="cap_remains_coaxial_when_rotated",
        )
        ctx.expect_contact(
            cap,
            bottle,
            elem_a=liner_land,
            elem_b=lip_land,
            contact_tol=0.0002,
            name="liner_stays_seated_when_rotated",
        )
        ctx.expect_gap(
            cap,
            bottle,
            axis="z",
            positive_elem=cap_shell,
            negative_elem=support_ring,
            min_gap=0.0005,
            max_gap=0.0012,
            name="support_ring_clearance_persists_when_rotated",
        )
        ctx.expect_overlap(
            cap,
            bottle,
            axes="xy",
            min_overlap=0.038,
            elem_a=cap_thread,
            elem_b=neck_thread,
            name="thread_axes_stay_registered_in_rotation",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
