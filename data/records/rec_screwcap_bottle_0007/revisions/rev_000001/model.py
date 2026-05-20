from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(filename: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    z_center: float,
    height: float,
    radial_segments: int = 64,
) -> MeshGeometry:
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.001,
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
    samples_per_turn: int = 16,
) -> list[tuple[float, float, float]]:
    sample_count = max(16, int(math.ceil(abs(turns) * samples_per_turn)))
    points: list[tuple[float, float, float]] = []
    for index in range(sample_count + 1):
        t = index / sample_count
        angle = phase + (turns * math.tau * t)
        points.append(
            (
                radius * math.cos(angle),
                radius * math.sin(angle),
                z_start + (turns * pitch * t),
            )
        )
    return points


def _thread_mesh(
    *,
    radius: float,
    z_start: float,
    turns: float,
    pitch: float,
    profile_width: float,
    profile_height: float,
    phase: float = 0.0,
) -> MeshGeometry:
    return sweep_profile_along_spline(
        _helix_points(
            radius=radius,
            z_start=z_start,
            turns=turns,
            pitch=pitch,
            phase=phase,
        ),
        profile=rounded_rect_profile(
            profile_width,
            profile_height,
            radius=min(profile_width, profile_height) * 0.25,
            corner_segments=4,
        ),
        samples_per_segment=4,
        cap_profile=True,
    )


def _build_bottle_body_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.0, 0.0060),
            (0.0100, 0.0040),
            (0.0220, 0.0010),
            (0.0308, 0.0000),
            (0.0320, 0.0100),
            (0.0323, 0.0480),
            (0.0322, 0.0980),
            (0.0310, 0.1180),
            (0.0278, 0.1310),
            (0.0240, 0.1388),
            (0.0218, 0.1412),
        ],
        [
            (0.0, 0.0190),
            (0.0090, 0.0160),
            (0.0220, 0.0130),
            (0.0280, 0.0140),
            (0.0292, 0.0220),
            (0.0290, 0.0480),
            (0.0289, 0.0970),
            (0.0277, 0.1180),
            (0.0244, 0.1310),
            (0.0210, 0.1388),
            (0.0190, 0.1412),
        ],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )


def _build_bottle_finish_mesh() -> MeshGeometry:
    finish_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0240, 0.1388),
            (0.0218, 0.1410),
            (0.0198, 0.1472),
            (0.0188, 0.1534),
            (0.0183, 0.1612),
            (0.0182, 0.1715),
        ],
        [
            (0.0210, 0.1388),
            (0.0190, 0.1410),
            (0.0171, 0.1472),
            (0.0160, 0.1534),
            (0.0156, 0.1612),
            (0.0155, 0.1715),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    return _merge_geometries(
        [
            finish_shell,
            _ring_band(
                outer_radius=0.0208,
                inner_radius=0.0172,
                z_center=0.1514,
                height=0.0042,
                radial_segments=56,
            ),
            _thread_mesh(
                radius=0.0180,
                z_start=0.1537,
                turns=2.0,
                pitch=0.0063,
                profile_width=0.0010,
                profile_height=0.0012,
                phase=math.radians(18.0),
            ),
            _ring_band(
                outer_radius=0.0182,
                inner_radius=0.0155,
                z_center=0.1708,
                height=0.0014,
                radial_segments=56,
            ),
        ]
    )


def _build_label_band_mesh() -> MeshGeometry:
    return _ring_band(
        outer_radius=0.0329,
        inner_radius=0.0312,
        z_center=0.0740,
        height=0.0450,
        radial_segments=72,
    )


def _build_cap_shell_mesh() -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.0248, 0.0000),
            (0.0250, 0.0080),
            (0.0250, 0.0245),
            (0.0243, 0.0305),
            (0.0229, 0.0320),
        ],
        [
            (0.0216, 0.0000),
            (0.0216, 0.0258),
            (0.0186, 0.0288),
            (0.0100, 0.0302),
            (0.0000, 0.0308),
        ],
        segments=84,
        start_cap="flat",
        end_cap="flat",
    )

    ribs = MeshGeometry()
    for index in range(18):
        rib = BoxGeometry((0.0016, 0.0030, 0.0190)).translate(0.0242, 0.0, 0.0115)
        rib.rotate_z(index * math.tau / 18.0)
        ribs.merge(rib)

    return _merge_geometries(
        [
            shell,
            ribs,
            _thread_mesh(
                radius=0.0202,
                z_start=0.0092,
                turns=2.0,
                pitch=0.0063,
                profile_width=0.0009,
                profile_height=0.0011,
                phase=math.radians(18.0),
            ),
            _ring_band(
                outer_radius=0.0216,
                inner_radius=0.0194,
                z_center=0.0093,
                height=0.0020,
                radial_segments=56,
            ),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_screwcap_bottle", assets=ASSETS)

    bottle_satin = model.material("bottle_satin", rgba=(0.42, 0.46, 0.49, 0.72))
    bottle_matte = model.material("bottle_matte", rgba=(0.78, 0.79, 0.80, 1.0))
    finish_satin = model.material("finish_satin", rgba=(0.58, 0.60, 0.63, 0.96))
    cap_satin = model.material("cap_satin", rgba=(0.14, 0.15, 0.17, 1.0))

    bottle = model.part("bottle_body")
    bottle.visual(
        _save_mesh("bottle_shell.obj", _build_bottle_body_shell_mesh()),
        material=bottle_satin,
        name="bottle_shell",
    )
    bottle.visual(
        _save_mesh("bottle_finish.obj", _build_bottle_finish_mesh()),
        material=finish_satin,
        name="bottle_finish",
    )
    bottle.visual(
        _save_mesh("label_band.obj", _build_label_band_mesh()),
        material=bottle_matte,
        name="label_band",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0330, length=0.1730),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.0865)),
    )

    cap = model.part("screw_cap")
    cap.visual(
        _save_mesh("cap_shell.obj", _build_cap_shell_mesh()),
        material=cap_satin,
        name="cap_shell",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0250, length=0.0320),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, 0.0160)),
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.1452)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bottle = object_model.get_part("bottle_body")
    cap = object_model.get_part("screw_cap")
    cap_spin = object_model.get_articulation("cap_spin")

    bottle_shell = bottle.get_visual("bottle_shell")
    bottle_finish = bottle.get_visual("bottle_finish")
    label_band = bottle.get_visual("label_band")
    cap_shell = cap.get_visual("cap_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        bottle,
        cap,
        reason="The screw cap intentionally nests around the bottle's threaded finish; seating, coaxiality, and engagement depth are verified with exact checks.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=8,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    ctx.expect_origin_distance(
        cap,
        bottle,
        axes="xy",
        max_dist=0.0005,
        name="cap_is_coaxial_with_bottle",
    )
    ctx.expect_within(
        bottle,
        cap,
        axes="xy",
        inner_elem=bottle_finish,
        outer_elem=cap_shell,
        margin=0.0012,
        name="neck_finish_stays_within_cap_skirt",
    )
    ctx.expect_overlap(
        cap,
        bottle,
        axes="z",
        elem_a=cap_shell,
        elem_b=bottle_finish,
        min_overlap=0.020,
        name="cap_engagement_depth_is_practical",
    )
    ctx.expect_contact(
        cap,
        bottle,
        elem_a=cap_shell,
        elem_b=bottle_finish,
        name="cap_seats_on_finish_without_floating",
    )

    bottle_aabb = ctx.part_world_aabb(bottle)
    cap_aabb = ctx.part_world_aabb(cap)
    shell_aabb = ctx.part_element_world_aabb(bottle, elem=bottle_shell)
    finish_aabb = ctx.part_element_world_aabb(bottle, elem=bottle_finish)
    label_aabb = ctx.part_element_world_aabb(bottle, elem=label_band)

    ctx.check(
        "proportions_read_as_premium_bottle",
        bottle_aabb is not None
        and cap_aabb is not None
        and (bottle_aabb[1][2] - bottle_aabb[0][2]) > 0.170
        and (bottle_aabb[1][0] - bottle_aabb[0][0]) > 0.064
        and 0.048 <= (cap_aabb[1][0] - cap_aabb[0][0]) <= 0.052,
        details=f"bottle_aabb={bottle_aabb}, cap_aabb={cap_aabb}",
    )
    ctx.check(
        "label_band_is_centered_on_body_land",
        label_aabb is not None
        and shell_aabb is not None
        and label_aabb[0][2] > shell_aabb[0][2] + 0.040
        and label_aabb[1][2] < shell_aabb[1][2] - 0.040,
        details=f"label_aabb={label_aabb}, shell_aabb={shell_aabb}",
    )
    ctx.check(
        "cap_crown_clearance_above_finish_is_balanced",
        cap_aabb is not None
        and finish_aabb is not None
        and 0.004 <= (cap_aabb[1][2] - finish_aabb[1][2]) <= 0.008,
        details=f"cap_aabb={cap_aabb}, finish_aabb={finish_aabb}",
    )
    ctx.check(
        "cap_break_starts_above_body_shoulder",
        cap_aabb is not None
        and shell_aabb is not None
        and 0.003 <= (cap_aabb[0][2] - shell_aabb[1][2]) <= 0.008,
        details=f"cap_aabb={cap_aabb}, shell_aabb={shell_aabb}",
    )

    with ctx.pose({cap_spin: math.pi / 2.0}):
        ctx.expect_origin_distance(
            cap,
            bottle,
            axes="xy",
            max_dist=0.0005,
            name="cap_remains_coaxial_at_quarter_turn",
        )
        ctx.expect_within(
            bottle,
            cap,
            axes="xy",
            inner_elem=bottle_finish,
            outer_elem=cap_shell,
            margin=0.0012,
            name="neck_finish_stays_within_cap_skirt_at_quarter_turn",
        )

    with ctx.pose({cap_spin: math.pi}):
        ctx.expect_overlap(
            cap,
            bottle,
            axes="z",
            elem_a=cap_shell,
            elem_b=bottle_finish,
            min_overlap=0.020,
            name="cap_engagement_depth_holds_at_half_turn",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
