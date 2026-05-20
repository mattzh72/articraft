from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _helix_points(
    *,
    radius: float,
    z_start: float,
    z_end: float,
    turns: float,
    phase: float = 0.0,
    samples_per_turn: int = 24,
) -> list[tuple[float, float, float]]:
    sample_count = max(8, int(math.ceil(turns * samples_per_turn)) + 1)
    points: list[tuple[float, float, float]] = []
    for index in range(sample_count):
        t = index / (sample_count - 1)
        angle = phase + (math.tau * turns * t)
        z_pos = z_start + ((z_end - z_start) * t)
        points.append((radius * math.cos(angle), radius * math.sin(angle), z_pos))
    return points


def _thread_bead(
    *,
    radius: float,
    z_start: float,
    z_end: float,
    turns: float,
    bead_radius: float,
    phase: float = 0.0,
):
    return tube_from_spline_points(
        _helix_points(
            radius=radius,
            z_start=z_start,
            z_end=z_end,
            turns=turns,
            phase=phase,
            samples_per_turn=26,
        ),
        radius=bead_radius,
        samples_per_segment=2,
        radial_segments=12,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_screwcap_bottle")

    bottle_polymer = model.material("bottle_polymer", rgba=(0.82, 0.86, 0.88, 0.78))
    cap_resin = model.material("cap_resin", rgba=(0.12, 0.13, 0.15, 1.0))
    bright_mark = model.material("bright_mark", rgba=(0.90, 0.92, 0.95, 1.0))
    datum_gray = model.material("datum_gray", rgba=(0.62, 0.66, 0.70, 1.0))

    bottle_body = model.part("bottle_body")

    bottle_outer_profile = [
        (0.0000, 0.0000),
        (0.0305, 0.0000),
        (0.0328, 0.0040),
        (0.0338, 0.0120),
        (0.0338, 0.1020),
        (0.0328, 0.1150),
        (0.0288, 0.1240),
        (0.0225, 0.1320),
        (0.0172, 0.1385),
        (0.0140, 0.1450),
        (0.0138, 0.1620),
        (0.0146, 0.1650),
    ]
    bottle_inner_profile = [
        (0.0000, 0.0000),
        (0.0060, 0.0025),
        (0.0200, 0.0060),
        (0.0290, 0.0120),
        (0.0296, 0.1010),
        (0.0286, 0.1140),
        (0.0248, 0.1230),
        (0.0180, 0.1330),
        (0.0122, 0.1450),
        (0.0108, 0.1500),
        (0.0108, 0.1630),
    ]
    bottle_body.visual(
        _save_mesh(
            "bottle_shell",
            LatheGeometry.from_shell_profiles(
                bottle_outer_profile,
                bottle_inner_profile,
                segments=80,
                start_cap="flat",
                end_cap="flat",
                lip_samples=8,
            ),
        ),
        material=bottle_polymer,
        name="bottle_shell",
    )
    bottle_body.visual(
        Cylinder(radius=0.0137, length=0.0140),
        origin=Origin(xyz=(0.0, 0.0, 0.1550)),
        material=bottle_polymer,
        name="neck_land",
    )
    bottle_body.visual(
        _save_mesh(
            "neck_thread",
            _thread_bead(
                radius=0.01445,
                z_start=0.1470,
                z_end=0.1580,
                turns=2.0,
                bead_radius=0.00042,
                phase=0.22,
            ),
        ),
        material=bottle_polymer,
        name="thread_major",
    )
    bottle_body.visual(
        Cylinder(radius=0.0166, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.1390)),
        material=datum_gray,
        name="shoulder_stop",
    )
    bottle_body.visual(
        Cylinder(radius=0.0118, length=0.0010),
        origin=Origin(xyz=(0.0, 0.0, 0.1638)),
        material=datum_gray,
        name="lip_land",
    )
    bottle_body.visual(
        Box((0.0200, 0.0042, 0.0300)),
        origin=Origin(xyz=(0.0, 0.0312, 0.0920)),
        material=datum_gray,
        name="datum_panel",
    )
    bottle_body.visual(
        Box((0.0030, 0.0024, 0.0140)),
        origin=Origin(xyz=(0.0, 0.0208, 0.1345)),
        material=bright_mark,
        name="front_index",
    )
    for index, angle in enumerate((-0.34, -0.17, 0.0, 0.17, 0.34)):
        x_pos = 0.0215 * math.sin(angle)
        y_pos = 0.0215 * math.cos(angle)
        bottle_body.visual(
            Box((0.0016, 0.0022, 0.0060)),
            origin=Origin(xyz=(x_pos, y_pos, 0.1360), rpy=(0.0, 0.0, -angle)),
            material=bright_mark,
            name=f"neck_tick_{index:02d}",
        )
    bottle_body.inertial = Inertial.from_geometry(
        Box((0.068, 0.068, 0.165)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.0825)),
    )

    cap = model.part("cap")
    cap_outer_profile = [
        (0.0208, 0.0000),
        (0.0215, 0.0028),
        (0.0222, 0.0070),
        (0.0222, 0.0228),
        (0.0216, 0.0280),
    ]
    cap_inner_profile = [
        (0.0196, 0.0000),
        (0.0192, 0.0030),
        (0.0181, 0.0062),
        (0.0175, 0.0100),
        (0.0172, 0.0185),
        (0.0158, 0.0220),
        (0.0122, 0.0236),
        (0.0000, 0.0246),
    ]
    cap.visual(
        _save_mesh(
            "cap_shell",
            LatheGeometry.from_shell_profiles(
                cap_outer_profile,
                cap_inner_profile,
                segments=80,
                start_cap="flat",
                end_cap="flat",
                lip_samples=8,
            ),
        ),
        material=cap_resin,
        name="cap_shell",
    )
    cap.visual(
        _save_mesh(
            "cap_thread",
            _thread_bead(
                radius=0.01555,
                z_start=0.0055,
                z_end=0.0165,
                turns=2.0,
                bead_radius=0.00045,
                phase=0.22,
            ),
        ),
        material=cap_resin,
        name="cap_thread",
    )
    thread_bridge_angle = 0.22
    for name, z_pos in (("thread_bridge_lower", 0.0048), ("thread_bridge_upper", 0.0169)):
        cap.visual(
            Box((0.0040, 0.0012, 0.0022)),
            origin=Origin(
                xyz=(0.0178 * math.cos(thread_bridge_angle), 0.0178 * math.sin(thread_bridge_angle), z_pos),
                rpy=(0.0, 0.0, thread_bridge_angle),
            ),
            material=cap_resin,
            name=name,
        )
    cap.visual(
        Cylinder(radius=0.0202, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0006)),
        material=cap_resin,
        name="cap_lower_datum",
    )
    cap.visual(
        Cylinder(radius=0.0112, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0238)),
        material=datum_gray,
        name="seal_land",
    )
    cap.visual(
        Box((0.0026, 0.0048, 0.0120)),
        origin=Origin(xyz=(0.0, 0.0213, 0.0080)),
        material=bright_mark,
        name="cap_pointer",
    )
    cap.visual(
        Box((0.0160, 0.0022, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, 0.0252)),
        material=bright_mark,
        name="top_index_bar",
    )
    for index in range(18):
        angle = (math.tau * index) / 18.0
        cap.visual(
            Box((0.0042, 0.0048, 0.0160)),
            origin=Origin(
                xyz=(0.0207 * math.cos(angle), 0.0207 * math.sin(angle), 0.0100),
                rpy=(0.0, 0.0, angle),
            ),
            material=cap_resin,
            name=f"grip_rib_{index:02d}",
        )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0210, length=0.0280),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.0140)),
    )

    model.articulation(
        "cap_rotation",
        ArticulationType.REVOLUTE,
        parent=bottle_body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, 0.1418)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=8.0,
            lower=0.0,
            upper=6.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle_body = object_model.get_part("bottle_body")
    cap = object_model.get_part("cap")
    cap_rotation = object_model.get_articulation("cap_rotation")
    bottle_shell = bottle_body.get_visual("bottle_shell")
    cap_shell = cap.get_visual("cap_shell")

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
    ctx.allow_overlap(
        bottle_body,
        cap,
        elem_a=bottle_shell,
        elem_b=cap_shell,
        reason=(
            "Nested watertight shell meshes express a hollow screwcap over a hollow bottle neck; "
            "exact datum-gap checks below verify the intended controlled clearances even though "
            "the shell pair is compiled as overlapping volume."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    cap_limits = cap_rotation.motion_limits
    ctx.check(
        "cap_joint_is_axial_revolute",
        cap_rotation.axis == (0.0, 0.0, 1.0),
        f"axis={cap_rotation.axis}",
    )
    ctx.check(
        "cap_joint_has_multi_turn_travel",
        cap_limits is not None
        and cap_limits.lower == 0.0
        and cap_limits.upper is not None
        and cap_limits.upper >= 6.0,
        f"limits={cap_limits}",
    )

    with ctx.pose({cap_rotation: 0.0}):
        ctx.expect_overlap(
            cap,
            bottle_body,
            axes="xy",
            min_overlap=0.029,
            name="cap_remains_coaxial_with_bottle",
        )
        ctx.expect_gap(
            cap,
            bottle_body,
            axis="z",
            positive_elem="cap_lower_datum",
            negative_elem="shoulder_stop",
            min_gap=0.0012,
            max_gap=0.0032,
            name="controlled_skirt_gap_above_shoulder_stop",
        )
        ctx.expect_gap(
            cap,
            bottle_body,
            axis="z",
            positive_elem="seal_land",
            negative_elem="lip_land",
            min_gap=0.0002,
            max_gap=0.0012,
            name="controlled_seal_gap_above_lip_land",
        )
        ctx.expect_overlap(
            cap,
            bottle_body,
            axes="xy",
            elem_a="cap_thread",
            elem_b="thread_major",
            min_overlap=0.028,
            name="thread_engagement_reads_as_coaxial_mating",
        )

    with ctx.pose({cap_rotation: math.pi}):
        ctx.expect_origin_distance(
            cap,
            bottle_body,
            axes="xy",
            max_dist=0.0001,
            name="cap_rotates_about_bottle_axis",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
