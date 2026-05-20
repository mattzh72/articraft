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
    BoxGeometry,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    sample_catmull_rom_spline_2d,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

LOWER_DEPTH = 0.055
LID_DEPTH = 0.085
LOWER_FLOOR_THICKNESS = 0.005
LID_TOP_THICKNESS = 0.007
HINGE_Y = -0.108
HANDLE_CENTER_X = 0.060
HANDLE_SPAN = 0.128
HANDLE_FRONT_Y = 0.138
HANDLE_BOSS_HEIGHT = 0.008
HANDLE_BOSS_SIZE = (0.028, 0.018, HANDLE_BOSS_HEIGHT)
HINGE_LEAF_SIZE = (0.122, 0.016, 0.010)
HANDLE_MOUNT_EMBED = 0.002
HINGE_BAND_SIZE = (0.640, 0.020, 0.010)
HANDLE_BRIDGE_SIZE = (0.182, 0.038, 0.010)


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _violin_outline() -> list[tuple[float, float]]:
    control_points = [
        (0.425, 0.000),
        (0.392, 0.128),
        (0.278, 0.160),
        (0.128, 0.124),
        (0.018, 0.094),
        (-0.118, 0.108),
        (-0.256, 0.086),
        (-0.356, 0.054),
        (-0.418, 0.018),
        (-0.426, 0.000),
        (-0.418, -0.018),
        (-0.356, -0.054),
        (-0.256, -0.086),
        (-0.118, -0.108),
        (0.018, -0.094),
        (0.128, -0.124),
        (0.278, -0.160),
        (0.392, -0.128),
    ]
    outline = sample_catmull_rom_spline_2d(
        control_points,
        samples_per_segment=14,
        closed=True,
    )
    if outline and outline[0] == outline[-1]:
        outline = outline[:-1]
    return outline


def _scale_profile(
    profile: list[tuple[float, float]],
    sx: float,
    sy: float,
) -> list[tuple[float, float]]:
    return [(sx * x, sy * y) for x, y in profile]


def _build_lower_shell_geometry(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
):
    return repair_loft(
        section_loft(
            [
                [(x, y, 0.0) for x, y in _scale_profile(outer_profile, 0.952, 0.900)],
                [(x, y, 0.020) for x, y in _scale_profile(outer_profile, 0.978, 0.952)],
                [(x, y, LOWER_DEPTH) for x, y in outer_profile],
            ]
        )
    )


def _build_lid_shell_geometry(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
):
    return repair_loft(
        section_loft(
            [
                [(x, y, 0.0) for x, y in outer_profile],
                [(x, y, 0.040) for x, y in _scale_profile(outer_profile, 0.978, 0.948)],
                [(x, y, LID_DEPTH) for x, y in _scale_profile(outer_profile, 0.938, 0.870)],
            ]
        )
    )


def _build_handle_geometry():
    half_span = HANDLE_SPAN * 0.5
    handle = BoxGeometry((0.026, 0.018, 0.008)).translate(-half_span, 0.0, 0.004)
    handle.merge(BoxGeometry((0.026, 0.018, 0.008)).translate(half_span, 0.0, 0.004))
    handle.merge(
        tube_from_spline_points(
            [
                (-half_span, 0.0, 0.008),
                (-0.050, 0.0, 0.028),
                (-0.018, 0.0, 0.045),
                (0.018, 0.0, 0.045),
                (0.050, 0.0, 0.028),
                (half_span, 0.0, 0.008),
            ],
            radius=0.0055,
            samples_per_segment=16,
            radial_segments=18,
        )
    )
    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="violin_case", assets=ASSETS)

    shell_black = model.material("shell_black", rgba=(0.15, 0.15, 0.16, 1.0))
    plush_burgundy = model.material("plush_burgundy", rgba=(0.42, 0.09, 0.11, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.24, 0.24, 0.25, 1.0))

    outer_profile = _violin_outline()
    inner_profile = _scale_profile(outer_profile, 0.930, 0.875)
    lower_lining_profile = _scale_profile(inner_profile, 0.975, 0.955)
    lid_lining_profile = _scale_profile(inner_profile, 0.970, 0.948)

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        _save_mesh(_build_lower_shell_geometry(outer_profile, inner_profile), "lower_shell.obj"),
        material=shell_black,
        name="lower_shell_body",
    )
    lower_shell.visual(
        _save_mesh(
            ExtrudeGeometry.from_z0(lower_lining_profile, 0.007).translate(
                0.0,
                0.0,
                LOWER_FLOOR_THICKNESS,
            ),
            "lower_lining.obj",
        ),
        material=plush_burgundy,
        name="lower_lining",
    )
    lower_shell.visual(
        Box(HINGE_BAND_SIZE),
        origin=Origin(
            xyz=(0.0, HINGE_Y + 0.008, LOWER_DEPTH - 0.005),
        ),
        material=hardware_dark,
        name="rear_hinge_band",
    )
    for index, x_center in enumerate((-0.235, 0.000, 0.235), start=1):
        lower_shell.visual(
            Cylinder(radius=0.0045, length=0.120),
            origin=Origin(
                xyz=(x_center, HINGE_Y, LOWER_DEPTH - 0.0005),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hardware_dark,
            name=f"lower_hinge_knuckle_{index}",
        )
    lower_shell.inertial = Inertial.from_geometry(
        Box((0.86, 0.34, LOWER_DEPTH)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * LOWER_DEPTH)),
    )

    lid = model.part("lid")
    lid.visual(
        _save_mesh(_build_lid_shell_geometry(outer_profile, inner_profile), "lid_shell.obj"),
        origin=Origin(xyz=(0.0, -HINGE_Y, 0.0)),
        material=shell_black,
        name="lid_shell_body",
    )
    lid.visual(
        _save_mesh(
            ExtrudeGeometry.from_z0(lid_lining_profile, 0.008).translate(
                0.0,
                0.0,
                LID_DEPTH - LID_TOP_THICKNESS - 0.008,
            ),
            "lid_lining.obj",
        ),
        origin=Origin(xyz=(0.0, -HINGE_Y, 0.0)),
        material=plush_burgundy,
        name="lid_lining",
    )
    for index, x_center in enumerate((-0.117, 0.117), start=1):
        lid.visual(
            Cylinder(radius=0.0045, length=0.105),
            origin=Origin(
                xyz=(x_center, 0.0, 0.0005),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hardware_dark,
            name=f"lid_hinge_knuckle_{index}",
        )
    lid.visual(
        Box(HANDLE_BRIDGE_SIZE),
        origin=Origin(
            xyz=(
                HANDLE_CENTER_X,
                HANDLE_FRONT_Y - HINGE_Y - 0.010,
                LID_DEPTH + 0.5 * HANDLE_BOSS_HEIGHT - HANDLE_MOUNT_EMBED,
            ),
        ),
        material=hardware_dark,
        name="handle_mount_bridge",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.86, 0.34, LID_DEPTH)),
        mass=1.8,
        origin=Origin(xyz=(0.0, -HINGE_Y, 0.5 * LID_DEPTH)),
    )

    handle = model.part("handle")
    handle.visual(
        _save_mesh(_build_handle_geometry(), "handle.obj"),
        material=hardware_dark,
        name="handle_body",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.16, 0.03, 0.05)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    model.articulation(
        "case_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, LOWER_DEPTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "lid_to_handle",
        ArticulationType.FIXED,
        parent=lid,
        child=handle,
        origin=Origin(
            xyz=(
                HANDLE_CENTER_X,
                HANDLE_FRONT_Y - HINGE_Y,
                LID_DEPTH + HANDLE_BOSS_HEIGHT - HANDLE_MOUNT_EMBED,
            ),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    case_hinge = object_model.get_articulation("case_hinge")

    lower_shell_body = lower_shell.get_visual("lower_shell_body")
    lid_shell_body = lid.get_visual("lid_shell_body")
    handle_body = handle.get_visual("handle_body")
    handle_mount_bridge = lid.get_visual("handle_mount_bridge")

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

    hinge_axis = tuple(getattr(case_hinge, "axis", ()))
    motion_limits = getattr(case_hinge, "motion_limits", None)
    lower_limit = getattr(motion_limits, "lower", None)
    upper_limit = getattr(motion_limits, "upper", None)

    ctx.check(
        "hinge_axis_runs_along_case_length",
        hinge_axis == (1.0, 0.0, 0.0),
        f"Expected hinge axis (1, 0, 0), got {hinge_axis!r}",
    )
    ctx.check(
        "hinge_opens_to_ninety_five_degrees",
        lower_limit == 0.0 and upper_limit is not None and abs(upper_limit - math.radians(95.0)) < 1e-6,
        f"Expected hinge limits [0, 95deg], got lower={lower_limit!r}, upper={upper_limit!r}",
    )

    ctx.expect_contact(
        lid,
        lower_shell,
        elem_a=lid_shell_body,
        elem_b=lower_shell_body,
        name="lid_seats_on_lower_shell",
    )
    ctx.expect_gap(
        lid,
        lower_shell,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=lid_shell_body,
        negative_elem=lower_shell_body,
        name="closed_shell_rims_meet_without_overlap",
    )
    ctx.expect_overlap(
        lid,
        lower_shell,
        axes="xy",
        min_overlap=0.22,
        elem_a=lid_shell_body,
        elem_b=lower_shell_body,
        name="lid_matches_lower_footprint",
    )
    ctx.expect_within(
        lid,
        lower_shell,
        axes="xy",
        margin=0.008,
        inner_elem=lid_shell_body,
        outer_elem=lower_shell_body,
        name="lid_stays_within_lower_outline",
    )
    ctx.expect_contact(
        handle,
        lid,
        name="handle_is_mounted_to_lid",
    )
    ctx.expect_contact(
        handle,
        lid,
        elem_a=handle_body,
        elem_b=handle_mount_bridge,
        name="handle_contacts_mount_bridge",
    )
    ctx.expect_origin_gap(
        handle,
        lower_shell,
        axis="y",
        min_gap=0.11,
        max_gap=0.18,
        name="handle_sits_on_front_edge",
    )

    handle_rest = ctx.part_world_position(handle)
    with ctx.pose({case_hinge: math.radians(95.0)}):
        handle_open = ctx.part_world_position(handle)
        if handle_rest is not None and handle_open is not None:
            ctx.check(
                "lid_lifts_handle_well_above_case_when_open",
                handle_open[2] > handle_rest[2] + 0.12,
                f"Expected open handle z to increase by > 0.12 m, got rest={handle_rest!r}, open={handle_open!r}",
            )
        else:
            ctx.fail(
                "lid_lifts_handle_well_above_case_when_open",
                "Could not measure handle position in one or both poses.",
            )
        ctx.expect_contact(handle, lid, name="handle_stays_attached_when_lid_opens")
        ctx.expect_gap(
            handle,
            lower_shell,
            axis="z",
            min_gap=0.08,
            name="open_lid_clears_lower_shell",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
