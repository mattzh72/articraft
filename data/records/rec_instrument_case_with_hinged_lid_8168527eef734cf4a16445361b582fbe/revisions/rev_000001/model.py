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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)


OUTER_FRONT_CONTROL = [
    (-0.52, 0.082),
    (-0.46, 0.138),
    (-0.39, 0.180),
    (-0.29, 0.192),
    (-0.18, 0.153),
    (-0.06, 0.118),
    (0.10, 0.132),
    (0.24, 0.116),
    (0.36, 0.082),
    (0.46, 0.066),
    (0.56, 0.072),
]
OUTER_BACK_CONTROL = [
    (0.56, -0.074),
    (0.46, -0.078),
    (0.36, -0.090),
    (0.24, -0.098),
    (0.10, -0.106),
    (-0.06, -0.115),
    (-0.18, -0.128),
    (-0.30, -0.150),
    (-0.40, -0.160),
    (-0.48, -0.134),
    (-0.52, -0.094),
]
INNER_FRONT_CONTROL = [
    (-0.48, 0.064),
    (-0.42, 0.104),
    (-0.34, 0.143),
    (-0.24, 0.154),
    (-0.14, 0.124),
    (-0.02, 0.092),
    (0.11, 0.102),
    (0.22, 0.093),
    (0.34, 0.063),
    (0.44, 0.050),
    (0.52, 0.054),
]
INNER_BACK_CONTROL = [
    (0.52, -0.062),
    (0.44, -0.066),
    (0.34, -0.074),
    (0.22, -0.082),
    (0.11, -0.088),
    (-0.02, -0.094),
    (-0.14, -0.102),
    (-0.26, -0.116),
    (-0.36, -0.126),
    (-0.44, -0.108),
    (-0.48, -0.078),
]
LATCH_X_POSITIONS = (-0.34, -0.08, 0.16, 0.36)

SEAM_Z = 0.064
LOWER_FLOOR_THICKNESS = 0.006
LOWER_PAD_THICKNESS = 0.008
LID_WALL_HEIGHT = 0.034
LID_TOP_THICKNESS = 0.005
LID_CROWN_THICKNESS = 0.010
HINGE_Y = -0.082
LATCH_PIVOT_Z = 0.044


def _sample_curve(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return sample_catmull_rom_spline_2d(points, samples_per_segment=8, closed=False)


def _closed_profile(
    front_control: list[tuple[float, float]],
    back_control: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    return _sample_curve(front_control) + _sample_curve(back_control)


def _profile_center(profile: list[tuple[float, float]]) -> tuple[float, float]:
    sx = sum(x for x, _ in profile)
    sy = sum(y for _, y in profile)
    count = float(len(profile))
    return (sx / count, sy / count)


def _scale_profile(
    profile: list[tuple[float, float]],
    sx: float,
    sy: float,
) -> list[tuple[float, float]]:
    cx, cy = _profile_center(profile)
    return [((cx + (x - cx) * sx), (cy + (y - cy) * sy)) for x, y in profile]


def _interpolate_y(x_value: float, control: list[tuple[float, float]]) -> float:
    if x_value <= control[0][0]:
        return control[0][1]
    if x_value >= control[-1][0]:
        return control[-1][1]
    for (x0, y0), (x1, y1) in zip(control, control[1:]):
        if x0 <= x_value <= x1:
            t = (x_value - x0) / (x1 - x0)
            return y0 + (y1 - y0) * t
    return control[-1][1]


def _build_lower_shell_mesh(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
):
    wall_ring = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        SEAM_Z,
        cap=True,
        center=True,
    ).translate(0.0, 0.0, SEAM_Z * 0.5)
    cavity_floor = ExtrudeGeometry(
        inner_profile,
        LOWER_FLOOR_THICKNESS,
        cap=True,
        center=True,
    ).translate(0.0, 0.0, LOWER_FLOOR_THICKNESS * 0.5)
    lower_shell = wall_ring.copy()
    lower_shell.merge(cavity_floor)
    return lower_shell


def _build_lid_shell_mesh(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
):
    crown_profile = _scale_profile(inner_profile, 0.91, 0.86)
    wall_ring = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        LID_WALL_HEIGHT,
        cap=True,
        center=True,
    ).translate(0.0, 0.0, LID_WALL_HEIGHT * 0.5)
    top_panel = ExtrudeGeometry(
        inner_profile,
        LID_TOP_THICKNESS,
        cap=True,
        center=True,
    ).translate(0.0, 0.0, LID_WALL_HEIGHT + (LID_TOP_THICKNESS * 0.5))
    crown = ExtrudeGeometry(
        crown_profile,
        LID_CROWN_THICKNESS,
        cap=True,
        center=True,
    ).translate(
        0.0,
        0.0,
        LID_WALL_HEIGHT + LID_TOP_THICKNESS + (LID_CROWN_THICKNESS * 0.5),
    )
    lid_shell = wall_ring.copy()
    lid_shell.merge(top_panel)
    lid_shell.merge(crown)
    return lid_shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hard_guitar_case")

    shell_black = model.material("shell_black", rgba=(0.10, 0.10, 0.11, 1.0))
    plush_wine = model.material("plush_wine", rgba=(0.42, 0.09, 0.15, 1.0))
    hardware = model.material("hardware", rgba=(0.72, 0.74, 0.76, 1.0))

    outer_profile = _closed_profile(OUTER_FRONT_CONTROL, OUTER_BACK_CONTROL)
    inner_profile = _closed_profile(INNER_FRONT_CONTROL, INNER_BACK_CONTROL)
    cradle_profile = _scale_profile(inner_profile, 0.975, 0.94)
    lid_lining_profile = _scale_profile(inner_profile, 0.965, 0.92)

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        mesh_from_geometry(_build_lower_shell_mesh(outer_profile, inner_profile), "lower_shell_body"),
        material=shell_black,
        name="lower_shell_body",
    )
    lower_shell.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                cradle_profile,
                LOWER_PAD_THICKNESS,
                cap=True,
                center=True,
            ).translate(0.0, 0.0, LOWER_FLOOR_THICKNESS + (LOWER_PAD_THICKNESS * 0.5)),
            "lower_shell_cradle",
        ),
        material=plush_wine,
        name="cradle_pad",
    )
    lower_shell.inertial = Inertial.from_geometry(
        Box((1.08, 0.39, SEAM_Z)),
        mass=3.8,
        origin=Origin(xyz=(0.02, 0.0, SEAM_Z * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_shell_mesh(outer_profile, inner_profile), "lid_shell_body"),
        origin=Origin(xyz=(0.0, -HINGE_Y, 0.0)),
        material=shell_black,
        name="lid_shell_body",
    )
    lid.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                lid_lining_profile,
                0.004,
                cap=True,
                center=True,
            ).translate(0.0, 0.0, LID_WALL_HEIGHT - 0.002),
            "lid_lining",
        ),
        origin=Origin(xyz=(0.0, -HINGE_Y, 0.0)),
        material=plush_wine,
        name="lid_lining",
    )
    lid.inertial = Inertial.from_geometry(
        Box((1.08, 0.39, 0.055)),
        mass=2.6,
        origin=Origin(xyz=(0.02, -HINGE_Y, 0.0275)),
    )

    model.articulation(
        "lower_shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, SEAM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    for index, x_pos in enumerate(LATCH_X_POSITIONS, start=1):
        outer_front_y = _interpolate_y(x_pos, OUTER_FRONT_CONTROL)
        mount_center_y = outer_front_y + 0.012
        strike_center_y = outer_front_y + 0.012
        latch_pivot_y = outer_front_y + 0.024

        lower_shell.visual(
            Box((0.072, 0.022, 0.010)),
            origin=Origin(xyz=(x_pos, mount_center_y, 0.035)),
            material=hardware,
            name=f"latch_mount_{index}",
        )
        lid.visual(
            Box((0.040, 0.036, 0.008)),
            origin=Origin(
                xyz=(x_pos, strike_center_y - HINGE_Y, 0.014),
            ),
            material=hardware,
            name=f"strike_{index}",
        )

        latch = model.part(f"latch_{index}")
        latch.visual(
            Cylinder(radius=0.004, length=0.046),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=hardware,
            name="pivot_bar",
        )
        latch.visual(
            Box((0.046, 0.012, 0.014)),
            origin=Origin(xyz=(0.0, 0.002, 0.007)),
            material=hardware,
            name="pivot_arm",
        )
        latch.visual(
            Box((0.060, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, 0.004, 0.019)),
            material=hardware,
            name="butterfly_wing",
        )
        latch.visual(
            Box((0.020, 0.008, 0.008)),
            origin=Origin(xyz=(0.0, 0.004, 0.026)),
            material=hardware,
            name="catch",
        )
        latch.inertial = Inertial.from_geometry(
            Box((0.060, 0.016, 0.034)),
            mass=0.08,
            origin=Origin(xyz=(0.0, 0.003, 0.017)),
        )

        model.articulation(
            f"lower_shell_to_latch_{index}",
            ArticulationType.REVOLUTE,
            parent=lower_shell,
            child=latch,
            origin=Origin(xyz=(x_pos, latch_pivot_y, LATCH_PIVOT_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.8,
                velocity=3.0,
                lower=-1.55,
                upper=0.10,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lower_shell_to_lid")
    latch_parts = [object_model.get_part(f"latch_{index}") for index in range(1, 5)]
    latch_joints = [
        object_model.get_articulation(f"lower_shell_to_latch_{index}")
        for index in range(1, 5)
    ]
    latch_mounts = [lower_shell.get_visual(f"latch_mount_{index}") for index in range(1, 5)]
    strike_plates = [lid.get_visual(f"strike_{index}") for index in range(1, 5)]

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

    lid_axis = tuple(round(value, 6) for value in lid_hinge.axis)
    lid_limits = lid_hinge.motion_limits
    ctx.check(
        "lid hinge axis follows case length",
        lid_axis == (1.0, 0.0, 0.0),
        f"expected lid hinge axis (1, 0, 0), got {lid_hinge.axis}",
    )
    ctx.check(
        "lid hinge opens like a clamshell",
        lid_limits is not None
        and lid_limits.lower is not None
        and lid_limits.upper is not None
        and abs(lid_limits.lower) < 1e-6
        and lid_limits.upper >= math.radians(100.0),
        f"unexpected lid motion limits: {lid_limits}",
    )

    for index, latch_joint in enumerate(latch_joints, start=1):
        latch_axis = tuple(round(value, 6) for value in latch_joint.axis)
        latch_limits = latch_joint.motion_limits
        ctx.check(
            f"latch_{index} pivots on a local arm axis",
            latch_axis == (1.0, 0.0, 0.0),
            f"expected x-axis latch pivot, got {latch_joint.axis}",
        )
        ctx.check(
            f"latch_{index} has realistic swing limits",
            latch_limits is not None
            and latch_limits.lower is not None
            and latch_limits.upper is not None
            and latch_limits.lower <= -1.3
            and latch_limits.upper >= 0.0,
            f"unexpected latch limits: {latch_limits}",
        )

    ctx.expect_contact(
        lid,
        lower_shell,
        contact_tol=0.0015,
        name="lid closes onto lower shell rim",
    )
    ctx.expect_overlap(
        lid,
        lower_shell,
        axes="xy",
        min_overlap=0.25,
        name="lid planform matches lower shell",
    )

    for index, (latch, mount, strike) in enumerate(
        zip(latch_parts, latch_mounts, strike_plates),
        start=1,
    ):
        ctx.expect_contact(
            latch,
            lower_shell,
            elem_b=mount,
            contact_tol=0.0015,
            name=f"latch_{index} is mounted on the lower shell",
        )
        ctx.expect_contact(
            latch,
            lid,
            elem_b=strike,
            contact_tol=0.0015,
            name=f"latch_{index} reaches its strike plate when closed",
        )

    latch_positions = []
    for latch in latch_parts:
        position = ctx.part_world_position(latch)
        assert position is not None
        latch_positions.append(position)
    xs = [position[0] for position in latch_positions]
    ys = [position[1] for position in latch_positions]
    ctx.check(
        "latches are spaced along the front edge",
        all(left < right for left, right in zip(xs, xs[1:]))
        and (xs[-1] - xs[0]) > 0.60
        and min(ys) > 0.03,
        f"unexpected latch layout: {latch_positions}",
    )

    with ctx.pose({lid_hinge: math.radians(100.0)}):
        ctx.expect_gap(
            lid,
            lower_shell,
            axis="z",
            min_gap=0.10,
            positive_elem=strike_plates[1],
            name="open lid lifts the front edge clear of the base",
        )

    for index, (latch_joint, latch, strike) in enumerate(
        zip(latch_joints, latch_parts, strike_plates),
        start=1,
    ):
        with ctx.pose({latch_joint: -1.35}):
            ctx.expect_gap(
                lid,
                latch,
                axis="z",
                min_gap=0.010,
                positive_elem=strike,
                name=f"latch_{index} swings down away from its strike plate",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
