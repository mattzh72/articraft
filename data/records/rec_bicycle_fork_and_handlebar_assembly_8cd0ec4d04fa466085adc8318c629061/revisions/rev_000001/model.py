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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _hollow_cylinder_mesh(
    outer_radius: float,
    inner_radius: float,
    *,
    z0: float,
    z1: float,
    segments: int = 48,
) -> MeshGeometry:
    geom = MeshGeometry()
    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []

    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        c = math.cos(angle)
        s = math.sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * c, outer_radius * s, z0))
        outer_top.append(geom.add_vertex(outer_radius * c, outer_radius * s, z1))
        inner_bottom.append(geom.add_vertex(inner_radius * c, inner_radius * s, z0))
        inner_top.append(geom.add_vertex(inner_radius * c, inner_radius * s, z1))

    for index in range(segments):
        nxt = (index + 1) % segments
        _add_quad(
            geom,
            outer_bottom[index],
            outer_bottom[nxt],
            outer_top[nxt],
            outer_top[index],
        )
        _add_quad(
            geom,
            inner_bottom[index],
            inner_top[index],
            inner_top[nxt],
            inner_bottom[nxt],
        )
        _add_quad(
            geom,
            outer_top[index],
            outer_top[nxt],
            inner_top[nxt],
            inner_top[index],
        )
        _add_quad(
            geom,
            outer_bottom[nxt],
            outer_bottom[index],
            inner_bottom[index],
            inner_bottom[nxt],
        )

    return geom


def _xy_section(
    *,
    center_x: float,
    center_y: float,
    z: float,
    width: float,
    depth: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + px, center_y + py, z)
        for px, py in rounded_rect_profile(width, depth, radius, corner_segments=6)
    ]


def _xz_section(
    *,
    y: float,
    center_z: float,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (px, y, center_z + pz)
        for px, pz in rounded_rect_profile(width, height, radius, corner_segments=6)
    ]


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="track_fork_with_quill_and_bars")

    carbon_black = model.material("carbon_black", rgba=(0.08, 0.09, 0.10, 1.0))
    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    alloy_silver = model.material("alloy_silver", rgba=(0.77, 0.79, 0.81, 1.0))
    polished_alloy = model.material("polished_alloy", rgba=(0.87, 0.88, 0.89, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.27, 0.28, 0.30, 1.0))

    fork = model.part("fork")
    fork.inertial = Inertial.from_geometry(
        Box((0.14, 0.12, 0.62)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.02, -0.31)),
    )

    steerer_shell = mesh_from_geometry(
        _hollow_cylinder_mesh(
            outer_radius=0.0143,
            inner_radius=0.0121,
            z0=-0.220,
            z1=0.0,
            segments=56,
        ),
        "fork_steerer_shell",
    )
    fork.visual(steerer_shell, material=carbon_black, name="steerer_shell")

    crown_mesh = mesh_from_geometry(
        section_loft(
            [
                _xy_section(
                    center_x=0.0,
                    center_y=0.0,
                    z=-0.204,
                    width=0.040,
                    depth=0.030,
                    radius=0.009,
                ),
                _xy_section(
                    center_x=0.0,
                    center_y=0.002,
                    z=-0.226,
                    width=0.094,
                    depth=0.054,
                    radius=0.013,
                ),
                _xy_section(
                    center_x=0.0,
                    center_y=0.008,
                    z=-0.248,
                    width=0.118,
                    depth=0.044,
                    radius=0.012,
                ),
            ]
        ),
        "fork_crown_body",
    )
    fork.visual(crown_mesh, material=carbon_black, name="fork_crown")

    left_blade_mesh = mesh_from_geometry(
        section_loft(
            [
                _xy_section(
                    center_x=-0.043,
                    center_y=0.002,
                    z=-0.236,
                    width=0.028,
                    depth=0.040,
                    radius=0.007,
                ),
                _xy_section(
                    center_x=-0.045,
                    center_y=0.012,
                    z=-0.320,
                    width=0.024,
                    depth=0.033,
                    radius=0.006,
                ),
                _xy_section(
                    center_x=-0.047,
                    center_y=0.026,
                    z=-0.430,
                    width=0.019,
                    depth=0.026,
                    radius=0.005,
                ),
                _xy_section(
                    center_x=-0.049,
                    center_y=0.040,
                    z=-0.525,
                    width=0.016,
                    depth=0.021,
                    radius=0.004,
                ),
                _xy_section(
                    center_x=-0.050,
                    center_y=0.048,
                    z=-0.592,
                    width=0.014,
                    depth=0.017,
                    radius=0.0035,
                ),
            ]
        ),
        "left_fork_blade",
    )
    right_blade_mesh = mesh_from_geometry(
        section_loft(
            [
                _xy_section(
                    center_x=0.043,
                    center_y=0.002,
                    z=-0.236,
                    width=0.028,
                    depth=0.040,
                    radius=0.007,
                ),
                _xy_section(
                    center_x=0.045,
                    center_y=0.012,
                    z=-0.320,
                    width=0.024,
                    depth=0.033,
                    radius=0.006,
                ),
                _xy_section(
                    center_x=0.047,
                    center_y=0.026,
                    z=-0.430,
                    width=0.019,
                    depth=0.026,
                    radius=0.005,
                ),
                _xy_section(
                    center_x=0.049,
                    center_y=0.040,
                    z=-0.525,
                    width=0.016,
                    depth=0.021,
                    radius=0.004,
                ),
                _xy_section(
                    center_x=0.050,
                    center_y=0.048,
                    z=-0.592,
                    width=0.014,
                    depth=0.017,
                    radius=0.0035,
                ),
            ]
        ),
        "right_fork_blade",
    )
    fork.visual(left_blade_mesh, material=carbon_black, name="left_blade")
    fork.visual(right_blade_mesh, material=carbon_black, name="right_blade")

    fork.visual(
        Box((0.020, 0.010, 0.038)),
        origin=Origin(xyz=(-0.050, 0.049, -0.608)),
        material=dark_steel,
        name="left_dropout",
    )
    fork.visual(
        Box((0.020, 0.010, 0.038)),
        origin=Origin(xyz=(0.050, 0.049, -0.608)),
        material=dark_steel,
        name="right_dropout",
    )
    fork.visual(
        Cylinder(radius=0.0175, length=0.012),
        origin=Origin(xyz=(0.0, 0.004, -0.213)),
        material=dark_steel,
        name="crown_race_seat",
    )

    quill_stem = model.part("quill_stem")
    quill_stem.inertial = Inertial.from_geometry(
        Box((0.060, 0.140, 0.230)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.056, -0.070)),
    )
    quill_stem.visual(
        Cylinder(radius=0.0111, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=alloy_silver,
        name="quill_shaft",
    )
    quill_stem.visual(
        Box((0.020, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.182)),
        material=dark_steel,
        name="expander_wedge",
    )
    quill_stem.visual(
        Cylinder(radius=0.0141, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=alloy_silver,
        name="stem_collar",
    )

    stem_body_mesh = mesh_from_geometry(
        section_loft(
            [
                _xz_section(
                    y=-0.008,
                    center_z=0.018,
                    width=0.032,
                    height=0.032,
                    radius=0.007,
                ),
                _xz_section(
                    y=0.024,
                    center_z=0.022,
                    width=0.030,
                    height=0.028,
                    radius=0.006,
                ),
                _xz_section(
                    y=0.068,
                    center_z=0.026,
                    width=0.025,
                    height=0.021,
                    radius=0.005,
                ),
                _xz_section(
                    y=0.086,
                    center_z=0.028,
                    width=0.031,
                    height=0.034,
                    radius=0.007,
                ),
            ]
        ),
        "quill_stem_body",
    )
    quill_stem.visual(stem_body_mesh, material=alloy_silver, name="stem_body")

    clamp_shell_mesh = mesh_from_geometry(
        _hollow_cylinder_mesh(
            outer_radius=0.0265,
            inner_radius=0.0142,
            z0=-0.022,
            z1=0.022,
            segments=48,
        ).rotate_y(math.pi / 2.0),
        "handlebar_clamp_shell",
    )
    quill_stem.visual(
        clamp_shell_mesh,
        origin=Origin(xyz=(0.0, 0.108, 0.028)),
        material=alloy_silver,
        name="handlebar_clamp_shell",
    )
    quill_stem.visual(
        Cylinder(radius=0.004, length=0.030),
        origin=Origin(xyz=(-0.010, 0.128, 0.043), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="upper_clamp_bolt",
    )
    quill_stem.visual(
        Cylinder(radius=0.004, length=0.030),
        origin=Origin(xyz=(0.010, 0.128, 0.013), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lower_clamp_bolt",
    )

    handlebar = model.part("handlebar")
    handlebar.inertial = Inertial.from_geometry(
        Box((0.410, 0.190, 0.190)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.045, -0.060)),
    )

    right_side = [
        (0.000, 0.000, 0.000),
        (0.055, 0.000, 0.000),
        (0.082, 0.018, 0.004),
        (0.108, 0.052, 0.010),
        (0.132, 0.084, 0.001),
        (0.151, 0.092, -0.040),
        (0.172, 0.061, -0.110),
        (0.191, 0.028, -0.157),
        (0.202, 0.014, -0.172),
    ]
    bar_points = list(reversed(_mirror_x(right_side[1:]))) + right_side

    handlebar.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                bar_points,
                radius=0.0119,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
            "track_handlebar_tube",
        ),
        material=polished_alloy,
        name="bar_tube",
    )
    handlebar.visual(
        Cylinder(radius=0.0128, length=0.115),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_alloy,
        name="center_section",
    )

    left_grip_points = [
        (-0.132, 0.084, 0.001),
        (-0.151, 0.092, -0.040),
        (-0.172, 0.061, -0.110),
        (-0.191, 0.028, -0.157),
        (-0.202, 0.014, -0.172),
    ]
    right_grip_points = _mirror_x(left_grip_points)
    handlebar.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                left_grip_points,
                radius=0.0133,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
            "left_drop_grip",
        ),
        material=matte_black,
        name="left_drop_grip",
    )
    handlebar.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                right_grip_points,
                radius=0.0133,
                samples_per_segment=14,
                radial_segments=18,
                cap_ends=True,
            ),
            "right_drop_grip",
        ),
        material=matte_black,
        name="right_drop_grip",
    )

    model.articulation(
        "fork_to_quill_stem",
        ArticulationType.PRISMATIC,
        parent=fork,
        child=quill_stem,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.12,
            lower=0.0,
            upper=0.055,
        ),
    )
    model.articulation(
        "stem_to_handlebar",
        ArticulationType.REVOLUTE,
        parent=quill_stem,
        child=handlebar,
        origin=Origin(xyz=(0.0, 0.108, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.45,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    fork = object_model.get_part("fork")
    quill_stem = object_model.get_part("quill_stem")
    handlebar = object_model.get_part("handlebar")
    stem_slide = object_model.get_articulation("fork_to_quill_stem")
    bar_roll = object_model.get_articulation("stem_to_handlebar")

    steerer_shell = fork.get_visual("steerer_shell")
    quill_shaft = quill_stem.get_visual("quill_shaft")
    clamp_shell = quill_stem.get_visual("handlebar_clamp_shell")
    center_section = handlebar.get_visual("center_section")
    right_drop_grip = handlebar.get_visual("right_drop_grip")

    stem_upper = stem_slide.motion_limits.upper or 0.0
    bar_upper = bar_roll.motion_limits.upper or 0.0

    ctx.expect_within(
        quill_stem,
        fork,
        axes="xy",
        inner_elem=quill_shaft,
        outer_elem=steerer_shell,
        margin=0.0015,
        name="quill shaft stays centered inside the steerer at rest",
    )
    ctx.expect_overlap(
        quill_stem,
        fork,
        axes="z",
        elem_a=quill_shaft,
        elem_b=steerer_shell,
        min_overlap=0.180,
        name="quill shaft has deep insertion at rest",
    )
    ctx.expect_overlap(
        handlebar,
        quill_stem,
        axes="x",
        elem_a=center_section,
        elem_b=clamp_shell,
        min_overlap=0.040,
        name="handlebar center section spans the stem clamp",
    )

    rest_stem_pos = ctx.part_world_position(quill_stem)
    with ctx.pose({stem_slide: stem_upper}):
        ctx.expect_within(
            quill_stem,
            fork,
            axes="xy",
            inner_elem=quill_shaft,
            outer_elem=steerer_shell,
            margin=0.0015,
            name="raised quill shaft stays centered inside the steerer",
        )
        ctx.expect_overlap(
            quill_stem,
            fork,
            axes="z",
            elem_a=quill_shaft,
            elem_b=steerer_shell,
            min_overlap=0.125,
            name="raised quill shaft retains insertion in the steerer",
        )
        raised_stem_pos = ctx.part_world_position(quill_stem)

    ctx.check(
        "quill stem raises upward with positive travel",
        rest_stem_pos is not None
        and raised_stem_pos is not None
        and raised_stem_pos[2] > rest_stem_pos[2] + 0.045,
        details=f"rest={rest_stem_pos}, raised={raised_stem_pos}",
    )

    def _aabb_mid_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    rest_drop_aabb = ctx.part_element_world_aabb(handlebar, elem=right_drop_grip)
    with ctx.pose({bar_roll: bar_upper}):
        rolled_drop_aabb = ctx.part_element_world_aabb(handlebar, elem=right_drop_grip)

    rest_mid_z = _aabb_mid_z(rest_drop_aabb)
    rolled_mid_z = _aabb_mid_z(rolled_drop_aabb)
    ctx.check(
        "positive handlebar angle lifts the drops",
        rest_mid_z is not None and rolled_mid_z is not None and rolled_mid_z > rest_mid_z + 0.015,
        details=f"rest_mid_z={rest_mid_z}, rolled_mid_z={rolled_mid_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
