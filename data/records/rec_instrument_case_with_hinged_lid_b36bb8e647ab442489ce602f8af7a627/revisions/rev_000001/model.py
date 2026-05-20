from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)


CASE_LENGTH = 1.18
HINGE_Y = 0.255
HINGE_Z = 0.100


def _outline_profile() -> list[tuple[float, float]]:
    """Smooth top-view guitar-case outline in the XY plane."""

    top_controls = [
        (-0.590, 0.000),
        (-0.565, 0.085),
        (-0.500, 0.145),
        (-0.390, 0.132),
        (-0.245, 0.090),
        (-0.030, 0.104),
        (0.110, 0.170),
        (0.270, 0.236),
        (0.450, 0.224),
        (0.565, 0.132),
        (0.590, 0.000),
    ]
    upper = sample_catmull_rom_spline_2d(
        top_controls,
        samples_per_segment=7,
        closed=False,
    )
    # The helper includes both endpoints. Mirror it to make a single closed loop.
    lower = [(x, -y) for x, y in reversed(upper[1:-1])]
    return upper + lower


def _scaled_profile(
    profile: list[tuple[float, float]],
    *,
    scale_x: float = 1.0,
    scale_y: float = 1.0,
) -> list[tuple[float, float]]:
    return [(x * scale_x, y * scale_y) for x, y in profile]


def _half_width_at(x: float) -> float:
    controls = [
        (-0.590, 0.000),
        (-0.565, 0.085),
        (-0.500, 0.145),
        (-0.390, 0.132),
        (-0.245, 0.090),
        (-0.030, 0.104),
        (0.110, 0.170),
        (0.270, 0.236),
        (0.450, 0.224),
        (0.565, 0.132),
        (0.590, 0.000),
    ]
    for (x0, w0), (x1, w1) in zip(controls[:-1], controls[1:]):
        if x0 <= x <= x1:
            t = (x - x0) / (x1 - x0)
            return w0 + (w1 - w0) * t
    return 0.08


def _add_loop(geom: MeshGeometry, points: list[tuple[float, float]], z: float) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y in points]


def _add_shifted_loop(
    geom: MeshGeometry,
    points: list[tuple[float, float]],
    *,
    z: float,
    y_shift: float,
) -> list[int]:
    return [geom.add_vertex(x, y - y_shift, z) for x, y in points]


def _bridge_loops(geom: MeshGeometry, a: list[int], b: list[int]) -> None:
    count = len(a)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(a[i], a[j], b[j])
        geom.add_face(a[i], b[j], b[i])


def _cap_loop(geom: MeshGeometry, loop: list[int], *, reverse: bool = False) -> None:
    # Fan triangulation is sufficient for this smooth, near-convex case outline.
    coords = [geom.vertices[index] for index in loop]
    center = (
        sum(p[0] for p in coords) / len(coords),
        sum(p[1] for p in coords) / len(coords),
        sum(p[2] for p in coords) / len(coords),
    )
    c = geom.add_vertex(*center)
    for i in range(len(loop)):
        j = (i + 1) % len(loop)
        if reverse:
            geom.add_face(c, loop[j], loop[i])
        else:
            geom.add_face(c, loop[i], loop[j])


def _lower_shell_geometry() -> MeshGeometry:
    profile = _outline_profile()
    inner = _scaled_profile(profile, scale_x=0.925, scale_y=0.800)
    geom = MeshGeometry()

    outer_bottom = _add_loop(geom, _scaled_profile(profile, scale_x=0.975, scale_y=0.925), 0.000)
    outer_top = _add_loop(geom, profile, 0.092)
    inner_top = _add_loop(geom, inner, 0.092)
    inner_floor = _add_loop(geom, _scaled_profile(profile, scale_x=0.900, scale_y=0.735), 0.032)

    _bridge_loops(geom, outer_bottom, outer_top)
    _bridge_loops(geom, outer_top, inner_top)
    _bridge_loops(geom, inner_top, inner_floor)
    _cap_loop(geom, outer_bottom, reverse=True)
    _cap_loop(geom, inner_floor, reverse=False)
    return geom


def _lid_shell_geometry() -> MeshGeometry:
    profile = _outline_profile()
    geom = MeshGeometry()

    lower_outer = _add_shifted_loop(geom, profile, z=0.000, y_shift=HINGE_Y)
    upper_outer = _add_shifted_loop(
        geom,
        _scaled_profile(profile, scale_x=0.970, scale_y=0.900),
        z=0.092,
        y_shift=HINGE_Y,
    )
    underside = _add_shifted_loop(
        geom,
        _scaled_profile(profile, scale_x=0.900, scale_y=0.735),
        z=0.012,
        y_shift=HINGE_Y,
    )

    _bridge_loops(geom, lower_outer, upper_outer)
    _cap_loop(geom, upper_outer, reverse=False)
    # A visible underside liner surface makes the upper shell read as hollow when opened.
    _cap_loop(geom, underside, reverse=True)
    _bridge_loops(geom, lower_outer, underside)
    return geom


def _plush_insert_geometry() -> MeshGeometry:
    profile = _scaled_profile(_outline_profile(), scale_x=0.885, scale_y=0.715)
    geom = MeshGeometry()
    lower = _add_loop(geom, profile, 0.032)
    upper = _add_loop(geom, _scaled_profile(profile, scale_x=0.975, scale_y=0.965), 0.038)
    _bridge_loops(geom, lower, upper)
    _cap_loop(geom, upper, reverse=False)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hard_guitar_case")

    black_shell = model.material("black_pebbled_shell", rgba=(0.015, 0.014, 0.013, 1.0))
    seam_black = model.material("black_rubber_seam", rgba=(0.005, 0.005, 0.005, 1.0))
    plush_red = model.material("crimson_plush", rgba=(0.55, 0.025, 0.045, 1.0))
    nickel = model.material("brushed_nickel", rgba=(0.72, 0.70, 0.65, 1.0))
    dark_steel = model.material("dark_hinge_pin", rgba=(0.18, 0.18, 0.17, 1.0))

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        mesh_from_geometry(_lower_shell_geometry(), "lower_shell_body"),
        material=black_shell,
        name="lower_shell_body",
    )
    lower_shell.visual(
        mesh_from_geometry(_plush_insert_geometry(), "lower_plush_cradle"),
        material=plush_red,
        name="lower_plush_cradle",
    )
    # Long dark rubber seam bead on the rear hinge side.
    lower_shell.visual(
        Box((CASE_LENGTH * 0.86, 0.016, 0.018)),
        origin=Origin(xyz=(0.000, 0.243, 0.091)),
        material=seam_black,
        name="rear_seam_bead",
    )
    lower_shell.visual(
        Cylinder(radius=0.0075, length=CASE_LENGTH * 0.90),
        origin=Origin(xyz=(0.000, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_hinge_pin",
    )
    lower_shell.visual(
        Box((CASE_LENGTH * 0.86, 0.018, 0.012)),
        origin=Origin(xyz=(0.000, 0.244, 0.086)),
        material=nickel,
        name="lower_hinge_leaf",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_lid_shell_geometry(), "lid_shell_body"),
        material=black_shell,
        name="lid_shell_body",
    )
    lid.visual(
        Box((CASE_LENGTH * 0.84, 0.016, 0.010)),
        origin=Origin(xyz=(0.000, -0.016, 0.017)),
        material=nickel,
        name="lid_hinge_leaf",
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.35),
    )

    latch_positions = [-0.455, -0.245, 0.020, 0.260, 0.470]
    for index, x in enumerate(latch_positions):
        width = _half_width_at(x)
        pivot_y = -width - 0.018
        pivot_z = 0.094
        # Fixed keeper and bracket on the lower shell.
        lower_shell.visual(
            Box((0.092, 0.012, 0.038)),
            origin=Origin(xyz=(x, -width - 0.006, 0.080)),
            material=nickel,
            name=f"latch_keeper_{index}",
        )
        lower_shell.visual(
            Box((0.022, 0.016, 0.022)),
            origin=Origin(xyz=(x - 0.052, pivot_y, pivot_z)),
            material=nickel,
            name=f"latch_ear_{index}_0",
        )
        lower_shell.visual(
            Box((0.022, 0.016, 0.022)),
            origin=Origin(xyz=(x + 0.052, pivot_y, pivot_z)),
            material=nickel,
            name=f"latch_ear_{index}_1",
        )

        latch = model.part(f"latch_{index}")
        latch.visual(
            Cylinder(radius=0.006, length=0.080),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name="pivot_barrel",
        )
        latch.visual(
            Box((0.020, 0.008, 0.052)),
            origin=Origin(xyz=(0.0, -0.004, -0.026)),
            material=nickel,
            name="pivot_arm",
        )
        latch.visual(
            Box((0.074, 0.007, 0.032)),
            origin=Origin(xyz=(0.0, -0.006, -0.055)),
            material=nickel,
            name="butterfly_plate",
        )
        latch.visual(
            Cylinder(radius=0.017, length=0.008),
            origin=Origin(xyz=(-0.030, -0.006, -0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=nickel,
            name="wing_0",
        )
        latch.visual(
            Cylinder(radius=0.017, length=0.008),
            origin=Origin(xyz=(0.030, -0.006, -0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=nickel,
            name="wing_1",
        )
        latch.visual(
            Box((0.030, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, -0.012, -0.066)),
            material=dark_steel,
            name="finger_tab",
        )

        model.articulation(
            f"latch_pivot_{index}",
            ArticulationType.REVOLUTE,
            parent=lower_shell,
            child=latch,
            origin=Origin(xyz=(x, pivot_y, pivot_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=5.0, lower=0.0, upper=1.05),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    rear_hinge = object_model.get_articulation("rear_hinge")

    ctx.expect_overlap(
        lower_shell,
        lid,
        axes="xy",
        elem_a="lower_shell_body",
        elem_b="lid_shell_body",
        min_overlap=0.35,
        name="clamshell halves share the guitar-case footprint",
    )
    ctx.expect_gap(
        lid,
        lower_shell,
        axis="z",
        positive_elem="lid_shell_body",
        negative_elem="lower_shell_body",
        min_gap=0.003,
        max_gap=0.014,
        name="closed lid sits just above the lower rim",
    )
    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell_body")
    with ctx.pose({rear_hinge: 1.0}):
        ctx.expect_origin_gap(
            lid,
            lower_shell,
            axis="z",
            min_gap=0.0,
            name="lid hinge opens upward from the rear edge",
        )
        raised_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell_body")
    ctx.check(
        "lid upper shell rotates high above the lower cradle",
        closed_lid_aabb is not None
        and raised_lid_aabb is not None
        and raised_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.25,
        details=f"closed={closed_lid_aabb}, raised={raised_lid_aabb}",
    )

    for index in range(5):
        latch = object_model.get_part(f"latch_{index}")
        pivot = object_model.get_articulation(f"latch_pivot_{index}")
        seated_tab_aabb = ctx.part_element_world_aabb(latch, elem="finger_tab")
        ctx.expect_contact(
            latch,
            lower_shell,
            elem_a="pivot_barrel",
            elem_b=f"latch_ear_{index}_0",
            contact_tol=0.010,
            name=f"latch {index} pivot barrel is carried by a keeper ear",
        )
        with ctx.pose({pivot: 0.75}):
            ctx.expect_origin_distance(
                latch,
                lower_shell,
                axes="xy",
                min_dist=0.01,
                name=f"latch {index} has its own revolute pivot",
            )
            lifted_tab_aabb = ctx.part_element_world_aabb(latch, elem="finger_tab")
        ctx.check(
            f"latch {index} butterfly tab swings outward",
            seated_tab_aabb is not None
            and lifted_tab_aabb is not None
            and lifted_tab_aabb[0][1] < seated_tab_aabb[0][1] - 0.010,
            details=f"seated={seated_tab_aabb}, lifted={lifted_tab_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
