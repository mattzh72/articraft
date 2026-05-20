from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 28,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * cos(2.0 * pi * i / segments),
            cy + radius * sin(2.0 * pi * i / segments),
        )
        for i in range(segments)
    ]


def _plate_with_holes_mesh(
    name: str,
    *,
    width_y: float,
    height_z: float,
    thickness_x: float,
    corner_radius: float,
    holes: list[tuple[float, float, float]],
):
    outer = rounded_rect_profile(
        height_z,
        width_y,
        min(corner_radius, height_z * 0.45, width_y * 0.45),
        corner_segments=8,
    )
    hole_profiles = [
        _circle_profile(radius, center=(-z, y), segments=24) for y, z, radius in holes
    ]
    geom = ExtrudeWithHolesGeometry(
        outer,
        hole_profiles,
        height=thickness_x,
        center=True,
    ).rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, name)


def _tube_shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 48,
):
    geom = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -length * 0.5),
            (outer_radius, length * 0.5),
        ],
        [
            (inner_radius, -length * 0.5),
            (inner_radius, length * 0.5),
        ],
        segments=segments,
    ).rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, name)


def _threaded_screw_mesh(
    name: str,
    *,
    root_radius: float,
    crest_radius: float,
    length: float,
    pitch: float,
):
    samples = max(40, int(length / pitch * 18))
    amplitude = crest_radius - root_radius
    profile = []
    for i in range(samples + 1):
        z = -length * 0.5 + length * i / samples
        phase = 2.0 * pi * (z + length * 0.5) / pitch
        radius = root_radius + amplitude * (0.5 - 0.5 * cos(phase))
        profile.append((radius, z))
    geom = LatheGeometry(profile, segments=52).rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="woodworking_front_vise")

    bench_oak = model.material("bench_oak", rgba=(0.58, 0.43, 0.27, 1.0))
    jaw_beech = model.material("jaw_beech", rgba=(0.74, 0.60, 0.40, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.77, 0.79, 0.81, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.35, 0.36, 0.38, 1.0))
    handle_wood = model.material("handle_wood", rgba=(0.66, 0.47, 0.27, 1.0))

    jaw_width = 0.39
    jaw_height = 0.145
    fixed_jaw_thickness = 0.032
    moving_jaw_thickness = 0.035
    bench_width = 0.62
    bench_height = 0.165
    bench_thickness = 0.048
    rear_plate_width = 0.22
    rear_plate_height = 0.115
    rear_plate_thickness = 0.014
    rod_radius = 0.011
    rod_clear_radius = 0.014
    screw_root_radius = 0.014
    screw_crest_radius = 0.018
    screw_clear_radius = 0.0215
    rod_z = 0.043
    close_gap = 0.006
    jaw_travel = 0.22

    fixed_holes = [
        (0.0, rod_z, rod_clear_radius),
        (0.0, -rod_z, rod_clear_radius),
        (0.0, 0.0, screw_clear_radius),
    ]
    moving_holes = [
        (0.0, rod_z, rod_radius * 0.96),
        (0.0, -rod_z, rod_radius * 0.96),
        (0.0, 0.0, screw_clear_radius),
    ]

    fixed_jaw_mesh = _plate_with_holes_mesh(
        "fixed_jaw_face",
        width_y=jaw_width,
        height_z=jaw_height,
        thickness_x=fixed_jaw_thickness,
        corner_radius=0.012,
        holes=fixed_holes,
    )
    bench_front_mesh = _plate_with_holes_mesh(
        "bench_front_apron",
        width_y=bench_width,
        height_z=bench_height,
        thickness_x=bench_thickness,
        corner_radius=0.010,
        holes=fixed_holes,
    )
    rear_plate_mesh = _plate_with_holes_mesh(
        "rear_support_plate",
        width_y=rear_plate_width,
        height_z=rear_plate_height,
        thickness_x=rear_plate_thickness,
        corner_radius=0.010,
        holes=fixed_holes,
    )
    moving_jaw_mesh = _plate_with_holes_mesh(
        "moving_jaw_face",
        width_y=jaw_width,
        height_z=jaw_height + 0.005,
        thickness_x=moving_jaw_thickness,
        corner_radius=0.013,
        holes=moving_holes,
    )
    upper_bushing_mesh = _tube_shell_mesh(
        "upper_guide_bushing_mesh",
        outer_radius=0.0175,
        inner_radius=rod_clear_radius,
        length=0.024,
    )
    lower_bushing_mesh = _tube_shell_mesh(
        "lower_guide_bushing_mesh",
        outer_radius=0.0175,
        inner_radius=rod_clear_radius,
        length=0.024,
    )
    screw_nut_mesh = _tube_shell_mesh(
        "screw_nut_shell_mesh",
        outer_radius=0.031,
        inner_radius=screw_clear_radius,
        length=0.032,
    )
    moving_hub_mesh = _tube_shell_mesh(
        "moving_hub_mesh",
        outer_radius=0.028,
        inner_radius=0.020,
        length=0.018,
    )
    threaded_screw_mesh = _threaded_screw_mesh(
        "threaded_lead_screw",
        root_radius=screw_root_radius,
        crest_radius=screw_crest_radius,
        length=0.31,
        pitch=0.018,
    )

    fixed_support = model.part("fixed_support")
    fixed_support.visual(
        fixed_jaw_mesh,
        origin=Origin(xyz=(-fixed_jaw_thickness * 0.5, 0.0, 0.0)),
        material=jaw_beech,
        name="fixed_jaw_face",
    )
    fixed_support.visual(
        bench_front_mesh,
        origin=Origin(
            xyz=(-fixed_jaw_thickness - bench_thickness * 0.5, 0.0, 0.0)
        ),
        material=bench_oak,
        name="bench_front",
    )
    fixed_support.visual(
        rear_plate_mesh,
        origin=Origin(
            xyz=(
                -fixed_jaw_thickness
                - bench_thickness
                - rear_plate_thickness * 0.5,
                0.0,
                0.0,
            )
        ),
        material=painted_steel,
        name="rear_support_plate",
    )
    bushing_x = -fixed_jaw_thickness - bench_thickness - rear_plate_thickness * 0.5
    fixed_support.visual(
        upper_bushing_mesh,
        origin=Origin(xyz=(bushing_x, 0.0, rod_z)),
        material=dark_steel,
        name="upper_guide_bushing",
    )
    fixed_support.visual(
        lower_bushing_mesh,
        origin=Origin(xyz=(bushing_x, 0.0, -rod_z)),
        material=dark_steel,
        name="lower_guide_bushing",
    )
    fixed_support.visual(
        screw_nut_mesh,
        origin=Origin(xyz=(bushing_x, 0.0, 0.0)),
        material=dark_steel,
        name="screw_nut",
    )
    for idx, (y, z) in enumerate(
        [
            (jaw_width * 0.45, jaw_height * 0.32),
            (-jaw_width * 0.45, jaw_height * 0.32),
            (jaw_width * 0.45, -jaw_height * 0.32),
            (-jaw_width * 0.45, -jaw_height * 0.32),
        ],
        start=1,
    ):
        fixed_support.visual(
            Cylinder(radius=0.011, length=0.008),
            origin=Origin(xyz=(0.004, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"mount_bolt_{idx}",
        )
    fixed_support.inertial = Inertial.from_geometry(
        Box(
            (
                fixed_jaw_thickness + bench_thickness + rear_plate_thickness,
                bench_width,
                bench_height,
            )
        ),
        mass=18.0,
        origin=Origin(
            xyz=(
                -0.5
                * (fixed_jaw_thickness + bench_thickness + rear_plate_thickness),
                0.0,
                0.0,
            )
        ),
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        moving_jaw_mesh,
        origin=Origin(xyz=(moving_jaw_thickness * 0.5, 0.0, 0.0)),
        material=jaw_beech,
        name="moving_jaw_face",
    )
    moving_jaw.visual(
        moving_hub_mesh,
        origin=Origin(xyz=(0.041, 0.0, 0.0)),
        material=painted_steel,
        name="moving_hub_ring",
    )
    rod_rear_extension = 0.33
    rod_front_embed = 0.004
    rod_length = rod_rear_extension + rod_front_embed
    rod_center_x = 0.5 * (rod_front_embed - rod_rear_extension)
    moving_jaw.visual(
        Cylinder(radius=rod_radius, length=rod_length),
        origin=Origin(xyz=(rod_center_x, 0.0, rod_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=bright_steel,
        name="upper_guide_rod",
    )
    moving_jaw.visual(
        Cylinder(radius=rod_radius, length=rod_length),
        origin=Origin(xyz=(rod_center_x, 0.0, -rod_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=bright_steel,
        name="lower_guide_rod",
    )
    moving_jaw.inertial = Inertial.from_geometry(
        Box((jaw_travel + moving_jaw_thickness, jaw_width, jaw_height + 0.01)),
        mass=7.0,
        origin=Origin(
            xyz=(
                -0.5 * jaw_travel + moving_jaw_thickness * 0.5,
                0.0,
                0.0,
            )
        ),
    )

    lead_screw = model.part("lead_screw")
    lead_screw.visual(
        threaded_screw_mesh,
        origin=Origin(xyz=(-0.155, 0.0, 0.0)),
        material=bright_steel,
        name="lead_screw_thread",
    )
    lead_screw.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=bright_steel,
        name="lead_screw_shank",
    )
    lead_screw.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="thrust_collar",
    )
    lead_screw.visual(
        Cylinder(radius=0.031, length=0.022),
        origin=Origin(xyz=(0.071, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="handle_hub",
    )
    lead_screw.visual(
        Cylinder(radius=0.0065, length=0.260),
        origin=Origin(xyz=(0.071, 0.0, 0.0)),
        material=bright_steel,
        name="handle_bar",
    )
    lead_screw.visual(
        Cylinder(radius=0.013, length=0.038),
        origin=Origin(xyz=(0.071, 0.0, 0.126), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_wood,
        name="handle_grip_top",
    )
    lead_screw.visual(
        Cylinder(radius=0.013, length=0.038),
        origin=Origin(xyz=(0.071, 0.0, -0.126), rpy=(0.0, pi / 2.0, 0.0)),
        material=handle_wood,
        name="handle_grip_bottom",
    )
    lead_screw.inertial = Inertial.from_geometry(
        Box((0.39, 0.07, 0.29)),
        mass=2.8,
        origin=Origin(xyz=(-0.10, 0.0, 0.0)),
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=fixed_support,
        child=moving_jaw,
        origin=Origin(xyz=(close_gap, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.10,
            lower=0.0,
            upper=jaw_travel,
        ),
    )
    model.articulation(
        "handle_spin",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=lead_screw,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=10.0),
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

    fixed_support = object_model.get_part("fixed_support")
    moving_jaw = object_model.get_part("moving_jaw")
    lead_screw = object_model.get_part("lead_screw")
    jaw_slide = object_model.get_articulation("jaw_slide")
    handle_spin = object_model.get_articulation("handle_spin")

    with ctx.pose({jaw_slide: 0.0}):
        ctx.expect_gap(
            moving_jaw,
            fixed_support,
            axis="x",
            positive_elem="moving_jaw_face",
            negative_elem="fixed_jaw_face",
            min_gap=0.003,
            max_gap=0.010,
            name="jaw closes to a thin clamping gap",
        )
        ctx.expect_overlap(
            moving_jaw,
            fixed_support,
            axes="yz",
            elem_a="moving_jaw_face",
            elem_b="fixed_jaw_face",
            min_overlap=0.11,
            name="jaw faces stay broad and aligned",
        )
        ctx.expect_overlap(
            moving_jaw,
            fixed_support,
            axes="x",
            elem_a="upper_guide_rod",
            elem_b="upper_guide_bushing",
            min_overlap=0.020,
            name="upper guide rod is engaged in the fixed bushing when closed",
        )
        ctx.expect_overlap(
            moving_jaw,
            fixed_support,
            axes="x",
            elem_a="lower_guide_rod",
            elem_b="lower_guide_bushing",
            min_overlap=0.020,
            name="lower guide rod is engaged in the fixed bushing when closed",
        )
        ctx.expect_contact(
            lead_screw,
            moving_jaw,
            elem_a="thrust_collar",
            elem_b="moving_hub_ring",
            contact_tol=0.0005,
            name="lead screw is captured by the moving jaw hub ring",
        )

    rest_pos = ctx.part_world_position(moving_jaw)
    with ctx.pose({jaw_slide: jaw_slide.motion_limits.upper}):
        open_pos = ctx.part_world_position(moving_jaw)
        ctx.expect_overlap(
            moving_jaw,
            fixed_support,
            axes="x",
            elem_a="upper_guide_rod",
            elem_b="upper_guide_bushing",
            min_overlap=0.020,
            name="upper guide rod keeps retained insertion at full opening",
        )
        ctx.expect_overlap(
            moving_jaw,
            fixed_support,
            axes="x",
            elem_a="lower_guide_rod",
            elem_b="lower_guide_bushing",
            min_overlap=0.020,
            name="lower guide rod keeps retained insertion at full opening",
        )
    ctx.check(
        "moving jaw opens outward along the screw axis",
        rest_pos is not None
        and open_pos is not None
        and open_pos[0] > rest_pos[0] + 0.18,
        details=f"rest_pos={rest_pos}, open_pos={open_pos}",
    )

    with ctx.pose({handle_spin: 0.0}):
        handle_aabb_0 = ctx.part_element_world_aabb(lead_screw, elem="handle_bar")
    with ctx.pose({handle_spin: pi / 2.0}):
        handle_aabb_90 = ctx.part_element_world_aabb(lead_screw, elem="handle_bar")

    def _extent(aabb, axis_index: int) -> float:
        return aabb[1][axis_index] - aabb[0][axis_index]

    handle_rotates = (
        handle_aabb_0 is not None
        and handle_aabb_90 is not None
        and _extent(handle_aabb_0, 2) > _extent(handle_aabb_0, 1) * 3.0
        and _extent(handle_aabb_90, 1) > _extent(handle_aabb_90, 2) * 3.0
    )
    ctx.check(
        "round handle rotates continuously about the lead screw axis",
        handle_rotates,
        details=f"q0={handle_aabb_0}, q90={handle_aabb_90}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
