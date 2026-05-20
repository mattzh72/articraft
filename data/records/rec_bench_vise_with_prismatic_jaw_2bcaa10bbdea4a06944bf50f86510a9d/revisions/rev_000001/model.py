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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    tube_from_spline_points,
)


def _shift_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _perforated_plate_mesh(
    *,
    thickness_x: float,
    width_y: float,
    height_z: float,
    corner_radius: float,
    holes: list[tuple[float, float, float]],
    name: str,
):
    outer_profile = rounded_rect_profile(height_z, width_y, corner_radius)
    hole_profiles = []
    for center_y, center_z, radius in holes:
        hole_profiles.append(
            _shift_profile(
                superellipse_profile(radius * 2.0, radius * 2.0, exponent=2.0, segments=32),
                -center_z,
                center_y,
            )
        )
    plate = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        thickness_x,
        cap=True,
        center=True,
        closed=True,
    )
    plate.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(plate, name)


def _bench_screw_thread_mesh(
    *,
    x_start: float,
    length: float,
    core_radius: float,
    thread_radius: float,
    turns: float,
    name: str,
):
    samples = max(96, int(turns * 40))
    helix_radius = core_radius + thread_radius * 0.55
    points = []
    for i in range(samples + 1):
        t = i / samples
        angle = turns * 2.0 * math.pi * t
        x = x_start + length * t
        points.append(
            (
                x,
                helix_radius * math.cos(angle),
                helix_radius * math.sin(angle),
            )
        )
    thread = tube_from_spline_points(
        points,
        radius=thread_radius,
        samples_per_segment=2,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    return mesh_from_geometry(thread, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="carvers_bench_screw_vise")

    oak = model.material("oak", rgba=(0.67, 0.52, 0.32, 1.0))
    walnut = model.material("walnut", rgba=(0.44, 0.28, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.33, 0.35, 0.38, 1.0))

    bench_block_mesh = _perforated_plate_mesh(
        thickness_x=0.18,
        width_y=0.34,
        height_z=0.28,
        corner_radius=0.022,
        holes=[
            (0.0, 0.0, 0.044),
            (0.0, 0.072, 0.014),
            (0.0, -0.072, 0.014),
        ],
        name="bench_vise_mounting_block",
    )
    jaw_plate_mesh = _perforated_plate_mesh(
        thickness_x=0.045,
        width_y=0.31,
        height_z=0.27,
        corner_radius=0.018,
        holes=[
            (0.0, 0.0, 0.040),
            (0.0, 0.072, 0.0115),
            (0.0, -0.072, 0.0115),
        ],
        name="bench_vise_jaw_plate",
    )
    screw_thread_mesh = _bench_screw_thread_mesh(
        x_start=0.040,
        length=0.205,
        core_radius=0.032,
        thread_radius=0.0044,
        turns=4.8,
        name="bench_screw_thread",
    )

    bench = model.part("bench")
    bench.visual(
        bench_block_mesh,
        material=oak,
        name="mounting_block",
    )
    bench.visual(
        Box((0.52, 0.60, 0.06)),
        origin=Origin(xyz=(0.14, 0.0, 0.165)),
        material=oak,
        name="benchtop",
    )
    bench.visual(
        Box((0.18, 0.34, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        material=oak,
        name="lower_apron",
    )
    bench.inertial = Inertial.from_geometry(
        Box((0.52, 0.60, 0.39)),
        mass=38.0,
        origin=Origin(xyz=(0.14, 0.0, 0.06)),
    )

    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        jaw_plate_mesh,
        material=walnut,
        name="jaw_plate",
    )
    moving_jaw.inertial = Inertial.from_geometry(
        Box((0.045, 0.31, 0.27)),
        mass=3.8,
    )

    left_guide_rod = model.part("upper_guide_rod")
    left_guide_rod.visual(
        Cylinder(radius=0.009, length=0.295),
        origin=Origin(xyz=(0.1025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rod_shaft",
    )
    left_guide_rod.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(-0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rod_collar",
    )
    left_guide_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.305),
        mass=0.35,
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_guide_rod = model.part("lower_guide_rod")
    right_guide_rod.visual(
        Cylinder(radius=0.009, length=0.295),
        origin=Origin(xyz=(0.1025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rod_shaft",
    )
    right_guide_rod.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(-0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rod_collar",
    )
    right_guide_rod.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.305),
        mass=0.35,
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    bench_screw = model.part("bench_screw")
    bench_screw.visual(
        Cylinder(radius=0.032, length=0.385),
        origin=Origin(xyz=(0.1125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=walnut,
        name="screw_shaft",
    )
    bench_screw.visual(
        screw_thread_mesh,
        material=oak,
        name="screw_thread",
    )
    bench_screw.visual(
        Cylinder(radius=0.048, length=0.010),
        origin=Origin(xyz=(-0.0275, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=walnut,
        name="front_collar",
    )
    bench_screw.visual(
        Cylinder(radius=0.062, length=0.030),
        origin=Origin(xyz=(-0.065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=walnut,
        name="handwheel",
    )
    bench_screw.visual(
        Cylinder(radius=0.007, length=0.185),
        origin=Origin(xyz=(-0.065, 0.0, 0.0)),
        material=steel,
        name="handle_bar",
    )
    bench_screw.inertial = Inertial.from_geometry(
        Cylinder(radius=0.062, length=0.345),
        mass=2.8,
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=bench,
        child=moving_jaw,
        origin=Origin(xyz=(-0.1125, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.14,
            lower=-0.10,
            upper=0.0,
        ),
    )
    model.articulation(
        "jaw_to_upper_rod",
        ArticulationType.FIXED,
        parent=moving_jaw,
        child=left_guide_rod,
        origin=Origin(xyz=(0.0225, 0.0, 0.072)),
    )
    model.articulation(
        "jaw_to_lower_rod",
        ArticulationType.FIXED,
        parent=moving_jaw,
        child=right_guide_rod,
        origin=Origin(xyz=(0.0225, 0.0, -0.072)),
    )
    model.articulation(
        "jaw_to_bench_screw",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=bench_screw,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=7.5),
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

    bench = object_model.get_part("bench")
    moving_jaw = object_model.get_part("moving_jaw")
    upper_rod = object_model.get_part("upper_guide_rod")
    lower_rod = object_model.get_part("lower_guide_rod")
    bench_screw = object_model.get_part("bench_screw")
    jaw_slide = object_model.get_articulation("jaw_slide")
    screw_spin = object_model.get_articulation("jaw_to_bench_screw")
    slide_open = jaw_slide.motion_limits.lower or 0.0
    slide_closed = jaw_slide.motion_limits.upper or 0.0

    with ctx.pose({jaw_slide: slide_open, screw_spin: 0.0}):
        jaw_open_pos = ctx.part_world_position(moving_jaw)
        screw_open_pos = ctx.part_world_position(bench_screw)
        ctx.expect_overlap(
            upper_rod,
            bench,
            axes="x",
            elem_a="rod_shaft",
            elem_b="mounting_block",
            min_overlap=0.14,
            name="upper guide rod remains inserted when the jaw is open",
        )
        ctx.expect_overlap(
            lower_rod,
            bench,
            axes="x",
            elem_a="rod_shaft",
            elem_b="mounting_block",
            min_overlap=0.14,
            name="lower guide rod remains inserted when the jaw is open",
        )
        ctx.expect_overlap(
            bench_screw,
            bench,
            axes="x",
            elem_a="screw_shaft",
            elem_b="mounting_block",
            min_overlap=0.17,
            name="bench screw remains engaged in the fixed block when open",
        )
        ctx.expect_contact(
            upper_rod,
            moving_jaw,
            elem_a="rod_collar",
            elem_b="jaw_plate",
            contact_tol=1e-4,
            name="upper guide rod collar mounts to the jaw plate",
        )
        ctx.expect_contact(
            lower_rod,
            moving_jaw,
            elem_a="rod_collar",
            elem_b="jaw_plate",
            contact_tol=1e-4,
            name="lower guide rod collar mounts to the jaw plate",
        )
        ctx.expect_contact(
            bench_screw,
            moving_jaw,
            elem_a="front_collar",
            elem_b="jaw_plate",
            contact_tol=1e-4,
            name="bench screw is retained against the moving jaw plate",
        )

    with ctx.pose({jaw_slide: slide_closed, screw_spin: 0.0}):
        jaw_closed_pos = ctx.part_world_position(moving_jaw)
        screw_closed_pos = ctx.part_world_position(bench_screw)
        handle_rest = ctx.part_element_world_aabb(bench_screw, elem="handle_bar")
        ctx.expect_gap(
            bench,
            moving_jaw,
            axis="x",
            min_gap=0.0,
            max_gap=0.004,
            positive_elem="mounting_block",
            negative_elem="jaw_plate",
            name="moving jaw closes up close to the bench without penetrating it",
        )
        ctx.expect_overlap(
            moving_jaw,
            bench,
            axes="yz",
            elem_a="jaw_plate",
            elem_b="mounting_block",
            min_overlap=0.24,
            name="moving jaw stays centered on the vise block when closed",
        )
        ctx.expect_overlap(
            upper_rod,
            bench,
            axes="x",
            elem_a="rod_shaft",
            elem_b="mounting_block",
            min_overlap=0.17,
            name="upper guide rod remains supported at the closed pose",
        )
        ctx.expect_overlap(
            lower_rod,
            bench,
            axes="x",
            elem_a="rod_shaft",
            elem_b="mounting_block",
            min_overlap=0.17,
            name="lower guide rod remains supported at the closed pose",
        )
        ctx.expect_overlap(
            bench_screw,
            bench,
            axes="yz",
            elem_a="screw_shaft",
            elem_b="mounting_block",
            min_overlap=0.06,
            name="bench screw stays centered in the fixed screw bore",
        )

    with ctx.pose({jaw_slide: slide_closed, screw_spin: math.pi / 2.0}):
        handle_quarter_turn = ctx.part_element_world_aabb(bench_screw, elem="handle_bar")

    jaw_motion_ok = (
        jaw_open_pos is not None
        and jaw_closed_pos is not None
        and jaw_closed_pos[0] > jaw_open_pos[0] + 0.09
    )
    ctx.check(
        "jaw slide moves toward the bench along the screw axis",
        jaw_motion_ok,
        details=f"open={jaw_open_pos}, closed={jaw_closed_pos}",
    )

    screw_tracks_jaw_ok = (
        jaw_open_pos is not None
        and jaw_closed_pos is not None
        and screw_open_pos is not None
        and screw_closed_pos is not None
        and abs((screw_closed_pos[0] - screw_open_pos[0]) - (jaw_closed_pos[0] - jaw_open_pos[0]))
        < 1e-6
    )
    ctx.check(
        "bench screw translates with the moving jaw while rotating in place on that jaw",
        screw_tracks_jaw_ok,
        details=f"jaw_open={jaw_open_pos}, jaw_closed={jaw_closed_pos}, screw_open={screw_open_pos}, screw_closed={screw_closed_pos}",
    )

    handle_rotation_ok = False
    if handle_rest is not None and handle_quarter_turn is not None:
        rest_y = handle_rest[1][1] - handle_rest[0][1]
        rest_z = handle_rest[1][2] - handle_rest[0][2]
        quarter_y = handle_quarter_turn[1][1] - handle_quarter_turn[0][1]
        quarter_z = handle_quarter_turn[1][2] - handle_quarter_turn[0][2]
        handle_rotation_ok = (
            rest_z > 0.16
            and rest_y < 0.03
            and quarter_y > 0.16
            and quarter_z < 0.03
        )
    ctx.check(
        "bench screw rotates continuously about its own axis",
        handle_rotation_ok,
        details=f"rest={handle_rest}, quarter_turn={handle_quarter_turn}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
