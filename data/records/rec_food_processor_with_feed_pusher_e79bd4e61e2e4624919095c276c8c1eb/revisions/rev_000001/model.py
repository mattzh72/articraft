from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _offset_profile(profile, dx=0.0, dy=0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(radius: float, *, segments: int = 48, dx: float = 0.0, dy: float = 0.0):
    return [
        (
            dx + radius * math.cos(2.0 * math.pi * i / segments),
            dy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _hollow_tube_mesh(outer_profile, inner_profile, z0: float, z1: float) -> MeshGeometry:
    """Open-ended tube shell from matching 2-D outer/inner loops."""
    geom = MeshGeometry()

    def add_loop(profile, z):
        return [geom.add_vertex(x, y, z) for x, y in profile]

    outer_bottom = add_loop(outer_profile, z0)
    outer_top = add_loop(outer_profile, z1)
    inner_bottom = add_loop(inner_profile, z0)
    inner_top = add_loop(inner_profile, z1)
    n = len(outer_profile)
    if n != len(inner_profile):
        raise ValueError("tube profiles must have the same vertex count")

    for i in range(n):
        j = (i + 1) % n
        _add_quad(geom, outer_bottom[i], outer_bottom[j], outer_top[j], outer_top[i])
        _add_quad(geom, inner_bottom[j], inner_bottom[i], inner_top[i], inner_top[j])
        _add_quad(geom, outer_top[i], outer_top[j], inner_top[j], inner_top[i])
        _add_quad(geom, outer_bottom[j], outer_bottom[i], inner_bottom[i], inner_bottom[j])
    return geom


def _annular_cylinder_mesh(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    segments: int = 72,
) -> MeshGeometry:
    outer = _circle_profile(outer_radius, segments=segments)
    inner = _circle_profile(inner_radius, segments=segments)
    return _hollow_tube_mesh(outer, inner, -height / 2.0, height / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_food_processor")

    cream = model.material("warm_cream", rgba=(0.86, 0.84, 0.78, 1.0))
    dark = model.material("soft_black", rgba=(0.02, 0.022, 0.026, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.012, 0.012, 0.012, 1.0))
    clear = model.material("clear_blue_polycarbonate", rgba=(0.55, 0.78, 0.95, 0.38))
    smoky = model.material("smoky_clear_polycarbonate", rgba=(0.45, 0.58, 0.68, 0.50))
    steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    green = model.material("start_green", rgba=(0.05, 0.62, 0.22, 1.0))
    red = model.material("stop_red", rgba=(0.75, 0.05, 0.035, 1.0))

    base = model.part("base")
    base.visual(Box((0.34, 0.29, 0.22)), origin=Origin(xyz=(0.0, 0.0, 0.11)), material=cream, name="motor_housing")
    base.visual(Cylinder(radius=0.108, length=0.015), origin=Origin(xyz=(0.0, 0.0, 0.2275)), material=cream, name="bowl_seat")
    base.visual(Box((0.132, 0.014, 0.170)), origin=Origin(xyz=(0.0, -0.152, 0.108)), material=dark, name="control_strip")
    for i, (x, y) in enumerate(((-0.125, -0.105), (0.125, -0.105), (-0.125, 0.105), (0.125, 0.105))):
        base.visual(Box((0.052, 0.038, 0.010)), origin=Origin(xyz=(x, y, 0.005)), material=rubber, name=f"foot_{i}")

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [(0.085, 0.235), (0.132, 0.248), (0.148, 0.480), (0.155, 0.528)],
        [(0.035, 0.250), (0.105, 0.262), (0.128, 0.475), (0.136, 0.514)],
        segments=96,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    bowl.visual(mesh_from_geometry(bowl_shell, "bowl_hollow_shell"), material=clear, name="hollow_bowl")
    bowl.visual(Cylinder(radius=0.045, length=0.025), origin=Origin(xyz=(0.0, 0.0, 0.247)), material=smoky, name="shaft_boss")
    bowl.visual(Cylinder(radius=0.024, length=0.282), origin=Origin(xyz=(0.0, 0.0, 0.382)), material=smoky, name="vertical_shaft")
    bowl.visual(Box((0.050, 0.052, 0.025)), origin=Origin(xyz=(0.153, 0.0, 0.334)), material=clear, name="lower_handle_mount")
    bowl.visual(Box((0.050, 0.052, 0.025)), origin=Origin(xyz=(0.153, 0.0, 0.470)), material=clear, name="upper_handle_mount")
    bowl.visual(Box((0.032, 0.060, 0.170)), origin=Origin(xyz=(0.194, 0.0, 0.402)), material=clear, name="handle_grip")

    model.articulation("base_to_bowl", ArticulationType.FIXED, parent=base, child=bowl, origin=Origin())

    lid = model.part("lid")
    lid_outer = _offset_profile(rounded_rect_profile(0.270, 0.170, 0.035, corner_segments=8), dy=-0.005)
    main_hole = _offset_profile(rounded_rect_profile(0.098, 0.052, 0.012, corner_segments=8), dx=-0.040, dy=-0.020)
    small_hole = _circle_profile(0.019, segments=48, dx=0.065, dy=0.030)
    lid.visual(Box((0.140, 0.014, 0.014)), origin=Origin(xyz=(-0.040, 0.013, 0.54283)), material=clear, name="main_chute_frame_0")
    lid.visual(Box((0.140, 0.014, 0.014)), origin=Origin(xyz=(-0.040, -0.053, 0.54283)), material=clear, name="main_chute_frame_1")
    lid.visual(Box((0.014, 0.080, 0.014)), origin=Origin(xyz=(0.016, -0.020, 0.54283)), material=clear, name="main_chute_frame_2")
    lid.visual(Box((0.014, 0.080, 0.014)), origin=Origin(xyz=(-0.096, -0.020, 0.54283)), material=clear, name="main_chute_frame_3")
    lid.visual(
        mesh_from_geometry(_annular_cylinder_mesh(0.036, 0.019, 0.014, segments=64), "secondary_tube_lid_frame"),
        origin=Origin(xyz=(0.065, 0.030, 0.54283)),
        material=clear,
        name="secondary_tube_frame",
    )
    lid.visual(
        mesh_from_geometry(_annular_cylinder_mesh(0.176, 0.158, 0.014), "lid_outer_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.54283)),
        material=clear,
        name="bowl_rim_clamp",
    )
    lid.visual(Box((0.142, 0.018, 0.014)), origin=Origin(xyz=(0.087, 0.0, 0.54283)), material=clear, name="rim_bridge_0")
    lid.visual(Box((0.070, 0.018, 0.014)), origin=Origin(xyz=(-0.125, 0.0, 0.54283)), material=clear, name="rim_bridge_1")
    model.articulation("bowl_to_lid", ArticulationType.FIXED, parent=bowl, child=lid, origin=Origin())

    main_feed_tube = model.part("main_feed_tube")
    main_outer = _offset_profile(rounded_rect_profile(0.120, 0.075, 0.016, corner_segments=8), dx=-0.040, dy=-0.020)
    main_inner = _offset_profile(rounded_rect_profile(0.098, 0.052, 0.012, corner_segments=8), dx=-0.040, dy=-0.020)
    main_feed_tube.visual(
        mesh_from_geometry(_hollow_tube_mesh(main_outer, main_inner, 0.54983, 0.74983), "main_feed_tube_shell"),
        material=smoky,
        name="hollow_main_chute",
    )
    model.articulation("lid_to_main_feed_tube", ArticulationType.FIXED, parent=lid, child=main_feed_tube, origin=Origin())

    secondary_feed_tube = model.part("secondary_feed_tube")
    secondary_feed_tube.visual(
        mesh_from_geometry(
            _hollow_tube_mesh(
                _circle_profile(0.030, segments=64, dx=0.065, dy=0.030),
                _circle_profile(0.019, segments=64, dx=0.065, dy=0.030),
                0.54983,
                0.74983,
            ),
            "secondary_feed_tube_shell",
        ),
        material=smoky,
        name="hollow_secondary_tube",
    )
    model.articulation("lid_to_secondary_feed_tube", ArticulationType.FIXED, parent=lid, child=secondary_feed_tube, origin=Origin())

    cutter_disc = model.part("cutter_disc")
    cutter_disc.visual(
        mesh_from_geometry(_annular_cylinder_mesh(0.112, 0.031, 0.008, segments=96), "cutter_disc_annulus"),
        material=steel,
        name="perforated_disc",
    )
    cutter_disc.visual(
        mesh_from_geometry(_annular_cylinder_mesh(0.044, 0.027, 0.024, segments=72), "cutter_disc_hub"),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=steel,
        name="shaft_hub",
    )
    cutter_disc.visual(Box((0.092, 0.014, 0.004)), origin=Origin(xyz=(0.030, 0.035, 0.006), rpy=(0.0, 0.0, 0.35)), material=steel, name="slicing_blade_0")
    cutter_disc.visual(Box((0.082, 0.012, 0.004)), origin=Origin(xyz=(-0.034, -0.038, 0.006), rpy=(0.0, 0.0, 3.45)), material=steel, name="slicing_blade_1")
    model.articulation(
        "bowl_to_cutter_disc",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=cutter_disc,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=40.0),
    )

    main_pusher = model.part("main_pusher")
    main_pusher.visual(Box((0.084, 0.040, 0.220)), origin=Origin(xyz=(0.0, 0.0, -0.110)), material=dark, name="main_plunger")
    main_pusher.visual(Box((0.142, 0.092, 0.022)), origin=Origin(xyz=(0.0, 0.0, 0.011)), material=dark, name="main_pusher_cap")
    model.articulation(
        "main_feed_tube_to_main_pusher",
        ArticulationType.PRISMATIC,
        parent=main_feed_tube,
        child=main_pusher,
        origin=Origin(xyz=(-0.040, -0.020, 0.74983)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.090),
    )

    small_pusher = model.part("small_pusher")
    small_pusher.visual(Cylinder(radius=0.0135, length=0.220), origin=Origin(xyz=(0.0, 0.0, -0.110)), material=dark, name="small_plunger")
    small_pusher.visual(Box((0.011, 0.0015, 0.160)), origin=Origin(xyz=(0.0135, 0.0, -0.080)), material=rubber, name="small_guide_fin_0")
    small_pusher.visual(Box((0.011, 0.0015, 0.160)), origin=Origin(xyz=(-0.0135, 0.0, -0.080)), material=rubber, name="small_guide_fin_1")
    small_pusher.visual(Box((0.0015, 0.011, 0.160)), origin=Origin(xyz=(0.0, 0.0135, -0.080)), material=rubber, name="small_guide_fin_2")
    small_pusher.visual(Box((0.0015, 0.011, 0.160)), origin=Origin(xyz=(0.0, -0.0135, -0.080)), material=rubber, name="small_guide_fin_3")
    small_pusher.visual(Cylinder(radius=0.026, length=0.018), origin=Origin(xyz=(0.0, 0.0, 0.009)), material=dark, name="small_pusher_cap")
    model.articulation(
        "secondary_feed_tube_to_small_pusher",
        ArticulationType.PRISMATIC,
        parent=secondary_feed_tube,
        child=small_pusher,
        origin=Origin(xyz=(0.065, 0.030, 0.74983)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.30, lower=0.0, upper=0.085),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(TorusGeometry(radius=0.033, tube=0.005, radial_segments=24, tubular_segments=72), "front_ring_dial"),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="dial_ring",
    )
    dial.visual(Cylinder(radius=0.029, length=0.008), origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=rubber, name="dial_center")
    for i, angle in enumerate((0.0, 2.094, 4.188)):
        dial.visual(
            Box((0.018, 0.004, 0.004)),
            origin=Origin(xyz=(0.017 * math.cos(angle), -0.005, 0.017 * math.sin(angle)), rpy=(0.0, -angle, 0.0)),
            material=dark,
            name=f"dial_spoke_{i}",
        )
    dial.visual(Box((0.006, 0.006, 0.022)), origin=Origin(xyz=(0.0, -0.008, 0.033)), material=cream, name="dial_pointer")
    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.0, -0.159, 0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0),
    )

    start_button = model.part("start_button")
    start_button.visual(Cylinder(radius=0.017, length=0.012), origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=green, name="button_cap")
    model.articulation(
        "base_to_start_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=start_button,
        origin=Origin(xyz=(-0.032, -0.159, 0.072)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.12, lower=0.0, upper=0.008),
    )

    stop_button = model.part("stop_button")
    stop_button.visual(Cylinder(radius=0.017, length=0.012), origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=red, name="button_cap")
    model.articulation(
        "base_to_stop_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stop_button,
        origin=Origin(xyz=(0.032, -0.159, 0.072)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.12, lower=0.0, upper=0.008),
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
    main_tube = object_model.get_part("main_feed_tube")
    main_pusher = object_model.get_part("main_pusher")
    small_tube = object_model.get_part("secondary_feed_tube")
    small_pusher = object_model.get_part("small_pusher")
    cutter_disc = object_model.get_part("cutter_disc")
    bowl = object_model.get_part("bowl")

    main_slide = object_model.get_articulation("main_feed_tube_to_main_pusher")
    small_slide = object_model.get_articulation("secondary_feed_tube_to_small_pusher")

    ctx.expect_within(main_pusher, main_tube, axes="xy", inner_elem="main_plunger", outer_elem="hollow_main_chute", margin=0.004, name="main pusher is guided inside wide feed tube")
    ctx.expect_within(small_pusher, small_tube, axes="xy", inner_elem="small_plunger", outer_elem="hollow_secondary_tube", margin=0.004, name="small pusher is guided inside narrow tube")
    ctx.expect_overlap(cutter_disc, bowl, axes="xy", elem_a="perforated_disc", elem_b="hollow_bowl", min_overlap=0.090, name="cutter disc sits inside bowl processing cavity")
    ctx.expect_gap(main_pusher, cutter_disc, axis="z", positive_elem="main_plunger", negative_elem="perforated_disc", min_gap=0.060, name="main pusher clears cutter disc at rest")

    rest_main = ctx.part_world_position(main_pusher)
    rest_small = ctx.part_world_position(small_pusher)
    with ctx.pose({main_slide: 0.075, small_slide: 0.070}):
        ctx.expect_within(main_pusher, main_tube, axes="xy", inner_elem="main_plunger", outer_elem="hollow_main_chute", margin=0.004, name="main pusher remains centered when lifted")
        ctx.expect_within(small_pusher, small_tube, axes="xy", inner_elem="small_plunger", outer_elem="hollow_secondary_tube", margin=0.004, name="small pusher remains centered when lifted")
        moved_main = ctx.part_world_position(main_pusher)
        moved_small = ctx.part_world_position(small_pusher)
    ctx.check(
        "pushers slide upward while retained in their guides",
        rest_main is not None
        and moved_main is not None
        and moved_main[2] > rest_main[2] + 0.060
        and rest_small is not None
        and moved_small is not None
        and moved_small[2] > rest_small[2] + 0.055,
        details=f"rest_main={rest_main}, moved_main={moved_main}, rest_small={rest_small}, moved_small={moved_small}",
    )

    return ctx.report()


object_model = build_object_model()
