from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_carriage_body_mesh():
    left_block = BoxGeometry((0.055, 0.07, 0.065)).translate(-0.07, -0.005, 0.0)
    right_block = BoxGeometry((0.055, 0.07, 0.065)).translate(0.07, -0.005, 0.0)
    left_bore = (
        CylinderGeometry(radius=0.013, height=0.08, radial_segments=28)
        .rotate_x(pi / 2.0)
        .translate(-0.07, -0.005, 0.0)
    )
    right_bore = (
        CylinderGeometry(radius=0.013, height=0.08, radial_segments=28)
        .rotate_x(pi / 2.0)
        .translate(0.07, -0.005, 0.0)
    )
    left_slider = boolean_difference(left_block, left_bore)
    right_slider = boolean_difference(right_block, right_bore)
    left_slider.merge(right_slider)
    left_slider.merge(BoxGeometry((0.16, 0.02, 0.018)).translate(0.0, -0.028, 0.028))
    return left_slider


def _build_guard_ring_mesh():
    outer = CylinderGeometry(radius=0.126, height=0.028, radial_segments=56).rotate_y(pi / 2.0)
    inner = CylinderGeometry(radius=0.106, height=0.032, radial_segments=56).rotate_y(pi / 2.0)
    ring = boolean_difference(outer, inner)
    lower_cut = BoxGeometry((0.05, 0.30, 0.16)).translate(0.0, 0.0, -0.08)
    upper_shell = boolean_difference(ring, lower_cut)
    rear_cut = BoxGeometry((0.05, 0.16, 0.30)).translate(0.0, -0.13, 0.01)
    return boolean_difference(upper_shell, rear_cut)


def _build_handle_mesh():
    return tube_from_spline_points(
        [
            (0.0, 0.040, 0.055),
            (0.0, 0.080, 0.112),
            (0.0, 0.145, 0.120),
            (0.0, 0.190, 0.080),
        ],
        radius=0.009,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_compound_miter_saw", assets=ASSETS)

    cast_aluminum = model.material("cast_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_charcoal = model.material("dark_charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.78, 0.80, 0.83, 1.0))
    guard_gray = model.material("guard_gray", rgba=(0.48, 0.50, 0.53, 1.0))
    saw_red = model.material("saw_red", rgba=(0.72, 0.16, 0.13, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.84, 0.85, 0.87, 1.0))
    grip_black = model.material("grip_black", rgba=(0.09, 0.09, 0.10, 1.0))

    base = model.part("base")
    base.visual(Box((0.60, 0.38, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.0225)), material=dark_charcoal, name="plinth")
    base.visual(
        Box((0.58, 0.14, 0.018)),
        origin=Origin(xyz=(0.0, -0.12, 0.054)),
        material=cast_aluminum,
        name="rear_deck",
    )
    base.visual(
        Box((0.18, 0.12, 0.018)),
        origin=Origin(xyz=(-0.20, 0.10, 0.054)),
        material=cast_aluminum,
        name="left_support_wing",
    )
    base.visual(
        Box((0.18, 0.12, 0.018)),
        origin=Origin(xyz=(0.20, 0.10, 0.054)),
        material=cast_aluminum,
        name="right_support_wing",
    )
    base.visual(
        Cylinder(radius=0.145, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=cast_aluminum,
        name="turntable_seat",
    )
    base.visual(
        Box((0.20, 0.024, 0.085)),
        origin=Origin(xyz=(-0.16, -0.162, 0.1055)),
        material=cast_aluminum,
        name="left_fence",
    )
    base.visual(
        Box((0.20, 0.024, 0.085)),
        origin=Origin(xyz=(0.16, -0.162, 0.1055)),
        material=cast_aluminum,
        name="right_fence",
    )
    base.visual(
        Box((0.12, 0.11, 0.14)),
        origin=Origin(xyz=(0.0, -0.230, 0.133)),
        material=dark_charcoal,
        name="rear_column",
    )
    base.visual(
        Box((0.22, 0.035, 0.04)),
        origin=Origin(xyz=(0.0, -0.265, 0.223)),
        material=dark_charcoal,
        name="rail_bridge",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.42),
        origin=Origin(xyz=(-0.07, -0.10, 0.225), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rail_steel,
        name="left_rail",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.42),
        origin=Origin(xyz=(0.07, -0.10, 0.225), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rail_steel,
        name="right_rail",
    )
    base.visual(
        Box((0.18, 0.03, 0.012)),
        origin=Origin(xyz=(0.0, 0.16, 0.051)),
        material=dark_charcoal,
        name="front_scale",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.60, 0.38, 0.27)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.13, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=cast_aluminum,
        name="turntable_plate",
    )
    turntable.visual(
        Box((0.028, 0.10, 0.014)),
        origin=Origin(xyz=(0.0, 0.09, 0.007)),
        material=dark_charcoal,
        name="miter_lock_handle",
    )
    turntable.visual(
        Box((0.018, 0.16, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0195)),
        material=guard_gray,
        name="kerf_insert",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.13, length=0.018),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        _save_mesh("miter_saw_carriage_body.obj", _build_carriage_body_mesh()),
        material=guard_gray,
        name="carriage_body",
    )
    carriage.visual(
        Box((0.016, 0.03, 0.11)),
        origin=Origin(xyz=(-0.047, 0.040, 0.055)),
        material=dark_charcoal,
        name="left_yoke",
    )
    carriage.visual(
        Box((0.016, 0.03, 0.11)),
        origin=Origin(xyz=(0.047, 0.040, 0.055)),
        material=dark_charcoal,
        name="right_yoke",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.19, 0.10, 0.10)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    saw_head = model.part("saw_head")
    saw_head.visual(
        Cylinder(radius=0.015, length=0.078),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rail_steel,
        name="pivot_barrel",
    )
    saw_head.visual(
        Box((0.032, 0.20, 0.04)),
        origin=Origin(xyz=(0.0, 0.085, 0.035)),
        material=saw_red,
        name="upper_arm",
    )
    saw_head.visual(
        Box((0.04, 0.07, 0.055)),
        origin=Origin(xyz=(0.03, 0.145, 0.018)),
        material=saw_red,
        name="motor_brace",
    )
    saw_head.visual(
        Box((0.04, 0.06, 0.06)),
        origin=Origin(xyz=(-0.03, 0.135, 0.015)),
        material=guard_gray,
        name="gear_case",
    )
    saw_head.visual(
        Cylinder(radius=0.038, length=0.08),
        origin=Origin(xyz=(0.075, 0.145, -0.020), rpy=(0.0, pi / 2.0, 0.0)),
        material=guard_gray,
        name="motor_housing",
    )
    saw_head.visual(
        Cylinder(radius=0.020, length=0.06),
        origin=Origin(xyz=(0.02, 0.145, -0.030), rpy=(0.0, pi / 2.0, 0.0)),
        material=rail_steel,
        name="blade_hub",
    )
    saw_head.visual(
        _save_mesh("miter_saw_guard_ring.obj", _build_guard_ring_mesh()),
        origin=Origin(xyz=(0.0, 0.145, -0.030)),
        material=guard_gray,
        name="guard_shell",
    )
    saw_head.visual(
        Cylinder(radius=0.105, length=0.004),
        origin=Origin(xyz=(0.0, 0.145, -0.030), rpy=(0.0, pi / 2.0, 0.0)),
        material=blade_steel,
        name="blade",
    )
    saw_head.visual(
        _save_mesh("miter_saw_handle.obj", _build_handle_mesh()),
        material=grip_black,
        name="handle",
    )
    saw_head.inertial = Inertial.from_geometry(
        Box((0.24, 0.22, 0.21)),
        mass=8.5,
        origin=Origin(xyz=(0.02, 0.11, 0.03)),
    )

    model.articulation(
        "base_to_turntable_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-(pi / 4.0), upper=pi / 4.0),
    )
    model.articulation(
        "base_to_carriage_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.11, 0.225)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.4, lower=0.0, upper=0.20),
    )
    model.articulation(
        "carriage_to_head_pitch",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=saw_head,
        origin=Origin(xyz=(0.0, 0.05, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-0.87, upper=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    carriage = object_model.get_part("carriage")
    saw_head = object_model.get_part("saw_head")

    yaw = object_model.get_articulation("base_to_turntable_yaw")
    slide = object_model.get_articulation("base_to_carriage_slide")
    pitch = object_model.get_articulation("carriage_to_head_pitch")

    plinth = base.get_visual("plinth")
    turntable_seat = base.get_visual("turntable_seat")
    left_rail = base.get_visual("left_rail")
    right_rail = base.get_visual("right_rail")
    turntable_plate = turntable.get_visual("turntable_plate")
    carriage_body = carriage.get_visual("carriage_body")
    left_yoke = carriage.get_visual("left_yoke")
    right_yoke = carriage.get_visual("right_yoke")
    pivot_barrel = saw_head.get_visual("pivot_barrel")
    blade = saw_head.get_visual("blade")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=18)

    ctx.check(
        "turntable_yaw_axis_and_limits",
        yaw.axis == (0.0, 0.0, 1.0)
        and abs(yaw.motion_limits.lower + (pi / 4.0)) < 1e-9
        and abs(yaw.motion_limits.upper - (pi / 4.0)) < 1e-9,
        details=f"axis={yaw.axis}, limits=({yaw.motion_limits.lower}, {yaw.motion_limits.upper})",
    )
    ctx.check(
        "carriage_slide_axis_and_stroke",
        slide.axis == (0.0, 1.0, 0.0)
        and abs(slide.motion_limits.lower - 0.0) < 1e-9
        and abs(slide.motion_limits.upper - 0.20) < 1e-9,
        details=f"axis={slide.axis}, limits=({slide.motion_limits.lower}, {slide.motion_limits.upper})",
    )
    ctx.check(
        "head_pitch_axis_and_range",
        pitch.axis == (1.0, 0.0, 0.0)
        and abs(pitch.motion_limits.lower + 0.87) < 1e-9
        and abs(pitch.motion_limits.upper - 0.0) < 1e-9,
        details=f"axis={pitch.axis}, limits=({pitch.motion_limits.lower}, {pitch.motion_limits.upper})",
    )

    ctx.expect_gap(
        turntable,
        base,
        axis="z",
        positive_elem=turntable_plate,
        negative_elem=turntable_seat,
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable_sits_on_base_seat",
    )
    ctx.expect_overlap(
        turntable,
        base,
        axes="xy",
        elem_a=turntable_plate,
        elem_b=turntable_seat,
        min_overlap=0.25,
        name="turntable_centered_on_seat",
    )
    ctx.expect_within(
        turntable,
        base,
        axes="xy",
        inner_elem=turntable_plate,
        outer_elem=plinth,
        margin=0.0,
        name="turntable_within_base_footprint",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="xz",
        elem_a=carriage_body,
        elem_b=left_rail,
        min_overlap=0.015,
        name="carriage_engages_left_rail",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="xz",
        elem_a=carriage_body,
        elem_b=right_rail,
        min_overlap=0.015,
        name="carriage_engages_right_rail",
    )
    ctx.expect_overlap(
        saw_head,
        carriage,
        axes="yz",
        elem_a=pivot_barrel,
        elem_b=left_yoke,
        min_overlap=0.018,
        name="head_pivot_aligned_with_left_yoke",
    )
    ctx.expect_overlap(
        saw_head,
        carriage,
        axes="yz",
        elem_a=pivot_barrel,
        elem_b=right_yoke,
        min_overlap=0.018,
        name="head_pivot_aligned_with_right_yoke",
    )
    ctx.expect_gap(
        saw_head,
        turntable,
        axis="z",
        positive_elem=blade,
        negative_elem=turntable_plate,
        min_gap=0.07,
        max_gap=0.12,
        name="blade_raised_above_turntable",
    )
    ctx.expect_overlap(
        saw_head,
        turntable,
        axes="y",
        elem_a=blade,
        elem_b=turntable_plate,
        min_overlap=0.10,
        name="blade_tracks_over_cutting_zone_at_rest",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.20}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        carriage_travel = None
        if rest_carriage_pos is not None and extended_carriage_pos is not None:
            carriage_travel = extended_carriage_pos[1] - rest_carriage_pos[1]
        ctx.check(
            "carriage_translates_about_200mm",
            carriage_travel is not None and 0.195 <= carriage_travel <= 0.205,
            details=f"expected ~0.200 m forward travel, got {carriage_travel}",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="xz",
            elem_a=carriage_body,
            elem_b=left_rail,
            min_overlap=0.015,
            name="carriage_stays_on_left_rail_when_extended",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="xz",
            elem_a=carriage_body,
            elem_b=right_rail,
            min_overlap=0.015,
            name="carriage_stays_on_right_rail_when_extended",
        )

    rest_blade_aabb = ctx.part_element_world_aabb(saw_head, elem=blade)
    with ctx.pose({pitch: -0.87}):
        lowered_blade_aabb = ctx.part_element_world_aabb(saw_head, elem=blade)
        blade_drop = None
        if rest_blade_aabb is not None and lowered_blade_aabb is not None:
            blade_drop = rest_blade_aabb[0][2] - lowered_blade_aabb[0][2]
        ctx.check(
            "head_pitch_lowers_blade",
            blade_drop is not None and blade_drop >= 0.075,
            details=f"expected blade drop >= 0.075 m, got {blade_drop}",
        )
        ctx.expect_gap(
            saw_head,
            turntable,
            axis="z",
            positive_elem=blade,
            negative_elem=turntable_plate,
            min_gap=0.002,
            max_gap=0.03,
            name="blade_approaches_turntable_without_penetration",
        )
        ctx.expect_overlap(
            saw_head,
            turntable,
            axes="y",
            elem_a=blade,
            elem_b=turntable_plate,
            min_overlap=0.10,
            name="lowered_blade_remains_over_turntable_zone",
        )

    with ctx.pose({yaw: pi / 4.0}):
        ctx.expect_gap(
            turntable,
            base,
            axis="z",
            positive_elem=turntable_plate,
            negative_elem=turntable_seat,
            max_gap=0.001,
            max_penetration=0.0,
            name="yawed_turntable_stays_seated",
        )
        ctx.expect_origin_distance(
            turntable,
            base,
            axes="xy",
            max_dist=0.001,
            name="turntable_yaws_about_central_vertical_axis",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
