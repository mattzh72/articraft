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
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="azimuth_elevation_dish")

    concrete = model.material("concrete", rgba=(0.66, 0.67, 0.69, 1.0))
    dish_white = model.material("dish_white", rgba=(0.90, 0.92, 0.94, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.36, 0.39, 0.43, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.15, 0.17, 0.20, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.76, 0.79, 1.0))
    instrument_black = model.material("instrument_black", rgba=(0.08, 0.09, 0.10, 1.0))

    arm_profile = rounded_rect_profile(0.20, 0.28, radius=0.045, corner_segments=8)
    left_arm_mesh = _mesh(
        "left_yoke_arm",
        sweep_profile_along_spline(
            [
                (0.02, 0.98, 0.50),
                (0.08, 1.00, 1.18),
                (0.11, 1.03, 1.88),
                (0.13, 1.01, 2.34),
                (0.17, 0.96, 2.58),
            ],
            profile=arm_profile,
            samples_per_segment=16,
            cap_profile=True,
        ),
    )
    right_arm_mesh = _mesh(
        "right_yoke_arm",
        sweep_profile_along_spline(
            [
                (0.02, -0.98, 0.50),
                (0.08, -1.00, 1.18),
                (0.11, -1.03, 1.88),
                (0.13, -1.01, 2.34),
                (0.17, -0.96, 2.58),
            ],
            profile=arm_profile,
            samples_per_segment=16,
            cap_profile=True,
        ),
    )
    left_rear_brace_mesh = _mesh(
        "left_rear_brace",
        sweep_profile_along_spline(
            [
                (-0.42, 0.56, 0.66),
                (-0.27, 0.68, 1.10),
                (-0.10, 0.80, 1.58),
                (0.05, 0.89, 2.02),
                (0.12, 0.93, 2.42),
            ],
            profile=rounded_rect_profile(0.12, 0.18, radius=0.028, corner_segments=8),
            samples_per_segment=14,
            cap_profile=True,
        ),
    )
    right_rear_brace_mesh = _mesh(
        "right_rear_brace",
        sweep_profile_along_spline(
            [
                (-0.42, -0.56, 0.66),
                (-0.27, -0.68, 1.10),
                (-0.10, -0.80, 1.58),
                (0.05, -0.89, 2.02),
                (0.12, -0.93, 2.42),
            ],
            profile=rounded_rect_profile(0.12, 0.18, radius=0.028, corner_segments=8),
            samples_per_segment=14,
            cap_profile=True,
        ),
    )
    rib_mesh = _mesh(
        "dish_back_rib",
        tube_from_spline_points(
            [
                (0.08, 0.0, 0.04),
                (0.18, 0.0, 0.16),
                (0.34, 0.0, 0.55),
                (0.54, 0.0, 1.08),
                (0.61, 0.0, 1.46),
            ],
            radius=0.035,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    feed_strut_mesh = _mesh(
        "feed_support_strut",
        tube_from_spline_points(
            [
                (0.40, 0.0, 0.20),
                (0.82, 0.0, 0.34),
                (1.30, 0.0, 0.22),
                (1.78, 0.0, 0.04),
            ],
            radius=0.028,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    reflector_shell_mesh = _mesh(
        "reflector_shell",
        LatheGeometry.from_shell_profiles(
            [
                (0.00, -0.52),
                (0.28, -0.50),
                (0.82, -0.43),
                (1.45, -0.26),
                (1.95, -0.08),
                (2.12, 0.00),
            ],
            [
                (0.00, -0.46),
                (0.24, -0.45),
                (0.78, -0.39),
                (1.42, -0.24),
                (1.92, -0.075),
                (2.04, -0.01),
            ],
            segments=80,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    reflector_rim_mesh = _mesh(
        "reflector_rim",
        TorusGeometry(radius=2.08, tube=0.045, radial_segments=18, tubular_segments=84),
    )
    rear_ring_outer_mesh = _mesh(
        "rear_ring_outer",
        TorusGeometry(radius=1.46, tube=0.032, radial_segments=16, tubular_segments=64),
    )
    service_collar_mesh = _mesh(
        "service_collar",
        TorusGeometry(radius=1.06, tube=0.04, radial_segments=16, tubular_segments=56),
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Box((3.20, 3.20, 0.40)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=concrete,
        name="foundation_plinth",
    )
    pedestal_base.visual(
        Cylinder(radius=0.58, length=2.20),
        origin=Origin(xyz=(0.0, 0.0, 1.50)),
        material=dish_white,
        name="pedestal_column",
    )
    pedestal_base.visual(
        Cylinder(radius=0.92, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 2.81)),
        material=machinery_gray,
        name="azimuth_housing",
    )
    pedestal_base.visual(
        service_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 3.04)),
        material=dark_steel,
        name="service_collar",
    )
    pedestal_base.visual(
        Box((0.22, 0.08, 0.08)),
        origin=Origin(xyz=(0.97, 0.0, 3.02)),
        material=dark_steel,
        name="collar_bracket_pos_x",
    )
    pedestal_base.visual(
        Box((0.22, 0.08, 0.08)),
        origin=Origin(xyz=(-0.97, 0.0, 3.02)),
        material=dark_steel,
        name="collar_bracket_neg_x",
    )
    pedestal_base.visual(
        Box((0.08, 0.22, 0.08)),
        origin=Origin(xyz=(0.0, 0.97, 3.02)),
        material=dark_steel,
        name="collar_bracket_pos_y",
    )
    pedestal_base.visual(
        Box((0.08, 0.22, 0.08)),
        origin=Origin(xyz=(0.0, -0.97, 3.02)),
        material=dark_steel,
        name="collar_bracket_neg_y",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((3.20, 3.20, 3.05)),
        mass=4800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.525)),
    )

    azimuth_yoke = model.part("azimuth_yoke")
    azimuth_yoke.visual(
        Cylinder(radius=0.84, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=dark_steel,
        name="turntable_drum",
    )
    azimuth_yoke.visual(
        Cylinder(radius=1.06, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        material=machinery_gray,
        name="rotating_deck",
    )
    azimuth_yoke.visual(
        Cylinder(radius=0.34, length=0.95),
        origin=Origin(xyz=(0.0, 0.0, 1.06)),
        material=dish_white,
        name="center_turret",
    )
    azimuth_yoke.visual(left_arm_mesh, material=dish_white, name="left_arm")
    azimuth_yoke.visual(right_arm_mesh, material=dish_white, name="right_arm")
    azimuth_yoke.visual(left_rear_brace_mesh, material=machinery_gray, name="left_rear_brace")
    azimuth_yoke.visual(right_rear_brace_mesh, material=machinery_gray, name="right_rear_brace")
    azimuth_yoke.visual(
        Box((0.30, 0.24, 0.44)),
        origin=Origin(xyz=(0.19, 0.95, 2.78)),
        material=dish_white,
        name="left_bearing_saddle",
    )
    azimuth_yoke.visual(
        Box((0.30, 0.24, 0.44)),
        origin=Origin(xyz=(0.19, -0.95, 2.78)),
        material=dish_white,
        name="right_bearing_saddle",
    )
    azimuth_yoke.visual(
        Cylinder(radius=0.22, length=0.24),
        origin=Origin(xyz=(0.45, 0.92, 2.92), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_bearing",
    )
    azimuth_yoke.visual(
        Cylinder(radius=0.22, length=0.24),
        origin=Origin(xyz=(0.45, -0.92, 2.92), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_bearing",
    )
    azimuth_yoke.visual(
        Box((0.64, 0.88, 0.54)),
        origin=Origin(xyz=(-0.06, 0.0, 0.92)),
        material=dark_steel,
        name="drive_pack",
    )
    azimuth_yoke.inertial = Inertial.from_geometry(
        Box((2.30, 2.25, 2.10)),
        mass=1450.0,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
    )

    dish_assembly = model.part("dish_assembly")
    dish_assembly.visual(
        Cylinder(radius=0.16, length=1.60),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    dish_assembly.visual(
        Cylinder(radius=0.30, length=0.56),
        origin=Origin(xyz=(0.26, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="hub_barrel",
    )
    dish_assembly.visual(
        Box((0.56, 0.90, 0.82)),
        origin=Origin(xyz=(0.22, 0.0, 0.0)),
        material=machinery_gray,
        name="hub_block",
    )
    dish_assembly.visual(
        reflector_shell_mesh,
        origin=Origin(xyz=(0.72, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dish_white,
        name="main_reflector",
    )
    dish_assembly.visual(
        reflector_rim_mesh,
        origin=Origin(xyz=(0.72, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="rim_ring",
    )
    dish_assembly.visual(
        rear_ring_outer_mesh,
        origin=Origin(xyz=(0.46, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_ring_outer",
    )
    for index in range(8):
        angle = (2.0 * math.pi * index) / 8.0
        dish_assembly.visual(
            rib_mesh,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=dark_steel,
            name=f"back_rib_{index:02d}",
        )
    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0
        dish_assembly.visual(
            feed_strut_mesh,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=aluminum,
            name=f"feed_strut_{index:02d}",
        )
    dish_assembly.visual(
        Cylinder(radius=0.06, length=1.48),
        origin=Origin(xyz=(1.13, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="feed_mount",
    )
    dish_assembly.visual(
        Box((0.30, 0.22, 0.20)),
        origin=Origin(xyz=(2.01, 0.0, 0.0)),
        material=instrument_black,
        name="feed_receiver",
    )
    dish_assembly.visual(
        Cylinder(radius=0.095, length=0.32),
        origin=Origin(xyz=(2.32, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="feed_horn",
    )
    dish_assembly.inertial = Inertial.from_geometry(
        Box((2.90, 4.40, 4.40)),
        mass=1650.0,
        origin=Origin(xyz=(0.88, 0.0, 0.0)),
    )

    instrument_box = model.part("instrument_box")
    instrument_box.visual(
        Box((0.55, 0.62, 0.56)),
        material=machinery_gray,
        name="instrument_box_shell",
    )
    instrument_box.visual(
        Box((0.04, 0.48, 0.40)),
        origin=Origin(xyz=(-0.255, 0.0, 0.0)),
        material=dark_steel,
        name="hatch_frame",
    )
    instrument_box.inertial = Inertial.from_geometry(
        Box((0.55, 0.62, 0.56)),
        mass=120.0,
    )

    instrument_hatch = model.part("instrument_hatch")
    instrument_hatch.visual(
        Box((0.03, 0.42, 0.34)),
        origin=Origin(xyz=(-0.015, -0.21, 0.0)),
        material=machinery_gray,
        name="hatch_panel",
    )
    instrument_hatch.visual(
        Cylinder(radius=0.012, length=0.34),
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        material=dark_steel,
        name="hatch_hinge_barrel",
    )
    instrument_hatch.visual(
        Cylinder(radius=0.010, length=0.14),
        origin=Origin(xyz=(-0.030, -0.32, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=instrument_black,
        name="hatch_handle",
    )
    instrument_hatch.inertial = Inertial.from_geometry(
        Box((0.04, 0.44, 0.36)),
        mass=12.0,
        origin=Origin(xyz=(-0.02, -0.21, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=azimuth_yoke,
        origin=Origin(xyz=(0.0, 0.0, 3.02)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.4),
    )
    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent=azimuth_yoke,
        child=dish_assembly,
        origin=Origin(xyz=(0.45, 0.0, 2.92)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=0.7,
            lower=math.radians(-10.0),
            upper=math.radians(95.0),
        ),
    )
    model.articulation(
        "dish_to_instrument_box",
        ArticulationType.FIXED,
        parent=dish_assembly,
        child=instrument_box,
        origin=Origin(xyz=(-0.335, 0.0, -0.48)),
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=instrument_box,
        child=instrument_hatch,
        origin=Origin(xyz=(-0.275, 0.21, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.5,
            lower=0.0,
            upper=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal_base = object_model.get_part("pedestal_base")
    azimuth_yoke = object_model.get_part("azimuth_yoke")
    dish_assembly = object_model.get_part("dish_assembly")
    instrument_box = object_model.get_part("instrument_box")
    instrument_hatch = object_model.get_part("instrument_hatch")
    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    elevation_axis = object_model.get_articulation("elevation_axis")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    for part in (
        pedestal_base,
        azimuth_yoke,
        dish_assembly,
        instrument_box,
        instrument_hatch,
    ):
        ctx.check(f"{part.name} exists", part is not None, details=part.name)

    ctx.expect_contact(
        azimuth_yoke,
        pedestal_base,
        elem_a="turntable_drum",
        elem_b="azimuth_housing",
        contact_tol=0.003,
        name="turntable sits on azimuth housing",
    )
    ctx.expect_contact(
        dish_assembly,
        azimuth_yoke,
        elem_a="trunnion_shaft",
        elem_b="left_bearing",
        contact_tol=0.003,
        name="left elevation bearing supports trunnion",
    )
    ctx.expect_contact(
        dish_assembly,
        azimuth_yoke,
        elem_a="trunnion_shaft",
        elem_b="right_bearing",
        contact_tol=0.003,
        name="right elevation bearing supports trunnion",
    )
    ctx.expect_contact(
        instrument_box,
        dish_assembly,
        elem_a="instrument_box_shell",
        elem_b="hub_block",
        contact_tol=0.003,
        name="instrument box mounts against dish hub",
    )

    with ctx.pose({hatch_hinge: 0.0}):
        ctx.expect_gap(
            instrument_box,
            instrument_hatch,
            axis="x",
            positive_elem="instrument_box_shell",
            negative_elem="hatch_panel",
            max_gap=0.002,
            max_penetration=0.0,
            name="hatch closes flush to instrument box rear face",
        )
        ctx.expect_overlap(
            instrument_box,
            instrument_hatch,
            axes="yz",
            elem_a="instrument_box_shell",
            elem_b="hatch_panel",
            min_overlap=0.28,
            name="closed hatch covers rear service opening",
        )

    hatch_closed_pos = _aabb_center(ctx.part_element_world_aabb(instrument_hatch, elem="hatch_panel"))
    with ctx.pose({hatch_hinge: 1.1}):
        hatch_open_pos = _aabb_center(ctx.part_element_world_aabb(instrument_hatch, elem="hatch_panel"))
    ctx.check(
        "hatch opens outward on side hinge",
        hatch_closed_pos is not None
        and hatch_open_pos is not None
        and hatch_open_pos[0] < hatch_closed_pos[0] - 0.05,
        details=f"closed={hatch_closed_pos}, open={hatch_open_pos}",
    )

    rest_feed = _aabb_center(ctx.part_element_world_aabb(dish_assembly, elem="feed_receiver"))
    with ctx.pose({elevation_axis: math.radians(60.0)}):
        elevated_feed = _aabb_center(ctx.part_element_world_aabb(dish_assembly, elem="feed_receiver"))
    ctx.check(
        "positive elevation raises feed end of dish",
        rest_feed is not None
        and elevated_feed is not None
        and elevated_feed[2] > rest_feed[2] + 0.8,
        details=f"rest={rest_feed}, elevated={elevated_feed}",
    )

    with ctx.pose({azimuth_rotation: math.pi / 2.0}):
        turned_feed = _aabb_center(ctx.part_element_world_aabb(dish_assembly, elem="feed_receiver"))
    ctx.check(
        "azimuth rotation swings dish around vertical axis",
        rest_feed is not None
        and turned_feed is not None
        and turned_feed[1] > rest_feed[1] + 1.2,
        details=f"rest={rest_feed}, turned={turned_feed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
