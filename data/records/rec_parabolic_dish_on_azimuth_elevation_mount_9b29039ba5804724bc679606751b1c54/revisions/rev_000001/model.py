from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _parabolic_reflector_shell() -> LatheGeometry:
    """Thin, open parabolic reflector shell, authored around local +Z."""

    radius = 0.90
    depth = 0.30
    back_offset = 0.09
    thickness = 0.026
    samples = 18

    inner_profile = []
    outer_profile = []
    for i in range(samples):
        t = i / (samples - 1)
        r = 0.055 + (radius - 0.055) * t
        z = back_offset - depth * (r / radius) ** 2
        inner_profile.append((r, z))
        outer_profile.append((r, z + thickness))

    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="az_el_dish_with_hatch")

    white = Material("mat_warm_white", color=(0.86, 0.88, 0.84, 1.0))
    dark = Material("mat_dark_grey", color=(0.12, 0.13, 0.14, 1.0))
    steel = Material("mat_brushed_steel", color=(0.58, 0.60, 0.62, 1.0))
    rubber = Material("mat_black_rubber", color=(0.025, 0.025, 0.025, 1.0))
    blue = Material("mat_blue_hatch", color=(0.09, 0.22, 0.36, 1.0))
    amber = Material("mat_amber_lens", color=(1.0, 0.62, 0.16, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.72, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=dark,
        name="base_slab",
    )
    pedestal.visual(
        Cylinder(radius=0.46, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
        material=steel,
        name="pedestal_drum",
    )
    pedestal.visual(
        Cylinder(radius=0.34, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=steel,
        name="fixed_bearing",
    )
    for x, y in ((0.48, 0.0), (-0.48, 0.0), (0.0, 0.48), (0.0, -0.48)):
        pedestal.visual(
            Cylinder(radius=0.045, length=0.018),
            origin=Origin(xyz=(x, y, 0.151)),
            material=rubber,
            name=f"anchor_bolt_{len(pedestal.visuals)}",
        )

    azimuth_frame = model.part("azimuth_frame")
    azimuth_frame.visual(
        Cylinder(radius=0.34, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=steel,
        name="turntable",
    )
    azimuth_frame.visual(
        Cylinder(radius=0.18, length=0.95),
        origin=Origin(xyz=(0.42, 0.0, 0.515)),
        material=steel,
        name="rotating_column",
    )
    azimuth_frame.visual(
        Box((0.20, 2.22, 0.10)),
        origin=Origin(xyz=(0.42, 0.0, 0.60)),
        material=steel,
        name="rear_crossbar",
    )
    azimuth_frame.visual(
        Box((0.46, 0.16, 0.10)),
        origin=Origin(xyz=(0.22, 1.05, 0.60)),
        material=steel,
        name="upper_side_rail",
    )
    azimuth_frame.visual(
        Box((0.46, 0.16, 0.10)),
        origin=Origin(xyz=(0.22, -1.05, 0.60)),
        material=steel,
        name="lower_side_rail",
    )
    azimuth_frame.visual(
        Box((0.22, 0.12, 0.74)),
        origin=Origin(xyz=(0.02, 1.05, 0.95)),
        material=steel,
        name="upper_bearing_cheek",
    )
    azimuth_frame.visual(
        Cylinder(radius=0.16, length=0.08),
        origin=Origin(xyz=(0.02, 1.030, 1.30), rpy=(-pi / 2, 0.0, 0.0)),
        material=dark,
        name="upper_bearing_pad",
    )
    azimuth_frame.visual(
        Box((0.22, 0.12, 0.74)),
        origin=Origin(xyz=(0.02, -1.05, 0.95)),
        material=steel,
        name="lower_bearing_cheek",
    )
    azimuth_frame.visual(
        Cylinder(radius=0.16, length=0.08),
        origin=Origin(xyz=(0.02, -1.030, 1.30), rpy=(-pi / 2, 0.0, 0.0)),
        material=dark,
        name="lower_bearing_pad",
    )

    dish = model.part("dish")
    dish.visual(
        mesh_from_geometry(_parabolic_reflector_shell(), "reflector_shell"),
        origin=Origin(rpy=(0.0, pi / 2, 0.0)),
        material=white,
        name="reflector_shell",
    )
    dish.visual(
        mesh_from_geometry(TorusGeometry(radius=0.90, tube=0.024, radial_segments=24, tubular_segments=96), "reflector_rim"),
        origin=Origin(xyz=(-0.21, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=white,
        name="reflector_rim",
    )
    dish.visual(
        Cylinder(radius=0.11, length=0.12),
        origin=Origin(xyz=(0.085, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=white,
        name="rear_neck",
    )
    dish.visual(
        Box((0.34, 0.52, 0.34)),
        origin=Origin(xyz=(0.27, 0.0, 0.0)),
        material=dark,
        name="instrument_box",
    )
    dish.visual(
        Box((0.09, 0.40, 0.07)),
        origin=Origin(xyz=(0.09, 0.0, 0.0)),
        material=steel,
        name="box_mount_spine",
    )
    dish.visual(
        Box((0.08, 1.84, 0.10)),
        origin=Origin(xyz=(0.13, 0.0, 0.0)),
        material=steel,
        name="trunnion_spreader",
    )
    for z, length, name in ((0.28, 0.64, "upper_backstay"), (-0.28, 0.64, "lower_backstay")):
        dish.visual(
            Box((0.06, 0.06, length)),
            origin=Origin(xyz=(0.15, 0.0, z), rpy=(0.0, pi / 9, 0.0)),
            material=steel,
            name=name,
        )
    dish.visual(
        Cylinder(radius=0.075, length=0.08),
        origin=Origin(xyz=(0.02, 0.95, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=steel,
        name="trunnion_0",
    )
    dish.visual(
        Cylinder(radius=0.12, length=0.035),
        origin=Origin(xyz=(0.02, 0.893, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=dark,
        name="trunnion_0_hub",
    )
    dish.visual(
        Cylinder(radius=0.075, length=0.08),
        origin=Origin(xyz=(0.02, -0.95, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=steel,
        name="trunnion_1",
    )
    dish.visual(
        Cylinder(radius=0.12, length=0.035),
        origin=Origin(xyz=(0.02, -0.893, 0.0), rpy=(-pi / 2, 0.0, 0.0)),
        material=dark,
        name="trunnion_1_hub",
    )
    for z, name in ((0.095, "hinge_leaf_0"), (-0.095, "hinge_leaf_1")):
        dish.visual(
            Box((0.026, 0.040, 0.045)),
            origin=Origin(xyz=(0.445, -0.190, z)),
            material=steel,
            name=name,
        )
        dish.visual(
            Cylinder(radius=0.017, length=0.055),
            origin=Origin(xyz=(0.461, -0.16, z)),
            material=steel,
            name=f"{name}_barrel",
        )
    dish.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.444, 0.16, -0.10), rpy=(0.0, pi / 2, 0.0)),
        material=amber,
        name="service_lamp",
    )

    hatch = model.part("hatch")
    hatch.visual(
        Box((0.022, 0.25, 0.22)),
        origin=Origin(xyz=(-0.005, 0.146, 0.0)),
        material=blue,
        name="hatch_panel",
    )
    hatch.visual(
        Cylinder(radius=0.016, length=0.072),
        origin=Origin(),
        material=steel,
        name="hatch_barrel",
    )
    hatch.visual(
        Box((0.018, 0.018, 0.080)),
        origin=Origin(xyz=(-0.005, 0.019, 0.0)),
        material=steel,
        name="hatch_hinge_leaf",
    )
    hatch.visual(
        Box((0.012, 0.04, 0.055)),
        origin=Origin(xyz=(-0.016, 0.20, 0.0)),
        material=steel,
        name="hatch_pull",
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=azimuth_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.45),
    )
    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=azimuth_frame,
        child=dish,
        origin=Origin(xyz=(0.02, 0.0, 1.30)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=-0.35, upper=1.20),
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=dish,
        child=hatch,
        origin=Origin(xyz=(0.461, -0.16, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.70),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    azimuth_frame = object_model.get_part("azimuth_frame")
    dish = object_model.get_part("dish")
    hatch = object_model.get_part("hatch")
    elevation = object_model.get_articulation("elevation")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    ctx.expect_gap(
        azimuth_frame,
        pedestal,
        axis="z",
        positive_elem="turntable",
        negative_elem="fixed_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable sits on fixed bearing",
    )
    ctx.expect_gap(
        azimuth_frame,
        dish,
        axis="y",
        positive_elem="upper_bearing_pad",
        negative_elem="trunnion_0",
        max_gap=0.003,
        max_penetration=0.0,
        name="upper yoke pad captures trunnion",
    )
    ctx.expect_gap(
        dish,
        azimuth_frame,
        axis="y",
        positive_elem="trunnion_1",
        negative_elem="lower_bearing_pad",
        max_gap=0.003,
        max_penetration=0.0,
        name="lower yoke pad captures trunnion",
    )
    ctx.expect_gap(
        hatch,
        dish,
        axis="x",
        positive_elem="hatch_panel",
        negative_elem="instrument_box",
        min_gap=0.001,
        max_gap=0.008,
        name="hatch sits proud of rear box",
    )
    ctx.expect_within(
        hatch,
        dish,
        axes="yz",
        inner_elem="hatch_panel",
        outer_elem="instrument_box",
        margin=0.005,
        name="hatch panel is on the instrument box face",
    )

    closed_rim_aabb = ctx.part_element_world_aabb(dish, elem="reflector_rim")
    with ctx.pose({elevation: 0.70}):
        raised_rim_aabb = ctx.part_element_world_aabb(dish, elem="reflector_rim")
    ctx.check(
        "positive elevation raises reflector face",
        closed_rim_aabb is not None
        and raised_rim_aabb is not None
        and raised_rim_aabb[0][2] > closed_rim_aabb[0][2] + 0.05,
        details=f"closed={closed_rim_aabb}, raised={raised_rim_aabb}",
    )

    closed_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    with ctx.pose({hatch_hinge: 1.10}):
        open_hatch_aabb = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    ctx.check(
        "positive hatch hinge opens rearward",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[1][0] > closed_hatch_aabb[1][0] + 0.10,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
