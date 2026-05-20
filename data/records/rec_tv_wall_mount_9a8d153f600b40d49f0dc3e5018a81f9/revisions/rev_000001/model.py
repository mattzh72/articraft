from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_display_bracket")

    powder_coat = Material("satin_black_powder_coat", rgba=(0.02, 0.022, 0.025, 1.0))
    dark_metal = Material("dark_burnished_steel", rgba=(0.09, 0.095, 0.10, 1.0))
    edge_wear = Material("worn_edge_highlights", rgba=(0.30, 0.31, 0.32, 1.0))
    fastener = Material("black_oxide_fasteners", rgba=(0.005, 0.005, 0.006, 1.0))

    model.materials.extend([powder_coat, dark_metal, edge_wear, fastener])

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.035, 0.230, 0.420)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=powder_coat,
        name="plate_slab",
    )
    wall_plate.visual(
        Box((0.012, 0.170, 0.360)),
        origin=Origin(xyz=(0.0235, 0.0, 0.0)),
        material=edge_wear,
        name="raised_pressing",
    )
    # Four flush screw heads visibly ground the root plate to the side wall.
    for i, (y, z) in enumerate(((-0.075, -0.150), (0.075, -0.150), (-0.075, 0.150), (0.075, 0.150))):
        wall_plate.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(0.0340, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=fastener,
            name=f"screw_head_{i}",
        )
    # A wall-side clevis supports the stout first arm link around a vertical pin.
    wall_plate.visual(
        Box((0.100, 0.140, 0.050)),
        origin=Origin(xyz=(0.0675, 0.0, 0.095)),
        material=dark_metal,
        name="upper_wall_lug",
    )
    wall_plate.visual(
        Box((0.100, 0.140, 0.050)),
        origin=Origin(xyz=(0.0675, 0.0, -0.095)),
        material=dark_metal,
        name="lower_wall_lug",
    )
    wall_plate.visual(
        Cylinder(radius=0.014, length=0.170),
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        material=fastener,
        name="shoulder_pin",
    )

    first_link = model.part("first_link")
    first_link.visual(
        Cylinder(radius=0.042, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="shoulder_barrel",
    )
    first_link.visual(
        Box((0.036, 0.060, 0.052)),
        origin=Origin(xyz=(0.057, 0.0, 0.0)),
        material=dark_metal,
        name="shoulder_web",
    )
    first_link.visual(
        Box((0.275, 0.078, 0.060)),
        origin=Origin(xyz=(0.1925, 0.0, 0.0)),
        material=powder_coat,
        name="stout_tube",
    )
    first_link.visual(
        Box((0.022, 0.118, 0.178)),
        origin=Origin(xyz=(0.319, 0.0, 0.0)),
        material=dark_metal,
        name="elbow_bridge",
    )
    first_link.visual(
        Box((0.120, 0.118, 0.054)),
        origin=Origin(xyz=(0.380, 0.0, 0.065)),
        material=dark_metal,
        name="upper_elbow_lug",
    )
    first_link.visual(
        Box((0.120, 0.118, 0.054)),
        origin=Origin(xyz=(0.380, 0.0, -0.065)),
        material=dark_metal,
        name="lower_elbow_lug",
    )
    first_link.visual(
        Cylinder(radius=0.012, length=0.156),
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        material=fastener,
        name="elbow_pin",
    )

    second_link = model.part("second_link")
    second_link.visual(
        Cylinder(radius=0.032, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="elbow_barrel",
    )
    second_link.visual(
        Box((0.220, 0.054, 0.040)),
        origin=Origin(xyz=(0.140, 0.0, 0.0)),
        material=powder_coat,
        name="slim_tube",
    )
    second_link.visual(
        Box((0.020, 0.098, 0.150)),
        origin=Origin(xyz=(0.244, 0.0, 0.0)),
        material=dark_metal,
        name="swivel_bridge",
    )
    second_link.visual(
        Box((0.100, 0.098, 0.050)),
        origin=Origin(xyz=(0.300, 0.0, 0.060)),
        material=dark_metal,
        name="upper_swivel_lug",
    )
    second_link.visual(
        Box((0.100, 0.098, 0.050)),
        origin=Origin(xyz=(0.300, 0.0, -0.060)),
        material=dark_metal,
        name="lower_swivel_lug",
    )
    second_link.visual(
        Cylinder(radius=0.010, length=0.135),
        origin=Origin(xyz=(0.300, 0.0, 0.0)),
        material=fastener,
        name="swivel_pin",
    )

    swivel_yoke = model.part("swivel_yoke")
    swivel_yoke.visual(
        Cylinder(radius=0.029, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="swivel_barrel",
    )
    swivel_yoke.visual(
        Box((0.050, 0.048, 0.040)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=powder_coat,
        name="short_neck",
    )
    swivel_yoke.visual(
        Box((0.032, 0.160, 0.040)),
        origin=Origin(xyz=(0.061, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_crosshead",
    )
    swivel_yoke.visual(
        Box((0.070, 0.022, 0.120)),
        origin=Origin(xyz=(0.100, 0.069, 0.0)),
        material=dark_metal,
        name="upper_tilt_ear",
    )
    swivel_yoke.visual(
        Box((0.070, 0.022, 0.120)),
        origin=Origin(xyz=(0.100, -0.069, 0.0)),
        material=dark_metal,
        name="lower_tilt_ear",
    )
    swivel_yoke.visual(
        Cylinder(radius=0.010, length=0.165),
        origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=fastener,
        name="tilt_pin",
    )

    head_frame = model.part("head_frame")
    head_frame.visual(
        Cylinder(radius=0.018, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="tilt_barrel",
    )
    head_frame.visual(
        Box((0.046, 0.064, 0.036)),
        origin=Origin(xyz=(0.037, 0.0, 0.0)),
        material=dark_metal,
        name="barrel_stem",
    )
    head_frame.visual(
        Box((0.020, 0.222, 0.018)),
        origin=Origin(xyz=(0.074, 0.0, 0.082)),
        material=powder_coat,
        name="top_rail",
    )
    head_frame.visual(
        Box((0.020, 0.222, 0.018)),
        origin=Origin(xyz=(0.074, 0.0, -0.082)),
        material=powder_coat,
        name="bottom_rail",
    )
    head_frame.visual(
        Box((0.020, 0.018, 0.164)),
        origin=Origin(xyz=(0.074, 0.102, 0.0)),
        material=powder_coat,
        name="side_rail_0",
    )
    head_frame.visual(
        Box((0.020, 0.018, 0.164)),
        origin=Origin(xyz=(0.074, -0.102, 0.0)),
        material=powder_coat,
        name="side_rail_1",
    )
    head_frame.visual(
        Box((0.018, 0.198, 0.014)),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=edge_wear,
        name="center_crossbar",
    )
    head_frame.visual(
        Box((0.018, 0.014, 0.148)),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material=edge_wear,
        name="center_upright",
    )
    head_frame.visual(
        Box((0.012, 0.150, 0.112)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=dark_metal,
        name="vesa_plate",
    )
    for i, (y, z) in enumerate(((-0.060, -0.045), (0.060, -0.045), (-0.060, 0.045), (0.060, 0.045))):
        head_frame.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(xyz=(0.052, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=fastener,
            name=f"vesa_boss_{i}",
        )

    model.articulation(
        "wall_to_first",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=first_link,
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.6, lower=-2.10, upper=2.10),
    )
    model.articulation(
        "second_to_swivel",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=swivel_yoke,
        origin=Origin(xyz=(0.300, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.40, upper=1.40),
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel_yoke,
        child=head_frame,
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.7, lower=-0.55, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    expected_joints = ("wall_to_first", "first_to_second", "second_to_swivel", "swivel_to_head")
    ctx.check(
        "four revolute bracket joints",
        all(object_model.get_articulation(name).articulation_type == ArticulationType.REVOLUTE for name in expected_joints),
        details="The side-wall arm should have two arm pivots plus separate swivel and tilt pivots.",
    )

    wall = object_model.get_part("wall_plate")
    first = object_model.get_part("first_link")
    second = object_model.get_part("second_link")
    swivel = object_model.get_part("swivel_yoke")
    head = object_model.get_part("head_frame")

    ctx.allow_overlap(
        wall,
        first,
        elem_a="shoulder_pin",
        elem_b="shoulder_barrel",
        reason="The first arm link rotates on a captured vertical shoulder pin inside the barrel.",
    )
    ctx.allow_overlap(
        first,
        second,
        elem_a="elbow_pin",
        elem_b="elbow_barrel",
        reason="The slimmer second link is intentionally retained by the elbow pin inside its barrel.",
    )
    ctx.allow_overlap(
        second,
        swivel,
        elem_a="swivel_pin",
        elem_b="swivel_barrel",
        reason="The compact head swivel has a captured vertical pin represented inside the swivel barrel.",
    )
    ctx.allow_overlap(
        swivel,
        head,
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        reason="The tilt axis is a horizontal pin intentionally captured by the head-frame barrel.",
    )

    ctx.expect_overlap(
        wall,
        first,
        axes="xy",
        elem_a="shoulder_pin",
        elem_b="shoulder_barrel",
        min_overlap=0.020,
        name="first link is captured on wall shoulder pin",
    )
    ctx.expect_overlap(
        first,
        second,
        axes="xy",
        elem_a="elbow_pin",
        elem_b="elbow_barrel",
        min_overlap=0.018,
        name="second link is captured on elbow pin",
    )
    ctx.expect_overlap(
        second,
        swivel,
        axes="xy",
        elem_a="swivel_pin",
        elem_b="swivel_barrel",
        min_overlap=0.016,
        name="head swivel is captured on second-link pin",
    )
    ctx.expect_overlap(
        swivel,
        head,
        axes="yz",
        elem_a="tilt_pin",
        elem_b="tilt_barrel",
        min_overlap=0.016,
        name="head frame is captured on horizontal tilt pin",
    )

    first_to_second = object_model.get_articulation("first_to_second")
    second_to_swivel = object_model.get_articulation("second_to_swivel")
    swivel_to_head = object_model.get_articulation("swivel_to_head")

    straight_head = ctx.part_world_position(head)
    with ctx.pose({first_to_second: 0.70, second_to_swivel: -0.45}):
        swept_head = ctx.part_world_position(head)
    ctx.check(
        "arm yaw joints swing head sideways",
        straight_head is not None and swept_head is not None and abs(swept_head[1] - straight_head[1]) > 0.08,
        details=f"straight={straight_head}, swept={swept_head}",
    )

    level_aabb = ctx.part_world_aabb(head)
    with ctx.pose({swivel_to_head: 0.65}):
        tilted_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "head tilt changes head frame height envelope",
        level_aabb is not None
        and tilted_aabb is not None
        and abs((tilted_aabb[1][2] - tilted_aabb[0][2]) - (level_aabb[1][2] - level_aabb[0][2])) > 0.015,
        details=f"level={level_aabb}, tilted={tilted_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
