from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pan_sensor_base")

    dark_anodized = model.material("dark_anodized", color=(0.055, 0.060, 0.065, 1.0))
    satin_black = model.material("satin_black", color=(0.010, 0.012, 0.014, 1.0))
    machined_edge = model.material("machined_edge", color=(0.55, 0.57, 0.58, 1.0))
    warm_plate = model.material("warm_plate", color=(0.18, 0.20, 0.22, 1.0))
    rubber = model.material("rubber", color=(0.018, 0.018, 0.016, 1.0))

    base = model.part("base")

    # One lathed root casting keeps the squat lower body and stepped fixed
    # collar as a single grounded piece, with shoulders that visually define the
    # vertical yaw axis before the rotating member is added.
    base_profile = [
        (0.000, 0.000),
        (0.150, 0.000),
        (0.170, 0.008),
        (0.180, 0.020),
        (0.176, 0.034),
        (0.142, 0.046),
        (0.112, 0.054),
        (0.112, 0.069),
        (0.094, 0.077),
        (0.094, 0.096),
        (0.080, 0.103),
        (0.080, 0.119),
        (0.000, 0.119),
    ]
    base.visual(
        mesh_from_geometry(LatheGeometry(base_profile, segments=96), "base_body"),
        name="body",
        material=dark_anodized,
    )

    # Thin contrasting shoulders make the collar stack read as machined metal
    # instead of a single lump of plastic.
    base.visual(
        Cylinder(radius=0.148, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        name="lower_shoulder",
        material=machined_edge,
    )
    base.visual(
        Cylinder(radius=0.086, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.1205)),
        name="bearing_land",
        material=satin_black,
    )

    # Four short ribs are local reinforcements from the lower body to the collar,
    # not separate floating fins.
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        base.visual(
            Box((0.094, 0.018, 0.020)),
            origin=Origin(
                xyz=(0.119 * math.cos(angle), 0.119 * math.sin(angle), 0.064),
                rpy=(0.0, 0.0, angle),
            ),
            name=f"rib_{index}",
            material=dark_anodized,
        )

    # Low, proud stop pads live outside the rotating skirt's sweep and below the
    # top plate, so the yaw path stays clear while the base still reads like a
    # real limited pan mechanism.
    stop_angle_0 = math.radians(35.0)
    base.visual(
        Box((0.070, 0.020, 0.014)),
        origin=Origin(
            xyz=(0.115 * math.cos(stop_angle_0), 0.115 * math.sin(stop_angle_0), 0.086),
            rpy=(0.0, 0.0, stop_angle_0),
        ),
        name="stop_pad_0",
        material=satin_black,
    )
    stop_angle_1 = math.radians(145.0)
    base.visual(
        Box((0.070, 0.020, 0.014)),
        origin=Origin(
            xyz=(0.115 * math.cos(stop_angle_1), 0.115 * math.sin(stop_angle_1), 0.086),
            rpy=(0.0, 0.0, stop_angle_1),
        ),
        name="stop_pad_1",
        material=satin_black,
    )

    base.visual(
        Cylinder(radius=0.164, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        name="foot_ring",
        material=rubber,
    )

    top_plate = model.part("top_plate")

    # The child part frame is the yaw axis at the fixed bearing land.  All
    # rotating geometry is local to this axis and clears the fixed collar by a
    # small bearing gap.
    top_plate.visual(
        Cylinder(radius=0.068, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        name="rotor_skirt",
        material=warm_plate,
    )
    top_plate.visual(
        Cylinder(radius=0.058, length=0.023),
        origin=Origin(xyz=(0.0, 0.0, 0.0305)),
        name="support_boss",
        material=warm_plate,
    )

    plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(rounded_rect_profile(0.240, 0.078, 0.012, corner_segments=8), 0.016),
        "top_plate_panel",
    )
    top_plate.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        name="plate",
        material=warm_plate,
    )

    # Underside saddles broaden the support patch under the narrow plate and
    # physically tie the rectangular sensor platform into the round collar.
    top_plate.visual(
        Box((0.150, 0.044, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        name="under_saddle",
        material=warm_plate,
    )
    for index, angle in enumerate((0.0, math.pi)):
        top_plate.visual(
            Box((0.072, 0.014, 0.016)),
            origin=Origin(
                xyz=(0.052 * math.cos(angle), 0.052 * math.sin(angle), 0.035),
                rpy=(0.0, 0.0, angle),
            ),
            name=f"top_rib_{index}",
            material=warm_plate,
        )

    top_plate.visual(
        Cylinder(radius=0.046, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        name="retainer_cap",
        material=machined_edge,
    )
    top_plate.visual(
        Cylinder(radius=0.014, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0685)),
        name="center_plug",
        material=satin_black,
    )

    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-2.35, upper=2.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    top_plate = object_model.get_part("top_plate")
    yaw = object_model.get_articulation("yaw_joint")

    ctx.expect_origin_distance(
        base,
        top_plate,
        axes="xy",
        max_dist=0.001,
        name="yaw origins share the vertical centerline",
    )
    ctx.expect_gap(
        top_plate,
        base,
        axis="z",
        positive_elem="rotor_skirt",
        negative_elem="bearing_land",
        min_gap=0.0,
        max_gap=0.001,
        name="rotating skirt seats on the fixed bearing land",
    )
    ctx.expect_overlap(
        top_plate,
        base,
        axes="xy",
        elem_a="rotor_skirt",
        elem_b="bearing_land",
        min_overlap=0.120,
        name="collar layers are concentric around the yaw axis",
    )

    rest_aabb = ctx.part_element_world_aabb(top_plate, elem="plate")
    with ctx.pose({yaw: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(top_plate, elem="plate")

    if rest_aabb is not None and turned_aabb is not None:
        rest_x = rest_aabb[1][0] - rest_aabb[0][0]
        rest_y = rest_aabb[1][1] - rest_aabb[0][1]
        turned_x = turned_aabb[1][0] - turned_aabb[0][0]
        turned_y = turned_aabb[1][1] - turned_aabb[0][1]
    else:
        rest_x = rest_y = turned_x = turned_y = 0.0

    ctx.check(
        "top plate yaws about the centerline",
        rest_x > rest_y * 2.0 and turned_y > turned_x * 2.0,
        details=f"rest=({rest_x:.3f}, {rest_y:.3f}), turned=({turned_x:.3f}, {turned_y:.3f})",
    )

    for pose_name, angle in (("lower", -2.35), ("upper", 2.35)):
        with ctx.pose({yaw: angle}):
            ctx.expect_gap(
                top_plate,
                base,
                axis="z",
                positive_elem="plate",
                negative_elem="stop_pad_0",
                min_gap=0.060,
                name=f"top plate clears front stop at {pose_name} yaw limit",
            )
            ctx.expect_gap(
                top_plate,
                base,
                axis="z",
                positive_elem="plate",
                negative_elem="stop_pad_1",
                min_gap=0.060,
                name=f"top plate clears rear stop at {pose_name} yaw limit",
            )

    return ctx.report()


object_model = build_object_model()
