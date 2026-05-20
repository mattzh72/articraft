from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_laptop_stand")

    dark = model.material("dark", rgba=(0.18, 0.18, 0.20, 1.0))
    graphite = model.material("graphite", rgba=(0.26, 0.27, 0.30, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.75, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    base_w = 0.260
    base_d = 0.240
    base_t = 0.008
    pedestal_w = 0.090
    pedestal_d = 0.060
    pedestal_t = 0.020
    mast_y = -0.082

    mast_w = 0.038
    mast_d = 0.050
    mast_h = 0.310

    carriage_clear_x = 0.000
    carriage_clear_y = 0.000
    carriage_wall = 0.012
    carriage_h = 0.075
    carriage_inner_w = mast_w + 2.0 * carriage_clear_x
    carriage_inner_d = mast_d + 2.0 * carriage_clear_y
    carriage_outer_w = carriage_inner_w + 2.0 * carriage_wall
    carriage_outer_d = carriage_inner_d + 2.0 * carriage_wall
    carriage_rest_z = 0.205
    carriage_head_w = 0.160
    carriage_head_d = 0.034
    carriage_head_t = 0.014
    ear_t = 0.018
    ear_d = 0.020
    ear_h = 0.040
    ear_x = 0.066
    hinge_y = carriage_outer_d / 2.0 + 0.017
    hinge_z = carriage_h + ear_h / 2.0

    tray_w = 0.280
    tray_d = 0.240
    tray_t = 0.012
    rear_beam_w = 0.108
    rear_beam_d = 0.028
    rear_beam_t = 0.022
    rib_w = 0.018
    rib_d = 0.190
    rib_t = 0.020
    rib_x = 0.052
    lip_w = 0.052
    lip_d = 0.014
    lip_h = 0.018
    lip_x = 0.084

    base = model.part("base")
    base.visual(
        Box((base_w, base_d, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t / 2.0)),
        material=dark,
        name="plate",
    )
    base.visual(
        Box((pedestal_w, pedestal_d, pedestal_t)),
        origin=Origin(xyz=(0.0, mast_y, base_t + pedestal_t / 2.0)),
        material=graphite,
        name="pedestal",
    )

    mast = model.part("mast")
    mast.visual(
        Box((mast_w, mast_d, mast_h)),
        origin=Origin(xyz=(0.0, 0.0, mast_h / 2.0)),
        material=graphite,
        name="mast_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((carriage_wall, carriage_outer_d, carriage_h)),
        origin=Origin(
            xyz=(-(carriage_inner_w / 2.0 + carriage_wall / 2.0), 0.0, carriage_h / 2.0)
        ),
        material=graphite,
        name="side_wall_0",
    )
    carriage.visual(
        Box((carriage_wall, carriage_outer_d, carriage_h)),
        origin=Origin(
            xyz=((carriage_inner_w / 2.0 + carriage_wall / 2.0), 0.0, carriage_h / 2.0)
        ),
        material=graphite,
        name="side_wall_1",
    )
    carriage.visual(
        Box((carriage_inner_w, carriage_wall, carriage_h)),
        origin=Origin(
            xyz=(0.0, carriage_inner_d / 2.0 + carriage_wall / 2.0, carriage_h / 2.0)
        ),
        material=graphite,
        name="front_bridge",
    )
    carriage.visual(
        Box((carriage_inner_w, carriage_wall, carriage_h)),
        origin=Origin(
            xyz=(0.0, -(carriage_inner_d / 2.0 + carriage_wall / 2.0), carriage_h / 2.0)
        ),
        material=graphite,
        name="rear_bridge",
    )
    carriage.visual(
        Box((carriage_head_w, carriage_head_d, carriage_head_t)),
        origin=Origin(
            xyz=(0.0, carriage_outer_d / 2.0 + 0.014, carriage_h - carriage_head_t / 2.0)
        ),
        material=graphite,
        name="head_plate",
    )
    carriage.visual(
        Box((ear_t, ear_d, ear_h)),
        origin=Origin(xyz=(-ear_x, hinge_y, hinge_z)),
        material=graphite,
        name="hinge_ear_0",
    )
    carriage.visual(
        Box((ear_t, ear_d, ear_h)),
        origin=Origin(xyz=(ear_x, hinge_y, hinge_z)),
        material=graphite,
        name="hinge_ear_1",
    )

    tray = model.part("tray")
    tray.visual(
        Cylinder(radius=0.007, length=0.086),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    tray.visual(
        Box((rear_beam_w, rear_beam_d, rear_beam_t)),
        origin=Origin(xyz=(0.0, 0.016, 0.006)),
        material=aluminum,
        name="rear_beam",
    )
    tray.visual(
        Box((rib_w, rib_d, rib_t)),
        origin=Origin(xyz=(-rib_x, 0.112, 0.008)),
        material=aluminum,
        name="rib_0",
    )
    tray.visual(
        Box((rib_w, rib_d, rib_t)),
        origin=Origin(xyz=(rib_x, 0.112, 0.008)),
        material=aluminum,
        name="rib_1",
    )
    tray.visual(
        Box((tray_w, tray_d, tray_t)),
        origin=Origin(xyz=(0.0, tray_d / 2.0, 0.024)),
        material=aluminum,
        name="tray_panel",
    )
    tray.visual(
        Box((lip_w, lip_d, lip_h)),
        origin=Origin(xyz=(-lip_x, tray_d - lip_d / 2.0, 0.038)),
        material=rubber,
        name="front_lip_0",
    )
    tray.visual(
        Box((lip_w, lip_d, lip_h)),
        origin=Origin(xyz=(lip_x, tray_d - lip_d / 2.0, 0.038)),
        material=rubber,
        name="front_lip_1",
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.FIXED,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, mast_y, base_t + pedestal_t)),
    )
    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, carriage_rest_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.10,
            lower=0.0,
            upper=0.055,
        ),
    )
    model.articulation(
        "carriage_to_tray",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tray,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-0.25,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    tray = object_model.get_part("tray")
    mast_slide = object_model.get_articulation("mast_to_carriage")
    tray_tilt = object_model.get_articulation("carriage_to_tray")

    visual_names = {visual.name for visual in tray.visuals}
    ctx.check(
        "tray has two retaining lips",
        {"front_lip_0", "front_lip_1"}.issubset(visual_names),
        details=f"tray visuals={sorted(name for name in visual_names if name is not None)}",
    )

    ctx.expect_origin_distance(
        carriage,
        mast,
        axes="xy",
        max_dist=0.001,
        name="carriage stays centered on the mast axis",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        min_overlap=0.070,
        name="carriage remains engaged on the mast at rest",
    )
    ctx.expect_gap(
        tray,
        carriage,
        axis="z",
        positive_elem="tray_panel",
        negative_elem="head_plate",
        min_gap=0.025,
        name="tray panel stays above the carriage head",
    )
    ctx.expect_overlap(
        tray,
        carriage,
        axes="x",
        elem_a="rear_beam",
        elem_b="head_plate",
        min_overlap=0.100,
        name="tray hinge beam spans the carriage head",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({mast_slide: 0.055}):
        ctx.expect_origin_distance(
            carriage,
            mast,
            axes="xy",
            max_dist=0.001,
            name="raised carriage stays centered on the mast axis",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            min_overlap=0.049,
            name="raised carriage still overlaps the mast vertically",
        )
        raised_carriage_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage slides upward along the mast",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.045,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )

    rest_lip_aabb = ctx.part_element_world_aabb(tray, elem="front_lip_0")
    with ctx.pose({tray_tilt: 0.60}):
        tilted_lip_aabb = ctx.part_element_world_aabb(tray, elem="front_lip_0")

    rest_top_z = rest_lip_aabb[1][2] if rest_lip_aabb is not None else None
    tilted_top_z = tilted_lip_aabb[1][2] if tilted_lip_aabb is not None else None
    ctx.check(
        "tray tilts upward about the horizontal hinge",
        rest_top_z is not None and tilted_top_z is not None and tilted_top_z > rest_top_z + 0.050,
        details=f"rest_top_z={rest_top_z}, tilted_top_z={tilted_top_z}",
    )

    return ctx.report()


object_model = build_object_model()
