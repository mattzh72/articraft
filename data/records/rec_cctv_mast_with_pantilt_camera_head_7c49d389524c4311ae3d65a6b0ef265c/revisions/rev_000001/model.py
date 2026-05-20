from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BASE_PLATE_SIZE = (0.36, 0.36, 0.026)
POLE_RADIUS = 0.045
POLE_HEIGHT = 1.20
PLATE_TOP_Z = BASE_PLATE_SIZE[2]
POLE_TOP_Z = PLATE_TOP_Z + POLE_HEIGHT
TILT_ORIGIN = (0.265, 0.0, 0.078)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_cctv_pole")

    galvanized = model.material("galvanized_steel", rgba=(0.56, 0.59, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.13, 0.14, 0.15, 1.0))
    weathered = model.material("weathered_bolts", rgba=(0.36, 0.37, 0.36, 1.0))
    white = model.material("camera_white", rgba=(0.92, 0.93, 0.90, 1.0))
    smoked = model.material("smoked_dome", rgba=(0.03, 0.04, 0.055, 0.72))
    glass_black = model.material("black_lens", rgba=(0.0, 0.0, 0.0, 1.0))

    base = model.part("base")
    base.visual(
        Box(BASE_PLATE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_SIZE[2] / 2.0)),
        material=galvanized,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=POLE_RADIUS, length=POLE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PLATE_TOP_Z + POLE_HEIGHT / 2.0)),
        material=galvanized,
        name="pole",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, PLATE_TOP_Z + 0.015)),
        material=galvanized,
        name="weld_collar",
    )
    for index, (sx, sy) in enumerate(
        [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
    ):
        base.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(sx * 0.125, sy * 0.125, PLATE_TOP_Z + 0.003)),
            material=weathered,
            name=f"anchor_washer_{index}",
        )
        base.visual(
            Cylinder(radius=0.0075, length=0.032),
            origin=Origin(xyz=(sx * 0.125, sy * 0.125, PLATE_TOP_Z + 0.014)),
            material=weathered,
            name=f"anchor_bolt_{index}",
        )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.074, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=galvanized,
        name="pan_bearing",
    )
    yoke.visual(
        Cylinder(radius=0.034, length=0.112),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=galvanized,
        name="neck",
    )
    yoke.visual(
        Box((0.250, 0.170, 0.034)),
        origin=Origin(xyz=(0.125, 0.0, 0.143)),
        material=galvanized,
        name="front_arm",
    )
    yoke.visual(
        Box((0.078, 0.018, 0.130)),
        origin=Origin(xyz=(TILT_ORIGIN[0], -0.078, 0.078)),
        material=galvanized,
        name="yoke_cheek_0",
    )
    yoke.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(
            xyz=(TILT_ORIGIN[0], -0.078, TILT_ORIGIN[2]),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="tilt_boss_0",
    )
    yoke.visual(
        Box((0.078, 0.018, 0.130)),
        origin=Origin(xyz=(TILT_ORIGIN[0], 0.078, 0.078)),
        material=galvanized,
        name="yoke_cheek_1",
    )
    yoke.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(
            xyz=(TILT_ORIGIN[0], 0.078, TILT_ORIGIN[2]),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="tilt_boss_1",
    )

    camera = model.part("camera")
    camera.visual(
        Cylinder(radius=0.016, length=0.138),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="tilt_trunnion",
    )
    camera.visual(
        Cylinder(radius=0.068, length=0.026),
        origin=Origin(xyz=(0.030, 0.0, -0.020)),
        material=white,
        name="upper_housing",
    )
    camera.visual(
        mesh_from_geometry(
            DomeGeometry(radius=0.060, radial_segments=40, height_segments=12)
            .rotate_x(math.pi)
            .translate(0.030, 0.0, -0.033),
            "smoked_camera_dome",
        ),
        origin=Origin(),
        material=smoked,
        name="dome",
    )
    camera.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.072, 0.0, -0.055)),
        material=glass_black,
        name="lens",
    )

    model.articulation(
        "base_to_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, POLE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "yoke_to_camera",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=camera,
        origin=Origin(xyz=TILT_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=-0.80, upper=0.80),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    camera = object_model.get_part("camera")
    pan = object_model.get_articulation("base_to_yoke")
    tilt = object_model.get_articulation("yoke_to_camera")

    ctx.expect_gap(
        yoke,
        base,
        axis="z",
        positive_elem="pan_bearing",
        negative_elem="pole",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan bearing seats on pole top",
    )
    ctx.expect_contact(
        camera,
        yoke,
        elem_a="tilt_trunnion",
        elem_b="tilt_boss_0",
        contact_tol=0.002,
        name="trunnion meets one yoke boss",
    )
    ctx.expect_contact(
        camera,
        yoke,
        elem_a="tilt_trunnion",
        elem_b="tilt_boss_1",
        contact_tol=0.002,
        name="trunnion meets opposite yoke boss",
    )

    rest_camera_pos = ctx.part_world_position(camera)
    with ctx.pose({pan: 0.75}):
        panned_camera_pos = ctx.part_world_position(camera)
    ctx.check(
        "pan joint swings camera around pole",
        rest_camera_pos is not None
        and panned_camera_pos is not None
        and abs(panned_camera_pos[1] - rest_camera_pos[1]) > 0.08,
        details=f"rest={rest_camera_pos}, panned={panned_camera_pos}",
    )

    rest_dome_aabb = ctx.part_element_world_aabb(camera, elem="dome")
    with ctx.pose({tilt: 0.55}):
        tilted_dome_aabb = ctx.part_element_world_aabb(camera, elem="dome")
    ctx.check(
        "tilt joint rotates dome housing",
        rest_dome_aabb is not None
        and tilted_dome_aabb is not None
        and abs(float(tilted_dome_aabb[1][0] - rest_dome_aabb[1][0])) > 0.015,
        details=f"rest={rest_dome_aabb}, tilted={tilted_dome_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
