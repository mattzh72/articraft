from __future__ import annotations

import math

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


STEEL = Material("galvanized_dark_steel", rgba=(0.34, 0.36, 0.36, 1.0))
PAINT = Material("matte_black_powdercoat", rgba=(0.02, 0.025, 0.025, 1.0))
GLASS = Material("warm_floodlight_glass", rgba=(1.0, 0.86, 0.46, 0.82))
LENS = Material("smoked_camera_glass", rgba=(0.02, 0.035, 0.045, 0.95))
RUBBER = Material("dark_rubber_gasket", rgba=(0.005, 0.005, 0.004, 1.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stadium_floodlight_cctv_arm")

    mast = model.part("mast")

    # Ground-mounted floodlight pole: large base plate, anchor bolts and a tall
    # round steel mast sized like a small stadium lighting standard.
    mast.visual(
        Box((0.58, 0.58, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=STEEL,
        name="base_plate",
    )
    mast.visual(
        Cylinder(radius=0.105, length=7.55),
        origin=Origin(xyz=(0.0, 0.0, 3.845)),
        material=STEEL,
        name="tall_pole",
    )
    mast.visual(
        Cylinder(radius=0.125, length=0.11),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=STEEL,
        name="base_collar",
    )
    for ix, x in enumerate((-0.205, 0.205)):
        for iy, y in enumerate((-0.205, 0.205)):
            mast.visual(
                Cylinder(radius=0.018, length=0.045),
                origin=Origin(xyz=(x, y, 0.092)),
                material=STEEL,
                name=f"anchor_bolt_{ix}_{iy}",
            )

    # Fixed stadium crossbar with two static floodlight modules hanging below it.
    mast.visual(
        Cylinder(radius=0.055, length=3.10),
        origin=Origin(xyz=(0.0, 0.0, 7.34), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="fixed_crossbar",
    )
    mast.visual(
        Cylinder(radius=0.075, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 7.34), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="crossbar_collar",
    )
    mast.visual(
        Cylinder(radius=0.115, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 7.655)),
        material=STEEL,
        name="top_cap",
    )

    for i, x in enumerate((-0.72, 0.72)):
        mast.visual(
            Cylinder(radius=0.022, length=0.12),
            origin=Origin(xyz=(x, 0.055, 7.255)),
            material=STEEL,
            name=f"floodlight_stem_{i}",
        )
        mast.visual(
            Box((0.50, 0.105, 0.32)),
            origin=Origin(xyz=(x, 0.085, 7.06)),
            material=PAINT,
            name=f"floodlight_housing_{i}",
        )
        mast.visual(
            Box((0.41, 0.014, 0.235)),
            origin=Origin(xyz=(x, 0.143, 7.06)),
            material=GLASS,
            name=f"floodlight_glass_{i}",
        )
        for fin in range(3):
            mast.visual(
                Box((0.035, 0.045, 0.255)),
                origin=Origin(xyz=(x + (fin - 1) * 0.13, 0.028, 7.06)),
                material=STEEL,
                name=f"floodlight_fin_{i}_{fin}",
            )

    # CCTV branch arm is a fixed welded outrigger, not an articulated member.
    mast.visual(
        Cylinder(radius=0.047, length=1.45),
        origin=Origin(xyz=(0.0, 0.725, 6.08), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=STEEL,
        name="camera_arm",
    )
    mast.visual(
        Cylinder(radius=0.035, length=1.24),
        origin=Origin(
            xyz=(0.0, 0.60, 5.985),
            rpy=(-math.atan2(1.20, 0.25), 0.0, 0.0),
        ),
        material=STEEL,
        name="diagonal_brace",
    )
    mast.visual(
        Cylinder(radius=0.050, length=0.34),
        origin=Origin(xyz=(0.0, 1.45, 5.91)),
        material=STEEL,
        name="drop_tube",
    )
    mast.visual(
        Box((0.25, 0.22, 0.05)),
        origin=Origin(xyz=(0.0, 1.45, 5.72)),
        material=STEEL,
        name="bearing_mount_plate",
    )

    pan_bearing = model.part("pan_bearing")
    pan_bearing.visual(
        Cylinder(radius=0.115, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
        material=PAINT,
        name="pan_bearing_shell",
    )
    pan_bearing.visual(
        Cylinder(radius=0.052, length=0.19),
        origin=Origin(xyz=(0.0, 0.0, -0.195)),
        material=PAINT,
        name="pan_post",
    )
    pan_bearing.visual(
        Box((0.46, 0.14, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.305)),
        material=PAINT,
        name="yoke_bridge",
    )
    pan_bearing.visual(
        Box((0.060, 0.14, 0.34)),
        origin=Origin(xyz=(-0.18, 0.0, -0.46)),
        material=PAINT,
        name="yoke_arm_0",
    )
    pan_bearing.visual(
        Cylinder(radius=0.043, length=0.020),
        origin=Origin(xyz=(-0.1656, 0.0, -0.46), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=RUBBER,
        name="tilt_bushing_0",
    )
    pan_bearing.visual(
        Box((0.060, 0.14, 0.34)),
        origin=Origin(xyz=(0.18, 0.0, -0.46)),
        material=PAINT,
        name="yoke_arm_1",
    )
    pan_bearing.visual(
        Cylinder(radius=0.043, length=0.020),
        origin=Origin(xyz=(0.1656, 0.0, -0.46), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=RUBBER,
        name="tilt_bushing_1",
    )

    camera_head = model.part("camera_head")
    camera_head.visual(
        Cylinder(radius=0.034, length=0.300),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=STEEL,
        name="tilt_axle",
    )
    camera_head.visual(
        Box((0.215, 0.35, 0.17)),
        origin=Origin(xyz=(0.0, 0.185, 0.0)),
        material=PAINT,
        name="camera_body",
    )
    camera_head.visual(
        Box((0.285, 0.42, 0.035)),
        origin=Origin(xyz=(0.0, 0.205, 0.098)),
        material=PAINT,
        name="sunshade",
    )
    camera_head.visual(
        Cylinder(radius=0.073, length=0.095),
        origin=Origin(xyz=(0.0, 0.405, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=PAINT,
        name="lens_hood",
    )
    camera_head.visual(
        Cylinder(radius=0.058, length=0.012),
        origin=Origin(xyz=(0.0, 0.458, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=LENS,
        name="lens_glass",
    )
    camera_head.visual(
        Cylinder(radius=0.052, length=0.045),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=RUBBER,
        name="rear_gasket",
    )

    pan_joint = model.articulation(
        "camera_pan",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=pan_bearing,
        origin=Origin(xyz=(0.0, 1.45, 5.695)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.2),
    )
    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_bearing,
        child=camera_head,
        origin=Origin(xyz=(0.0, 0.0, -0.46)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.8, lower=-0.95, upper=0.75),
    )
    pan_joint.meta["description"] = "continuous vertical pan bearing at the end of the fixed CCTV arm"

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    pan_bearing = object_model.get_part("pan_bearing")
    camera_head = object_model.get_part("camera_head")
    pan = object_model.get_articulation("camera_pan")
    tilt = object_model.get_articulation("camera_tilt")

    ctx.check(
        "camera pan is continuous",
        pan.articulation_type == ArticulationType.CONTINUOUS,
        details=f"pan type is {pan.articulation_type}",
    )
    ctx.check(
        "camera tilt is limited revolute",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower <= -0.9
        and tilt.motion_limits.upper >= 0.7,
        details=f"tilt type={tilt.articulation_type}, limits={tilt.motion_limits}",
    )

    ctx.expect_contact(
        mast,
        pan_bearing,
        elem_a="bearing_mount_plate",
        elem_b="pan_bearing_shell",
        contact_tol=0.001,
        name="pan bearing seats against arm mount plate",
    )
    ctx.expect_contact(
        pan_bearing,
        camera_head,
        elem_a="yoke_arm_0",
        elem_b="tilt_axle",
        contact_tol=0.001,
        name="tilt axle reaches first yoke bushing",
    )
    ctx.expect_contact(
        pan_bearing,
        camera_head,
        elem_a="yoke_arm_1",
        elem_b="tilt_axle",
        contact_tol=0.001,
        name="tilt axle reaches second yoke bushing",
    )

    def elem_center_xy(part, elem: str) -> tuple[float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return ((aabb[0][0] + aabb[1][0]) * 0.5, (aabb[0][1] + aabb[1][1]) * 0.5)

    rest_lens = elem_center_xy(camera_head, "lens_glass")
    with ctx.pose({pan: math.pi / 2.0}):
        panned_lens = elem_center_xy(camera_head, "lens_glass")
    ctx.check(
        "pan joint rotates the camera view around the pole arm",
        rest_lens is not None
        and panned_lens is not None
        and rest_lens[1] > 1.83
        and panned_lens[0] < -0.37,
        details=f"rest_lens={rest_lens}, panned_lens={panned_lens}",
    )

    def part_center_z(part) -> float | None:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    rest_z = part_center_z(camera_head)
    with ctx.pose({tilt: 0.55}):
        up_z = part_center_z(camera_head)
    with ctx.pose({tilt: -0.70}):
        down_z = part_center_z(camera_head)
    ctx.check(
        "tilt joint pitches camera head upward and downward",
        rest_z is not None
        and up_z is not None
        and down_z is not None
        and up_z > rest_z + 0.04
        and down_z < rest_z - 0.04,
        details=f"rest_z={rest_z}, up_z={up_z}, down_z={down_z}",
    )

    return ctx.report()


object_model = build_object_model()
