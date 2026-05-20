from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="transportable_satellite_dish")

    dark = model.material("dark_powder_coat", rgba=(0.08, 0.09, 0.10, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    reflector_mat = model.material("matte_aluminum_reflector", rgba=(0.74, 0.77, 0.76, 1.0))
    frame_mat = model.material("zinc_frame", rgba=(0.22, 0.24, 0.25, 1.0))
    handle_mat = model.material("red_crank_grip", rgba=(0.72, 0.08, 0.04, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.70, 0.56, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark,
        name="base_box",
    )
    base.visual(
        Box((0.42, 0.08, 0.035)),
        origin=Origin(xyz=(0.0, -0.285, 0.075)),
        material=black,
        name="front_carry_grip",
    )
    base.visual(
        Cylinder(radius=0.235, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=frame_mat,
        name="fixed_bearing_race",
    )

    azimuth = model.part("azimuth")
    azimuth.visual(
        Cylinder(radius=0.220, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=frame_mat,
        name="rotating_bearing",
    )
    azimuth.visual(
        Box((0.48, 0.48, 0.160)),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=dark,
        name="azimuth_box",
    )
    azimuth.visual(
        Box((0.52, 0.76, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=frame_mat,
        name="yoke_bridge",
    )
    azimuth.visual(
        Box((0.140, 0.050, 0.450)),
        origin=Origin(xyz=(0.0, -0.34, 0.455)),
        material=frame_mat,
        name="yoke_side_0",
    )
    azimuth.visual(
        mesh_from_geometry(TorusGeometry(radius=0.046, tube=0.006, radial_segments=14, tubular_segments=36).rotate_x(math.pi / 2.0), "yoke_side_0_bore_face"),
        origin=Origin(xyz=(0.0, -0.369, 0.540)),
        material=dark,
        name="yoke_side_0_bore_face",
    )
    azimuth.visual(
        Box((0.140, 0.050, 0.450)),
        origin=Origin(xyz=(0.0, 0.34, 0.455)),
        material=frame_mat,
        name="yoke_side_1",
    )
    azimuth.visual(
        mesh_from_geometry(TorusGeometry(radius=0.046, tube=0.006, radial_segments=14, tubular_segments=36).rotate_x(math.pi / 2.0), "yoke_side_1_bore_face"),
        origin=Origin(xyz=(0.0, 0.369, 0.540)),
        material=dark,
        name="yoke_side_1_bore_face",
    )
    azimuth.visual(
        Cylinder(radius=0.042, length=0.060),
        origin=Origin(xyz=(-0.180, 0.270, 0.130), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="crank_bushing",
    )

    dish_frame = model.part("dish_frame")
    reflector_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.028, 0.000), (0.145, 0.040), (0.268, 0.130)],
        inner_profile=[(0.018, 0.010), (0.135, 0.050), (0.255, 0.120)],
        segments=72,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    ).rotate_y(math.pi / 2.0)
    dish_frame.visual(
        mesh_from_geometry(reflector_shell, "reflector_shell"),
        material=reflector_mat,
        name="reflector_shell",
    )
    dish_frame.visual(
        mesh_from_geometry(TorusGeometry(radius=0.268, tube=0.012, radial_segments=16, tubular_segments=72).rotate_y(math.pi / 2.0), "reflector_rim"),
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        material=frame_mat,
        name="reflector_rim",
    )
    dish_frame.visual(
        Cylinder(radius=0.052, length=0.150),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_mat,
        name="rear_hub",
    )
    dish_frame.visual(
        Cylinder(radius=0.030, length=0.740),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="elevation_pin",
    )
    dish_frame.visual(
        Box((0.028, 0.500, 0.026)),
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        material=frame_mat,
        name="rear_crossbar",
    )
    dish_frame.visual(
        Box((0.028, 0.026, 0.500)),
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        material=frame_mat,
        name="rear_upright",
    )
    dish_frame.visual(
        Box((0.045, 0.050, 0.090)),
        origin=Origin(xyz=(0.130, 0.0, -0.240)),
        material=frame_mat,
        name="feed_arm_mount",
    )
    dish_frame.visual(
        Box((0.540, 0.034, 0.026)),
        origin=Origin(xyz=(0.385, 0.0, -0.220)),
        material=frame_mat,
        name="feed_arm",
    )
    dish_frame.visual(
        Cylinder(radius=0.038, length=0.090),
        origin=Origin(xyz=(0.665, 0.0, -0.220), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="feed_horn",
    )

    crank = model.part("side_crank")
    crank.visual(
        Cylinder(radius=0.016, length=0.085),
        origin=Origin(xyz=(0.0, 0.0425, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=frame_mat,
        name="crank_shaft",
    )
    crank.visual(
        Box((0.026, 0.060, 0.145)),
        origin=Origin(xyz=(0.0, 0.115, -0.072)),
        material=frame_mat,
        name="crank_throw",
    )
    crank.visual(
        Cylinder(radius=0.022, length=0.090),
        origin=Origin(xyz=(0.0, 0.175, -0.145), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="crank_grip",
    )

    model.articulation(
        "base_to_azimuth",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=azimuth,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2),
    )
    model.articulation(
        "azimuth_to_dish_frame",
        ArticulationType.REVOLUTE,
        parent=azimuth,
        child=dish_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.540)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.8, lower=-0.35, upper=0.80),
    )
    model.articulation(
        "azimuth_to_side_crank",
        ArticulationType.CONTINUOUS,
        parent=azimuth,
        child=crank,
        origin=Origin(xyz=(-0.180, 0.300, 0.130)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    azimuth = object_model.get_part("azimuth")
    dish_frame = object_model.get_part("dish_frame")
    crank = object_model.get_part("side_crank")
    elevation = object_model.get_articulation("azimuth_to_dish_frame")
    crank_joint = object_model.get_articulation("azimuth_to_side_crank")

    for cheek in ("yoke_side_0", "yoke_side_1"):
        ctx.allow_overlap(
            azimuth,
            dish_frame,
            elem_a=cheek,
            elem_b="elevation_pin",
            reason="The trunnion pin is intentionally captured in the yoke cheek bore proxy.",
        )
        ctx.expect_overlap(
            azimuth,
            dish_frame,
            axes="xz",
            elem_a=cheek,
            elem_b="elevation_pin",
            min_overlap=0.045,
            name=f"{cheek} clips the elevation pin",
        )

    ctx.expect_contact(
        azimuth,
        crank,
        elem_a="crank_bushing",
        elem_b="crank_shaft",
        contact_tol=0.002,
        name="side crank shaft is seated in the base bushing",
    )

    rest_feed_aabb = ctx.part_element_world_aabb(dish_frame, elem="feed_horn")
    with ctx.pose({elevation: 0.80}):
        raised_feed_aabb = ctx.part_element_world_aabb(dish_frame, elem="feed_horn")
        ctx.expect_overlap(
            azimuth,
            dish_frame,
            axes="xz",
            elem_a="yoke_side_0",
            elem_b="elevation_pin",
            min_overlap=0.040,
            name="tilted dish remains clipped in the yoke",
        )
    ctx.check(
        "positive elevation raises the feed end",
        rest_feed_aabb is not None
        and raised_feed_aabb is not None
        and raised_feed_aabb[1][2] > rest_feed_aabb[1][2] + 0.20,
        details=f"rest={rest_feed_aabb}, raised={raised_feed_aabb}",
    )

    rest_crank_aabb = ctx.part_world_aabb(crank)
    with ctx.pose({crank_joint: math.pi}):
        rotated_crank_aabb = ctx.part_world_aabb(crank)
    ctx.check(
        "side crank rotates about its local shaft",
        rest_crank_aabb is not None
        and rotated_crank_aabb is not None
        and rotated_crank_aabb[0][2] > rest_crank_aabb[0][2] + 0.12,
        details=f"rest={rest_crank_aabb}, rotated={rotated_crank_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
