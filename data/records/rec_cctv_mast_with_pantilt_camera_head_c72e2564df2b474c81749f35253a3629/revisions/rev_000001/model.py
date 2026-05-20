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
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parking_cctv_overhead_mount")

    galvanized = model.material("galvanized_steel", rgba=(0.42, 0.45, 0.45, 1.0))
    off_white = model.material("powder_coated_white", rgba=(0.86, 0.88, 0.86, 1.0))
    dark = model.material("black_hardware", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    smoke = model.material("smoked_polycarbonate", rgba=(0.03, 0.04, 0.05, 0.42))
    lens = model.material("glossy_lens", rgba=(0.0, 0.015, 0.03, 0.92))

    beam_bracket = model.part("beam_bracket")
    beam_bracket.visual(
        Box((0.90, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=galvanized,
        name="beam_box",
    )
    beam_bracket.visual(
        Box((0.30, 0.22, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=galvanized,
        name="saddle_plate",
    )
    for index, y in enumerate((-0.091, 0.091)):
        beam_bracket.visual(
            Box((0.30, 0.014, 0.116)),
            origin=Origin(xyz=(0.0, y, 0.047)),
            material=galvanized,
            name=f"side_strap_{index}",
        )
    for index, (x, y) in enumerate(
        ((-0.105, -0.075), (-0.105, 0.075), (0.105, -0.075), (0.105, 0.075))
    ):
        beam_bracket.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, y, -0.006)),
            material=dark,
            name=f"clamp_bolt_{index}",
        )

    drop_rod = model.part("drop_rod")
    drop_rod.visual(
        Cylinder(radius=0.058, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=galvanized,
        name="top_flange",
    )
    drop_rod.visual(
        Cylinder(radius=0.022, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, -0.226)),
        material=galvanized,
        name="rod",
    )
    drop_rod.visual(
        Cylinder(radius=0.062, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.4535)),
        material=galvanized,
        name="bottom_collar",
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.066, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=dark,
        name="pan_bearing",
    )
    pan_head.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.066, tube=0.004, radial_segments=40, tubular_segments=10),
            "bearing_seal_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=rubber,
        name="bearing_seal",
    )
    pan_head.visual(
        Cylinder(radius=0.092, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=off_white,
        name="pan_motor",
    )
    pan_head.visual(
        Cylinder(radius=0.104, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.110)),
        material=off_white,
        name="yoke_bridge",
    )
    pan_head.visual(
        Box((0.130, 0.018, 0.192)),
        origin=Origin(xyz=(0.0, -0.106, -0.212)),
        material=off_white,
        name="yoke_arm_0",
    )
    pan_head.visual(
        Box((0.130, 0.018, 0.192)),
        origin=Origin(xyz=(0.0, 0.106, -0.212)),
        material=off_white,
        name="yoke_arm_1",
    )

    tilt_housing = model.part("tilt_housing")
    tilt_housing.visual(
        Cylinder(radius=0.011, length=0.246),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="tilt_axle",
    )
    upper_cap = DomeGeometry(radius=0.091, radial_segments=44, height_segments=10, closed=True)
    upper_cap.scale(1.0, 1.0, 0.36)
    tilt_housing.visual(
        mesh_from_geometry(upper_cap, "tilt_upper_cap"),
        origin=Origin(),
        material=off_white,
        name="upper_cap",
    )
    lower_dome = DomeGeometry(radius=0.087, radial_segments=52, height_segments=14, closed=True)
    lower_dome.rotate_x(math.pi)
    tilt_housing.visual(
        mesh_from_geometry(lower_dome, "smoked_lower_dome"),
        origin=Origin(),
        material=smoke,
        name="smoked_dome",
    )
    tilt_housing.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.089, tube=0.004, radial_segments=48, tubular_segments=10),
            "dome_rim_mesh",
        ),
        origin=Origin(),
        material=rubber,
        name="dome_rim",
    )
    tilt_housing.visual(
        Box((0.052, 0.038, 0.056)),
        origin=Origin(xyz=(0.026, 0.0, -0.030)),
        material=dark,
        name="inner_cradle",
    )
    tilt_housing.visual(
        Cylinder(radius=0.020, length=0.038),
        origin=Origin(xyz=(0.065, 0.0, -0.046), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="lens_barrel",
    )
    tilt_housing.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(0.086, 0.0, -0.046), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens,
        name="lens_glass",
    )

    model.articulation(
        "bracket_to_drop",
        ArticulationType.FIXED,
        parent=beam_bracket,
        child=drop_rod,
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
    )
    model.articulation(
        "drop_to_pan",
        ArticulationType.CONTINUOUS,
        parent=drop_rod,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, -0.471)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.4),
    )
    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_housing,
        origin=Origin(xyz=(0.0, 0.0, -0.215)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=1.0, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    beam_bracket = object_model.get_part("beam_bracket")
    drop_rod = object_model.get_part("drop_rod")
    pan_head = object_model.get_part("pan_head")
    tilt_housing = object_model.get_part("tilt_housing")
    pan = object_model.get_articulation("drop_to_pan")
    tilt = object_model.get_articulation("pan_to_tilt")

    ctx.allow_overlap(
        pan_head,
        tilt_housing,
        elem_a="yoke_arm_0",
        elem_b="tilt_axle",
        reason="The tilt axle is intentionally captured through the yoke cheek bore.",
    )
    ctx.allow_overlap(
        pan_head,
        tilt_housing,
        elem_a="yoke_arm_1",
        elem_b="tilt_axle",
        reason="The tilt axle is intentionally captured through the yoke cheek bore.",
    )

    ctx.check(
        "pan bearing is continuous",
        pan.articulation_type == ArticulationType.CONTINUOUS and pan.axis == (0.0, 0.0, 1.0),
        details=f"type={pan.articulation_type}, axis={pan.axis}",
    )
    ctx.check(
        "tilt joint has realistic stops",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.axis == (0.0, 1.0, 0.0)
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < -0.5
        and tilt.motion_limits.upper > 0.5,
        details=f"type={tilt.articulation_type}, axis={tilt.axis}, limits={tilt.motion_limits}",
    )

    ctx.expect_contact(
        drop_rod,
        beam_bracket,
        elem_a="top_flange",
        elem_b="saddle_plate",
        contact_tol=0.0005,
        name="drop rod flange is bolted to saddle plate",
    )
    ctx.expect_contact(
        pan_head,
        drop_rod,
        elem_a="pan_bearing",
        elem_b="bottom_collar",
        contact_tol=0.0005,
        name="pan bearing seats under drop rod collar",
    )
    ctx.expect_gap(
        pan_head,
        tilt_housing,
        axis="y",
        positive_elem="yoke_arm_1",
        negative_elem="smoked_dome",
        min_gap=0.004,
        name="positive yoke cheek clears smoked dome",
    )
    ctx.expect_gap(
        tilt_housing,
        pan_head,
        axis="y",
        positive_elem="smoked_dome",
        negative_elem="yoke_arm_0",
        min_gap=0.004,
        name="negative yoke cheek clears smoked dome",
    )
    ctx.expect_overlap(
        tilt_housing,
        pan_head,
        axes="y",
        elem_a="tilt_axle",
        elem_b="yoke_arm_0",
        min_overlap=0.010,
        name="tilt axle is retained in negative cheek",
    )
    ctx.expect_overlap(
        tilt_housing,
        pan_head,
        axes="y",
        elem_a="tilt_axle",
        elem_b="yoke_arm_1",
        min_overlap=0.010,
        name="tilt axle is retained in positive cheek",
    )

    rest_lens_aabb = ctx.part_element_world_aabb(tilt_housing, elem="lens_glass")
    with ctx.pose({pan: math.pi / 2.0}):
        panned_lens_aabb = ctx.part_element_world_aabb(tilt_housing, elem="lens_glass")
    with ctx.pose({tilt: 0.55}):
        tilted_lens_aabb = ctx.part_element_world_aabb(tilt_housing, elem="lens_glass")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    rest_center = _aabb_center(rest_lens_aabb)
    pan_center = _aabb_center(panned_lens_aabb)
    tilt_center = _aabb_center(tilted_lens_aabb)
    ctx.check(
        "pan rotates lens around drop rod",
        rest_center is not None
        and pan_center is not None
        and abs(rest_center[0] - pan_center[1]) < 0.010
        and abs(pan_center[0]) < 0.020,
        details=f"rest={rest_center}, panned={pan_center}",
    )
    ctx.check(
        "tilt joint changes lens elevation",
        rest_center is not None
        and tilt_center is not None
        and abs(tilt_center[2] - rest_center[2]) > 0.020,
        details=f"rest={rest_center}, tilted={tilt_center}",
    )

    return ctx.report()


object_model = build_object_model()
