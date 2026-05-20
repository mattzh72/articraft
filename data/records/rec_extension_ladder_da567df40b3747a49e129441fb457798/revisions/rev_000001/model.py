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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pontoon_boarding_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.78, 0.82, 0.82, 1.0))
    stainless = model.material("stainless_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    pontoon_blue = model.material("blue_painted_gunwale", rgba=(0.02, 0.12, 0.28, 1.0))
    tread_white = model.material("white_tread_insert", rgba=(0.90, 0.92, 0.88, 1.0))

    def cyl_x(part, name: str, radius: float, length: float, xyz, material: Material) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    def cyl_y(part, name: str, radius: float, length: float, xyz, material: Material) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    clamp = model.part("clamp_bracket")

    # A short section of pontoon gunwale is included so the clamp reads as a
    # real over-the-rail boarding ladder bracket rather than a floating hinge.
    clamp.visual(
        Box((0.62, 0.11, 0.085)),
        origin=Origin(xyz=(0.0, 0.095, 0.043)),
        material=pontoon_blue,
        name="gunwale_section",
    )
    clamp.visual(
        Box((0.57, 0.155, 0.014)),
        origin=Origin(xyz=(0.0, 0.090, 0.094)),
        material=stainless,
        name="top_clamp_plate",
    )
    for i, x in enumerate((-0.235, 0.235)):
        clamp.visual(
            Box((0.052, 0.038, 0.132)),
            origin=Origin(xyz=(x, -0.040, 0.052)),
            material=stainless,
            name=f"outboard_strap_{i}",
        )
        clamp.visual(
            Box((0.052, 0.106, 0.020)),
            origin=Origin(xyz=(x, 0.013, 0.087)),
            material=stainless,
            name=f"strap_bridge_{i}",
        )
        if i == 0:
            clamp.visual(
                Box((0.052, 0.050, 0.076)),
                origin=Origin(xyz=(x, -0.078, 0.035)),
                material=stainless,
                name="hinge_cheek_0",
            )
        else:
            clamp.visual(
                Box((0.052, 0.050, 0.076)),
                origin=Origin(xyz=(x, -0.078, 0.035)),
                material=stainless,
                name="hinge_cheek_1",
            )
        cyl_x(clamp, f"main_pin_cap_{i}", 0.018, 0.018, (x, -0.078, 0.035), stainless)
        clamp.visual(
            Box((0.105, 0.055, 0.014)),
            origin=Origin(xyz=(x, 0.095, -0.006)),
            material=rubber,
            name=f"rubber_clamp_pad_{i}",
        )
        clamp.visual(
            Cylinder(radius=0.009, length=0.132),
            origin=Origin(xyz=(x, 0.095, 0.030)),
            material=stainless,
            name=f"clamp_screw_{i}",
        )
        cyl_x(clamp, f"thumb_handle_{i}", 0.009, 0.092, (x, 0.095, -0.036), rubber)

    hinge_origin = (0.0, -0.078, 0.035)
    clamp.visual(
        Cylinder(radius=0.011, length=0.520),
        origin=Origin(xyz=hinge_origin, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="main_hinge_pin",
    )
    stile = model.part("stile_frame")

    rail_x = 0.165
    tube_r = 0.016
    rail_start_y = 0.035
    rail_start_z = -0.045
    frame_dy = 0.22
    frame_dz = -1.05
    rail_dy = frame_dy - rail_start_y
    rail_dz = frame_dz - rail_start_z
    frame_len = math.hypot(rail_dy, rail_dz)
    frame_roll = math.atan2(-rail_dy, rail_dz)
    stile.visual(
        Cylinder(radius=0.023, length=0.405),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="top_hinge_tube",
    )
    for i, x in enumerate((-rail_x, rail_x)):
        stile.visual(
            Cylinder(radius=tube_r, length=frame_len),
            origin=Origin(
                xyz=(x, (rail_start_y + frame_dy) / 2.0, (rail_start_z + frame_dz) / 2.0),
                rpy=(frame_roll, 0.0, 0.0),
            ),
            material=aluminum,
            name=f"side_stile_{i}",
        )
        stile.visual(
            Box((0.040, 0.025, 0.040)),
            origin=Origin(xyz=(x, 0.030, -0.025)),
            material=aluminum,
            name=f"upper_weld_socket_{i}",
        )
    upper_y = rail_start_y + rail_dy * 0.32
    upper_z = rail_start_z + rail_dz * 0.32
    cyl_x(stile, "upper_step_tube", 0.015, 0.455, (0.0, upper_y, upper_z), aluminum)
    stile.visual(
        Box((0.350, 0.080, 0.012)),
        origin=Origin(xyz=(0.0, upper_y + 0.020, upper_z - 0.006)),
        material=tread_white,
        name="upper_tread_pad",
    )
    cyl_x(stile, "bottom_cross_tube", 0.017, 0.455, (0.0, frame_dy, frame_dz), aluminum)

    rung_specs = (
        ("middle_rung", 0.61),
        ("lower_rung", 0.84),
    )
    stile.visual(
        Cylinder(radius=0.012, length=0.390),
        origin=Origin(
            xyz=(0.0, rail_start_y + rail_dy * 0.61, rail_start_z + rail_dz * 0.61),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=stainless,
        name="middle_rung_mount_pin",
    )
    stile.visual(
        Cylinder(radius=0.012, length=0.390),
        origin=Origin(
            xyz=(0.0, rail_start_y + rail_dy * 0.84, rail_start_z + rail_dz * 0.84),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=stainless,
        name="lower_rung_mount_pin",
    )
    for rung_name, station in rung_specs:
        mount_y = rail_start_y + rail_dy * station
        mount_z = rail_start_z + rail_dz * station
        for i, x in enumerate((-rail_x, rail_x)):
            stile.visual(
                Box((0.042, 0.044, 0.036)),
                origin=Origin(xyz=(x, mount_y, mount_z)),
                material=stainless,
                name=f"{rung_name}_hinge_boss_{i}",
            )

    model.articulation(
        "clamp_to_stile",
        ArticulationType.REVOLUTE,
        parent=clamp,
        child=stile,
        origin=Origin(xyz=hinge_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=1.45),
    )

    def add_folding_rung(name: str, station: float) -> None:
        rung = model.part(name)
        rung.visual(
            Cylinder(radius=0.014, length=0.276),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name="hinge_sleeve",
        )
        rung.visual(
            Box((0.270, 0.120, 0.016)),
            origin=Origin(xyz=(0.0, 0.060, -0.022)),
            material=aluminum,
            name="step_tread",
        )
        for i, y in enumerate((0.032, 0.060, 0.088)):
            rung.visual(
                Box((0.230, 0.010, 0.006)),
                origin=Origin(xyz=(0.0, y, -0.012)),
                material=rubber,
                name=f"grip_strip_{i}",
            )

        mount_y = rail_start_y + rail_dy * station
        mount_z = rail_start_z + rail_dz * station
        model.articulation(
            f"stile_to_{name}",
            ArticulationType.REVOLUTE,
            parent=stile,
            child=rung,
            origin=Origin(xyz=(0.0, mount_y, mount_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.70),
        )

    add_folding_rung("middle_rung", 0.61)
    add_folding_rung("lower_rung", 0.84)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clamp = object_model.get_part("clamp_bracket")
    stile = object_model.get_part("stile_frame")
    middle = object_model.get_part("middle_rung")
    lower = object_model.get_part("lower_rung")
    main_hinge = object_model.get_articulation("clamp_to_stile")
    middle_hinge = object_model.get_articulation("stile_to_middle_rung")
    lower_hinge = object_model.get_articulation("stile_to_lower_rung")

    ctx.check(
        "boarding ladder has three revolute hinges",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (main_hinge, middle_hinge, lower_hinge)
        ),
        details="The clamp/stile and both lower rungs must be revolute.",
    )
    ctx.allow_overlap(
        clamp,
        stile,
        elem_a="main_hinge_pin",
        elem_b="top_hinge_tube",
        reason="The clamp bracket's hinge pin intentionally passes through the stile hinge tube.",
    )
    ctx.expect_overlap(
        clamp,
        stile,
        axes="x",
        elem_a="main_hinge_pin",
        elem_b="top_hinge_tube",
        min_overlap=0.35,
        name="main hinge pin passes through stile hinge tube",
    )
    ctx.expect_within(
        clamp,
        stile,
        axes="yz",
        inner_elem="main_hinge_pin",
        outer_elem="top_hinge_tube",
        margin=0.002,
        name="main hinge pin is centered in the stile tube",
    )
    ctx.expect_gap(
        stile,
        clamp,
        axis="x",
        positive_elem="top_hinge_tube",
        negative_elem="hinge_cheek_0",
        min_gap=0.002,
        max_gap=0.012,
        name="main hinge tube clears lower clamp cheek",
    )
    ctx.expect_gap(
        clamp,
        stile,
        axis="x",
        positive_elem="hinge_cheek_1",
        negative_elem="top_hinge_tube",
        min_gap=0.002,
        max_gap=0.012,
        name="main hinge tube clears upper clamp cheek",
    )
    ctx.expect_overlap(
        stile,
        clamp,
        axes="yz",
        elem_a="top_hinge_tube",
        elem_b="hinge_cheek_0",
        min_overlap=0.020,
        name="main hinge tube aligns with clamp cheek bores",
    )
    ctx.allow_overlap(
        middle,
        stile,
        elem_a="hinge_sleeve",
        elem_b="middle_rung_mount_pin",
        reason="The middle folding rung sleeve is intentionally modeled around its hinge pin.",
    )
    ctx.expect_overlap(
        middle,
        stile,
        axes="x",
        elem_a="hinge_sleeve",
        elem_b="middle_rung_mount_pin",
        min_overlap=0.20,
        name="middle rung sleeve spans between its stile mounts",
    )
    ctx.expect_within(
        stile,
        middle,
        axes="yz",
        inner_elem="middle_rung_mount_pin",
        outer_elem="hinge_sleeve",
        margin=0.0025,
        name="middle rung hinge pin is centered inside the sleeve",
    )
    ctx.allow_overlap(
        lower,
        stile,
        elem_a="hinge_sleeve",
        elem_b="lower_rung_mount_pin",
        reason="The lower folding rung sleeve is intentionally modeled around its hinge pin.",
    )
    ctx.expect_overlap(
        lower,
        stile,
        axes="x",
        elem_a="hinge_sleeve",
        elem_b="lower_rung_mount_pin",
        min_overlap=0.20,
        name="lower rung sleeve spans between its stile mounts",
    )
    ctx.expect_within(
        stile,
        lower,
        axes="yz",
        inner_elem="lower_rung_mount_pin",
        outer_elem="hinge_sleeve",
        margin=0.0025,
        name="lower rung hinge pin is centered inside the sleeve",
    )

    rest_bottom = ctx.part_element_world_aabb(stile, elem="bottom_cross_tube")
    with ctx.pose({main_hinge: 1.20}):
        raised_bottom = ctx.part_element_world_aabb(stile, elem="bottom_cross_tube")
    ctx.check(
        "stile frame folds upward at clamp",
        rest_bottom is not None
        and raised_bottom is not None
        and raised_bottom[0][2] > rest_bottom[0][2] + 0.55,
        details=f"rest={rest_bottom}, raised={raised_bottom}",
    )

    for rung, hinge, label in ((middle, middle_hinge, "middle"), (lower, lower_hinge, "lower")):
        rest_tread = ctx.part_element_world_aabb(rung, elem="step_tread")
        with ctx.pose({hinge: 1.25}):
            folded_tread = ctx.part_element_world_aabb(rung, elem="step_tread")
        ctx.check(
            f"{label} rung tread folds upward",
            rest_tread is not None
            and folded_tread is not None
            and folded_tread[1][2] > rest_tread[1][2] + 0.08,
            details=f"rest={rest_tread}, folded={folded_tread}",
        )

    return ctx.report()


object_model = build_object_model()
