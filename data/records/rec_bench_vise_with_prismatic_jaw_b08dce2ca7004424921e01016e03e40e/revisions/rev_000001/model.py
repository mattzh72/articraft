from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _dovetail_prism_x(
    *,
    length: float,
    center_y: float,
    z0: float,
    bottom_width: float,
    top_width: float,
    height: float,
) -> MeshGeometry:
    """Trapezoidal dovetail rail extruded along local X."""
    profile = [
        (center_y - bottom_width / 2.0, z0),
        (center_y + bottom_width / 2.0, z0),
        (center_y + top_width / 2.0, z0 + height),
        (center_y - top_width / 2.0, z0 + height),
    ]
    geom = MeshGeometry()
    xs = (-length / 2.0, length / 2.0)
    for x in xs:
        for y, z in profile:
            geom.add_vertex(x, y, z)

    # End caps.
    geom.add_face(0, 1, 2)
    geom.add_face(0, 2, 3)
    geom.add_face(4, 6, 5)
    geom.add_face(4, 7, 6)

    # Side faces.
    for i in range(4):
        j = (i + 1) % 4
        a, b = i, j
        c, d = 4 + j, 4 + i
        geom.add_face(a, b, c)
        geom.add_face(a, c, d)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compound_angle_vise")

    model.material("cast_blue", rgba=(0.05, 0.18, 0.32, 1.0))
    model.material("dark_cast", rgba=(0.035, 0.04, 0.045, 1.0))
    model.material("machined_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    model.material("hardened_jaw", rgba=(0.80, 0.78, 0.70, 1.0))
    model.material("blackened_steel", rgba=(0.015, 0.014, 0.012, 1.0))

    base = model.part("base_plate")
    base.visual(
        Box((0.56, 0.32, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material="dark_cast",
        name="flat_base_plate",
    )
    for x in (-0.19, 0.19):
        for y in (-0.115, 0.115):
            base.visual(
                Box((0.105, 0.020, 0.002)),
                origin=Origin(xyz=(x, y, 0.033)),
                material="blackened_steel",
                name=f"mount_slot_{x:+.2f}_{y:+.2f}",
            )
    base.visual(
        Cylinder(radius=0.030, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0335)),
        material="machined_steel",
        name="swivel_pivot_washer",
    )

    swivel = model.part("swivel_stage")
    swivel.visual(
        Cylinder(radius=0.170, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material="cast_blue",
        name="swivel_disk",
    )
    swivel.visual(
        mesh_from_geometry(TorusGeometry(0.158, 0.0035, radial_segments=20, tubular_segments=64), "swivel_degree_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material="machined_steel",
        name="degree_ring",
    )
    swivel.visual(
        Cylinder(radius=0.055, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material="cast_blue",
        name="central_boss",
    )
    swivel.visual(
        Box((0.190, 0.205, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
        material="cast_blue",
        name="tilt_yoke_foot",
    )
    for idx, y in enumerate((-0.125, 0.125)):
        swivel.visual(
            Box((0.090, 0.035, 0.120)),
            origin=Origin(xyz=(0.0, y, 0.095)),
            material="cast_blue",
            name=f"yoke_cheek_{idx}",
        )

    tilt = model.part("tilt_stage")
    tilt.visual(
        Cylinder(radius=0.017, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="trunnion_pin",
    )
    tilt.visual(
        Box((0.460, 0.180, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material="cast_blue",
        name="tilting_bed",
    )
    for idx, y in enumerate((-0.075, 0.075)):
        tilt.visual(
            Box((0.080, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.015)),
            material="cast_blue",
            name=f"trunnion_lug_{idx}",
        )
    for idx, y in enumerate((-0.052, 0.052)):
        rail = _dovetail_prism_x(
            length=0.420,
            center_y=y,
            z0=0.063,
            bottom_width=0.026,
            top_width=0.045,
            height=0.018,
        )
        tilt.visual(
            mesh_from_geometry(rail, f"dovetail_way_{idx}"),
            origin=Origin(xyz=(-0.005, 0.0, 0.0)),
            material="machined_steel",
            name=f"dovetail_way_{idx}",
        )

    # The fixed jaw rises on two side ribs, leaving a central screw channel.
    for idx, y in enumerate((-0.060, 0.060)):
        tilt.visual(
            Box((0.075, 0.040, 0.042)),
            origin=Origin(xyz=(0.145, y, 0.083)),
            material="cast_blue",
            name=f"fixed_jaw_rib_{idx}",
        )
    tilt.visual(
        Box((0.080, 0.180, 0.095)),
        origin=Origin(xyz=(0.145, 0.0, 0.1495)),
        material="cast_blue",
        name="fixed_jaw_block",
    )
    tilt.visual(
        Box((0.009, 0.155, 0.070)),
        origin=Origin(xyz=(0.1005, 0.0, 0.148)),
        material="hardened_jaw",
        name="fixed_jaw_plate",
    )
    for idx, z in enumerate((0.122, 0.135, 0.148, 0.161, 0.174)):
        tilt.visual(
            Box((0.0015, 0.142, 0.003)),
            origin=Origin(xyz=(0.0953, 0.0, z)),
            material="blackened_steel",
            name=f"fixed_jaw_serration_{idx}",
        )

    tilt.visual(
        Box((0.035, 0.050, 0.005)),
        origin=Origin(xyz=(-0.205, 0.0, 0.0675)),
        material="cast_blue",
        name="screw_bearing_base",
    )
    tilt.visual(
        mesh_from_geometry(TorusGeometry(0.012, 0.003, radial_segments=16, tubular_segments=32), "front_screw_bearing"),
        origin=Origin(xyz=(-0.205, 0.0, 0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="machined_steel",
        name="front_screw_bearing",
    )

    moving = model.part("moving_jaw")
    for idx, y in enumerate((-0.052, 0.052)):
        moving.visual(
            Box((0.120, 0.035, 0.018)),
            origin=Origin(xyz=(-0.040, y, 0.090)),
            material="cast_blue",
            name=f"slide_shoe_{idx}",
        )
    moving.visual(
        Box((0.150, 0.130, 0.006)),
        origin=Origin(xyz=(-0.040, 0.0, 0.100)),
        material="cast_blue",
        name="jaw_saddle",
    )
    moving.visual(
        Box((0.075, 0.180, 0.095)),
        origin=Origin(xyz=(-0.035, 0.0, 0.1485)),
        material="cast_blue",
        name="moving_jaw_block",
    )
    moving.visual(
        Box((0.009, 0.155, 0.070)),
        origin=Origin(xyz=(0.0070, 0.0, 0.148)),
        material="hardened_jaw",
        name="moving_jaw_plate",
    )
    for idx, z in enumerate((0.122, 0.135, 0.148, 0.161, 0.174)):
        moving.visual(
            Box((0.0015, 0.142, 0.003)),
            origin=Origin(xyz=(0.0122, 0.0, z)),
            material="blackened_steel",
            name=f"moving_jaw_serration_{idx}",
        )

    leadscrew = model.part("leadscrew")
    leadscrew.visual(
        Cylinder(radius=0.0065, length=0.390),
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="machined_steel",
        name="screw_shaft",
    )
    leadscrew.visual(
        Cylinder(radius=0.0095, length=0.012),
        origin=Origin(xyz=(-0.205, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="machined_steel",
        name="bearing_journal",
    )
    for idx, x in enumerate((-0.165, -0.140, -0.115, -0.090, -0.065, -0.040, -0.015, 0.010, 0.035, 0.060, 0.085, 0.110)):
        leadscrew.visual(
            Cylinder(radius=0.0082, length=0.003),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="blackened_steel",
            name=f"thread_crest_{idx}",
        )
    leadscrew.visual(
        Cylinder(radius=0.015, length=0.024),
        origin=Origin(xyz=(-0.265, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="machined_steel",
        name="handwheel_hub",
    )
    leadscrew.visual(
        Cylinder(radius=0.006, length=0.070),
        origin=Origin(xyz=(-0.270, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="machined_steel",
        name="handle_stem",
    )
    leadscrew.visual(
        mesh_from_geometry(TorusGeometry(0.034, 0.004, radial_segments=16, tubular_segments=56), "handwheel_rim"),
        origin=Origin(xyz=(-0.265, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="blackened_steel",
        name="handwheel_rim",
    )
    leadscrew.visual(
        Cylinder(radius=0.0028, length=0.072),
        origin=Origin(xyz=(-0.265, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="handwheel_spoke_y",
    )
    leadscrew.visual(
        Cylinder(radius=0.0028, length=0.072),
        origin=Origin(xyz=(-0.265, 0.0, 0.0)),
        material="machined_steel",
        name="handwheel_spoke_z",
    )
    leadscrew.visual(
        Cylinder(radius=0.004, length=0.110),
        origin=Origin(xyz=(-0.286, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="tee_handle_bar",
    )
    for idx, y in enumerate((-0.055, 0.055)):
        leadscrew.visual(
            Sphere(radius=0.008),
            origin=Origin(xyz=(-0.286, y, 0.0)),
            material="blackened_steel",
            name=f"tee_handle_knob_{idx}",
        )

    model.articulation(
        "base_to_swivel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "swivel_to_tilt",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=tilt,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=100.0, velocity=0.8, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "tilt_to_moving_jaw",
        ArticulationType.PRISMATIC,
        parent=tilt,
        child=moving,
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.08, lower=0.0, upper=0.120),
    )
    model.articulation(
        "tilt_to_leadscrew",
        ArticulationType.CONTINUOUS,
        parent=tilt,
        child=leadscrew,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_plate")
    swivel = object_model.get_part("swivel_stage")
    tilt = object_model.get_part("tilt_stage")
    moving = object_model.get_part("moving_jaw")
    leadscrew = object_model.get_part("leadscrew")

    swivel_joint = object_model.get_articulation("base_to_swivel")
    tilt_joint = object_model.get_articulation("swivel_to_tilt")
    jaw_slide = object_model.get_articulation("tilt_to_moving_jaw")
    screw_turn = object_model.get_articulation("tilt_to_leadscrew")

    # The tilt trunnion is intentionally captured inside the solid cheek
    # proxies; the visible overlap is local to the hinge-bearing interface.
    for idx in (0, 1):
        ctx.allow_overlap(
            swivel,
            tilt,
            elem_a=f"yoke_cheek_{idx}",
            elem_b="trunnion_pin",
            reason="The trunnion pin is intentionally seated through the yoke cheek bearing proxy.",
        )
        ctx.expect_overlap(
            swivel,
            tilt,
            axes="yz",
            elem_a=f"yoke_cheek_{idx}",
            elem_b="trunnion_pin",
            min_overlap=0.020,
            name=f"trunnion pin is captured by cheek {idx}",
        )

    ctx.allow_overlap(
        tilt,
        leadscrew,
        elem_a="front_screw_bearing",
        elem_b="bearing_journal",
        reason="The leadscrew journal is intentionally captured in the front bearing proxy with a slight seated interference.",
    )
    ctx.expect_overlap(
        tilt,
        leadscrew,
        axes="yz",
        elem_a="front_screw_bearing",
        elem_b="bearing_journal",
        min_overlap=0.010,
        name="leadscrew journal is captured inside the front bearing",
    )

    ctx.expect_gap(
        swivel,
        base,
        axis="z",
        positive_elem="swivel_disk",
        negative_elem="flat_base_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel disk rests on the flat base plate",
    )
    ctx.expect_overlap(
        swivel,
        base,
        axes="xy",
        elem_a="swivel_disk",
        elem_b="flat_base_plate",
        min_overlap=0.30,
        name="swivel stage footprint is supported by the base plate",
    )

    for idx in (0, 1):
        ctx.expect_gap(
            moving,
            tilt,
            axis="z",
            positive_elem=f"slide_shoe_{idx}",
            negative_elem=f"dovetail_way_{idx}",
            max_gap=0.004,
            max_penetration=0.0,
            name=f"slide shoe {idx} rides just above its dovetail way",
        )
        ctx.expect_overlap(
            moving,
            tilt,
            axes="xy",
            elem_a=f"slide_shoe_{idx}",
            elem_b=f"dovetail_way_{idx}",
            min_overlap=0.030,
            name=f"slide shoe {idx} is engaged on the dovetail way",
        )

    ctx.expect_gap(
        tilt,
        moving,
        axis="x",
        positive_elem="fixed_jaw_plate",
        negative_elem="moving_jaw_plate",
        min_gap=0.035,
        max_gap=0.055,
        name="jaw plates start with a realistic clamping gap",
    )

    rest_jaw_pos = ctx.part_world_position(moving)
    with ctx.pose({jaw_slide: 0.120}):
        extended_jaw_pos = ctx.part_world_position(moving)
        ctx.expect_overlap(
            moving,
            tilt,
            axes="x",
            elem_a="slide_shoe_0",
            elem_b="dovetail_way_0",
            min_overlap=0.080,
            name="front jaw remains retained on the left dovetail at full travel",
        )
        ctx.expect_overlap(
            moving,
            tilt,
            axes="x",
            elem_a="slide_shoe_1",
            elem_b="dovetail_way_1",
            min_overlap=0.080,
            name="front jaw remains retained on the right dovetail at full travel",
        )
    ctx.check(
        "moving jaw opens forward on its prismatic slide",
        rest_jaw_pos is not None
        and extended_jaw_pos is not None
        and extended_jaw_pos[0] < rest_jaw_pos[0] - 0.10,
        details=f"rest={rest_jaw_pos}, extended={extended_jaw_pos}",
    )

    rest_handle_aabb = ctx.part_element_world_aabb(leadscrew, elem="tee_handle_bar")
    with ctx.pose({tilt_joint: 0.50}):
        tilted_handle_aabb = ctx.part_element_world_aabb(leadscrew, elem="tee_handle_bar")
    rest_handle_z = None if rest_handle_aabb is None else (rest_handle_aabb[0][2] + rest_handle_aabb[1][2]) / 2.0
    tilted_handle_z = None if tilted_handle_aabb is None else (tilted_handle_aabb[0][2] + tilted_handle_aabb[1][2]) / 2.0
    ctx.check(
        "positive tilt raises the front handwheel end of the vise",
        rest_handle_z is not None and tilted_handle_z is not None and tilted_handle_z > rest_handle_z + 0.08,
        details=f"rest_z={rest_handle_z}, tilted_z={tilted_handle_z}",
    )

    rest_moving_pos = ctx.part_world_position(moving)
    with ctx.pose({swivel_joint: math.pi / 2.0}):
        swiveled_moving_pos = ctx.part_world_position(moving)
    ctx.check(
        "swivel base rotates the jaw assembly around the vertical axis",
        rest_moving_pos is not None
        and swiveled_moving_pos is not None
        and abs(swiveled_moving_pos[1] - rest_moving_pos[1]) > 0.035,
        details=f"rest={rest_moving_pos}, swiveled={swiveled_moving_pos}",
    )

    rest_bar_aabb = ctx.part_element_world_aabb(leadscrew, elem="tee_handle_bar")
    with ctx.pose({screw_turn: math.pi / 2.0}):
        turned_bar_aabb = ctx.part_element_world_aabb(leadscrew, elem="tee_handle_bar")
    rest_bar_z_span = 0.0 if rest_bar_aabb is None else rest_bar_aabb[1][2] - rest_bar_aabb[0][2]
    turned_bar_z_span = 0.0 if turned_bar_aabb is None else turned_bar_aabb[1][2] - turned_bar_aabb[0][2]
    ctx.check(
        "leadscrew handle rotates about the screw axis",
        turned_bar_z_span > rest_bar_z_span + 0.05,
        details=f"rest_z_span={rest_bar_z_span}, turned_z_span={turned_bar_z_span}",
    )

    return ctx.report()


object_model = build_object_model()
