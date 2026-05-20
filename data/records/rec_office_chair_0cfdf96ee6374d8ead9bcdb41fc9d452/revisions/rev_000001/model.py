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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_mesh_office_chair")

    black = model.material("satin_black", color=(0.02, 0.02, 0.018, 1.0))
    mesh_mat = model.material("black_mesh", color=(0.01, 0.012, 0.012, 0.82))
    cushion = model.material("charcoal_fabric", color=(0.075, 0.078, 0.082, 1.0))
    metal = model.material("brushed_metal", color=(0.62, 0.62, 0.58, 1.0))
    rubber = model.material("matte_rubber", color=(0.008, 0.008, 0.008, 1.0))

    base = model.part("star_base")
    base.visual(
        Cylinder(radius=0.075, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=black,
        name="center_hub",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material=metal,
        name="lower_column",
    )

    caster_radius = 0.47
    for i in range(5):
        angle = i * 2.0 * math.pi / 5.0 + math.pi / 2.0
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mid = caster_radius * 0.48
        base.visual(
            Box((0.46, 0.070, 0.045)),
            origin=Origin(
                xyz=(leg_mid * c, leg_mid * s, 0.150),
                rpy=(0.0, 0.0, angle),
            ),
            material=black,
            name=f"spoke_{i}",
        )
        base.visual(
            Box((0.105, 0.090, 0.040)),
            origin=Origin(
                xyz=(caster_radius * c, caster_radius * s, 0.150),
                rpy=(0.0, 0.0, angle),
            ),
            material=black,
            name=f"caster_socket_{i}",
        )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.042, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=metal,
        name="upper_column",
    )
    seat.visual(
        Box((0.260, 0.260, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=black,
        name="seat_mechanism",
    )
    seat.visual(
        Box((0.520, 0.520, 0.075)),
        origin=Origin(xyz=(0.020, 0.0, 0.055)),
        material=cushion,
        name="seat_pan",
    )
    seat_mesh = SlotPatternPanelGeometry(
        (0.440, 0.440),
        0.007,
        slot_size=(0.040, 0.008),
        pitch=(0.055, 0.030),
        frame=0.020,
        corner_radius=0.018,
        slot_angle_deg=0.0,
        stagger=True,
    )
    seat.visual(
        mesh_from_geometry(seat_mesh, "seat_mesh"),
        origin=Origin(xyz=(0.045, 0.0, 0.096)),
        material=mesh_mat,
        name="seat_mesh",
    )
    seat.visual(
        Box((0.070, 0.080, 0.170)),
        origin=Origin(xyz=(-0.180, 0.300, 0.110)),
        material=black,
        name="left_arm_bracket",
    )
    seat.visual(
        Box((0.070, 0.080, 0.170)),
        origin=Origin(xyz=(-0.180, -0.300, 0.110)),
        material=black,
        name="right_arm_bracket",
    )
    seat.visual(
        Box((0.060, 0.050, 0.095)),
        origin=Origin(xyz=(-0.255, 0.278, 0.104)),
        material=black,
        name="left_back_bracket",
    )
    seat.visual(
        Box((0.060, 0.050, 0.095)),
        origin=Origin(xyz=(-0.255, -0.278, 0.104)),
        material=black,
        name="right_back_bracket",
    )

    model.articulation(
        "base_to_seat",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.5),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.018, length=0.506),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hinge_barrel",
    )
    backrest.visual(
        Box((0.082, 0.460, 0.056)),
        origin=Origin(xyz=(-0.028, 0.0, 0.035)),
        material=black,
        name="hinge_block",
    )
    backrest.visual(
        Box((0.050, 0.540, 0.045)),
        origin=Origin(xyz=(-0.064, 0.0, 0.080)),
        material=black,
        name="lower_rail",
    )
    backrest.visual(
        Box((0.046, 0.540, 0.034)),
        origin=Origin(xyz=(-0.064, 0.0, 0.585)),
        material=black,
        name="top_rail",
    )
    for side, y in (("left", 0.240), ("right", -0.240)):
        backrest.visual(
            Box((0.040, 0.040, 0.500)),
            origin=Origin(xyz=(-0.064, y, 0.335)),
            material=black,
            name=f"{side}_side_rail",
        )
    backrest.visual(
        Box((0.036, 0.028, 0.500)),
        origin=Origin(xyz=(-0.064, 0.0, 0.335)),
        material=black,
        name="center_spine",
    )
    back_mesh = SlotPatternPanelGeometry(
        (0.460, 0.198),
        0.006,
        slot_size=(0.036, 0.006),
        pitch=(0.052, 0.022),
        frame=0.014,
        corner_radius=0.020,
        slot_angle_deg=18.0,
        stagger=True,
    )
    backrest.visual(
        mesh_from_geometry(back_mesh, "left_back_mesh"),
        origin=Origin(xyz=(-0.058, 0.126, 0.340), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=mesh_mat,
        name="left_mesh",
    )
    backrest.visual(
        mesh_from_geometry(back_mesh, "right_back_mesh"),
        origin=Origin(xyz=(-0.058, -0.126, 0.340), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=mesh_mat,
        name="right_mesh",
    )

    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(-0.275, 0.0, 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.0, lower=0.0, upper=0.48),
    )

    for side, y in (("left", 0.380), ("right", -0.380)):
        arm = model.part(f"{side}_armrest")
        arm.visual(
            Cylinder(radius=0.022, length=0.080),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name="hinge_barrel",
        )
        arm.visual(
            Box((0.120, 0.060, 0.050)),
            origin=Origin(xyz=(0.060, 0.0, 0.020)),
            material=black,
            name="rear_knuckle",
        )
        arm.visual(
            Box((0.390, 0.075, 0.045)),
            origin=Origin(xyz=(0.245, 0.0, 0.040)),
            material=black,
            name="arm_pad",
        )
        arm.visual(
            Box((0.035, 0.050, 0.105)),
            origin=Origin(xyz=(0.090, 0.0, -0.028)),
            material=black,
            name="drop_link",
        )
        model.articulation(
            f"seat_to_{side}_armrest",
            ArticulationType.REVOLUTE,
            parent=seat,
            child=arm,
            origin=Origin(xyz=(-0.180, y, 0.215)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.65),
        )

    tire = TireGeometry(
        0.045,
        0.034,
        inner_radius=0.027,
        tread=TireTread(style="ribbed", depth=0.0025, count=18, land_ratio=0.62),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.003, radius=0.002),
    )
    tire_mesh = mesh_from_geometry(tire, "caster_tire")

    for i in range(5):
        angle = i * 2.0 * math.pi / 5.0 + math.pi / 2.0
        c = math.cos(angle)
        s = math.sin(angle)
        fork = model.part(f"caster_fork_{i}")
        fork.visual(
            Cylinder(radius=0.014, length=0.055),
            origin=Origin(xyz=(0.0, 0.0, -0.0275)),
            material=metal,
            name="swivel_stem",
        )
        fork.visual(
            Box((0.070, 0.046, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, -0.055)),
            material=black,
            name="fork_crown",
        )
        fork.visual(
            Box((0.006, 0.046, 0.088)),
            origin=Origin(xyz=(0.020, 0.0, -0.101)),
            material=black,
            name="fork_cheek_0",
        )
        fork.visual(
            Box((0.006, 0.046, 0.088)),
            origin=Origin(xyz=(-0.020, 0.0, -0.101)),
            material=black,
            name="fork_cheek_1",
        )
        model.articulation(
            f"base_to_caster_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=fork,
            origin=Origin(xyz=(caster_radius * c, caster_radius * s, 0.155)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=8.0),
        )

        wheel_part = model.part(f"caster_wheel_{i}")
        wheel_part.visual(tire_mesh, material=rubber, name="tire")
        wheel_part.visual(
            Cylinder(radius=0.028, length=0.032),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name="rim",
        )
        model.articulation(
            f"caster_{i}_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel_part,
            origin=Origin(xyz=(0.0, 0.0, -0.110)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    left_armrest = object_model.get_part("left_armrest")
    right_armrest = object_model.get_part("right_armrest")
    swivel = object_model.get_articulation("base_to_seat")
    back_hinge = object_model.get_articulation("seat_to_backrest")

    ctx.check(
        "chair has continuous seat swivel",
        swivel.articulation_type == ArticulationType.CONTINUOUS and swivel.axis == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )
    ctx.check(
        "split backrest reclines on transverse hinge",
        back_hinge.articulation_type == ArticulationType.REVOLUTE and back_hinge.axis == (0.0, -1.0, 0.0),
        details=f"type={back_hinge.articulation_type}, axis={back_hinge.axis}",
    )

    ctx.expect_contact(
        "seat",
        "star_base",
        elem_a="upper_column",
        elem_b="lower_column",
        contact_tol=0.003,
        name="swivel column halves meet",
    )

    for i in range(5):
        fork_name = f"caster_fork_{i}"
        wheel_name = f"caster_wheel_{i}"
        socket_name = f"caster_socket_{i}"
        swivel_joint = object_model.get_articulation(f"base_to_caster_{i}")
        spin_joint = object_model.get_articulation(f"caster_{i}_to_wheel")
        ctx.check(
            f"caster {i} swivels vertically",
            swivel_joint.articulation_type == ArticulationType.CONTINUOUS
            and swivel_joint.axis == (0.0, 0.0, 1.0),
            details=f"type={swivel_joint.articulation_type}, axis={swivel_joint.axis}",
        )
        ctx.check(
            f"caster {i} wheel spins on axle",
            spin_joint.articulation_type == ArticulationType.CONTINUOUS
            and spin_joint.axis == (1.0, 0.0, 0.0),
            details=f"type={spin_joint.articulation_type}, axis={spin_joint.axis}",
        )
        ctx.allow_overlap(
            "star_base",
            fork_name,
            elem_a=socket_name,
            elem_b="swivel_stem",
            reason="Each caster stem is intentionally seated inside its socket as the swivel bearing.",
        )
        ctx.expect_within(
            fork_name,
            "star_base",
            axes="xy",
            inner_elem="swivel_stem",
            outer_elem=socket_name,
            margin=0.003,
            name=f"caster {i} stem centered in socket",
        )
        ctx.expect_overlap(
            fork_name,
            "star_base",
            axes="z",
            elem_a="swivel_stem",
            elem_b=socket_name,
            min_overlap=0.020,
            name=f"caster {i} stem retained in socket",
        )
        ctx.expect_contact(
            wheel_name,
            fork_name,
            elem_a="tire",
            elem_b="fork_cheek_0",
            contact_tol=0.004,
            name=f"caster {i} wheel captured by fork",
        )

    closed_back = ctx.part_element_world_aabb(backrest, elem="top_rail")
    with ctx.pose({back_hinge: 0.42}):
        reclined_back = ctx.part_element_world_aabb(backrest, elem="top_rail")
    ctx.check(
        "backrest reclines rearward",
        closed_back is not None
        and reclined_back is not None
        and reclined_back[1][0] < closed_back[1][0] - 0.10,
        details=f"closed={closed_back}, reclined={reclined_back}",
    )

    for side, armrest in (("left", left_armrest), ("right", right_armrest)):
        joint = object_model.get_articulation(f"seat_to_{side}_armrest")
        closed = ctx.part_element_world_aabb(armrest, elem="arm_pad")
        with ctx.pose({joint: 1.45}):
            raised = ctx.part_element_world_aabb(armrest, elem="arm_pad")
        ctx.check(
            f"{side} armrest flips upward",
            joint.articulation_type == ArticulationType.REVOLUTE
            and closed is not None
            and raised is not None
            and raised[1][2] > closed[1][2] + 0.20,
            details=f"type={joint.articulation_type}, closed={closed}, raised={raised}",
        )

    return ctx.report()


object_model = build_object_model()
