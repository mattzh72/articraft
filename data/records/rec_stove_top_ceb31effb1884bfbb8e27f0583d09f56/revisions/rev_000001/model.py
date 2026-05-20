from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: Material | str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: Material | str,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_burner(
    part,
    *,
    x: float,
    y: float,
    radius: float,
    name: str,
    steel,
    cap,
    grate,
) -> None:
    """Stacked burner pieces that visibly touch the shallow hob plate."""
    plate_top = 0.889
    ring_h = 0.006
    cap_h = 0.012
    _add_cylinder(
        part,
        radius=radius,
        length=ring_h,
        xyz=(x, y, plate_top + ring_h / 2.0),
        material=steel,
        name=f"{name}_burner_ring",
    )
    _add_cylinder(
        part,
        radius=radius * 0.58,
        length=cap_h,
        xyz=(x, y, plate_top + ring_h + cap_h / 2.0),
        material=cap,
        name=f"{name}_cap",
    )
    bar_z = plate_top + ring_h + cap_h - 0.001
    _add_box(
        part,
        (radius * 2.75, 0.018, 0.010),
        (x, y, bar_z),
        grate,
        f"{name}_grate_x",
    )
    _add_box(
        part,
        (0.018, radius * 2.75, 0.010),
        (x, y, bar_z),
        grate,
        f"{name}_grate_y",
    )
    foot_radius = 0.006
    foot_h = 0.032
    for index, (fx, fy) in enumerate(
        (
            (x + radius * 1.18, y),
            (x - radius * 1.18, y),
            (x, y + radius * 1.18),
            (x, y - radius * 1.18),
        )
    ):
        _add_cylinder(
            part,
            radius=foot_radius,
            length=foot_h,
            xyz=(fx, fy, plate_top + foot_h / 2.0),
            material=grate,
            name=f"{name}_grate_foot_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_burner_gas_hob_cabinet")

    cabinet_paint = model.material("warm_white_lacquer", rgba=(0.88, 0.84, 0.76, 1.0))
    countertop = model.material("pale_countertop", rgba=(0.82, 0.80, 0.74, 1.0))
    shadow = model.material("shadow_gap", rgba=(0.035, 0.033, 0.030, 1.0))
    glass_black = model.material("black_glass", rgba=(0.010, 0.012, 0.014, 1.0))
    enamel = model.material("matte_enamel", rgba=(0.02, 0.02, 0.02, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.035, 0.035, 0.035, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.66, 0.62, 1.0))
    knob_material = model.material("brushed_knob_dark", rgba=(0.13, 0.13, 0.13, 1.0))

    cabinet = model.part("cabinet")

    cabinet_w = 1.20
    cabinet_d = 0.62
    cabinet_h = 0.82
    side_t = 0.030
    front_y = -cabinet_d / 2.0

    # Clean two-door cabinet bay and countertop.
    _add_box(cabinet, (side_t, cabinet_d, cabinet_h), (-cabinet_w / 2.0, 0.0, cabinet_h / 2.0), cabinet_paint, "side_panel_0")
    _add_box(cabinet, (side_t, cabinet_d, cabinet_h), (cabinet_w / 2.0, 0.0, cabinet_h / 2.0), cabinet_paint, "side_panel_1")
    _add_box(cabinet, (cabinet_w, 0.026, cabinet_h), (0.0, cabinet_d / 2.0 - 0.013, cabinet_h / 2.0), cabinet_paint, "back_panel")
    _add_box(cabinet, (cabinet_w, cabinet_d, 0.045), (0.0, 0.0, 0.0225), cabinet_paint, "bottom_deck")
    _add_box(cabinet, (cabinet_w, cabinet_d, 0.045), (0.0, 0.0, cabinet_h - 0.0225), cabinet_paint, "top_deck")
    cabinet.visual(
        Box((cabinet_w + 0.02, 0.035, 0.135)),
        origin=Origin(xyz=(0.0, front_y - 0.0175, 0.742)),
        material=cabinet_paint,
        name="control_fascia",
    )
    _add_box(cabinet, (0.045, 0.030, 0.650), (0.0, front_y - 0.020, 0.365), shadow, "center_reveal")
    _add_box(cabinet, (cabinet_w - 0.18, 0.035, 0.045), (0.0, front_y - 0.012, 0.0225), shadow, "recessed_toe_kick")
    _add_box(cabinet, (1.35, 0.74, 0.055), (0.0, -0.005, 0.8475), countertop, "countertop_slab")

    # Shallow inset gas hob on the countertop.
    hob_w = 0.98
    hob_d = 0.52
    _add_box(cabinet, (hob_w, hob_d, 0.014), (0.0, 0.035, 0.882), glass_black, "hob_black_plate")
    _add_box(cabinet, (hob_w + 0.030, 0.012, 0.008), (0.0, 0.035 - hob_d / 2.0 - 0.006, 0.893), brushed_steel, "hob_front_trim")
    _add_box(cabinet, (hob_w + 0.030, 0.012, 0.008), (0.0, 0.035 + hob_d / 2.0 + 0.006, 0.893), brushed_steel, "hob_rear_trim")
    _add_box(cabinet, (0.012, hob_d + 0.012, 0.008), (-hob_w / 2.0 - 0.006, 0.035, 0.893), brushed_steel, "hob_side_trim_0")
    _add_box(cabinet, (0.012, hob_d + 0.012, 0.008), (hob_w / 2.0 + 0.006, 0.035, 0.893), brushed_steel, "hob_side_trim_1")

    _add_burner(cabinet, x=0.0, y=0.045, radius=0.070, name="center", steel=brushed_steel, cap=enamel, grate=cast_iron)
    _add_burner(cabinet, x=-0.315, y=-0.135, radius=0.052, name="front_burner_0", steel=brushed_steel, cap=enamel, grate=cast_iron)
    _add_burner(cabinet, x=0.315, y=-0.135, radius=0.052, name="front_burner_1", steel=brushed_steel, cap=enamel, grate=cast_iron)
    _add_burner(cabinet, x=-0.300, y=0.215, radius=0.058, name="rear_burner_0", steel=brushed_steel, cap=enamel, grate=cast_iron)
    _add_burner(cabinet, x=0.300, y=0.215, radius=0.058, name="rear_burner_1", steel=brushed_steel, cap=enamel, grate=cast_iron)

    small_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.070,
            0.038,
            body_style="skirted",
            top_diameter=0.052,
            skirt=KnobSkirt(0.082, 0.008, flare=0.06, chamfer=0.0012),
            grip=KnobGrip(style="fluted", count=18, depth=0.0015),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            center=False,
        ),
        "small_rotary_knob",
    )
    center_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.088,
            0.043,
            body_style="skirted",
            top_diameter=0.066,
            skirt=KnobSkirt(0.102, 0.010, flare=0.06, chamfer=0.0015),
            grip=KnobGrip(style="fluted", count=22, depth=0.0018),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            center=False,
        ),
        "large_center_rotary_knob",
    )

    # Two slab doors below the fascia, each framed at its hinge line.
    door_h = 0.610
    door_w = 0.535
    door_t = 0.032
    hinge_z = 0.365
    hinge_y = front_y - 0.006
    hinge_x = 0.575
    door_specs = (
        ("door_0", -hinge_x, door_w / 2.0, -1.0),
        ("door_1", hinge_x, -door_w / 2.0, 1.0),
    )
    for door_name, joint_x, panel_x, axis_z in door_specs:
        lug_x = joint_x + (-0.020 if joint_x < 0.0 else 0.020)
        _add_cylinder(
            cabinet,
            radius=0.006,
            length=0.640,
            xyz=(joint_x, hinge_y, hinge_z),
            material=brushed_steel,
            name=f"{door_name}_hinge_pin",
        )
        _add_box(cabinet, (0.020, 0.024, 0.045), (lug_x, hinge_y, hinge_z - 0.3275), brushed_steel, f"{door_name}_lower_hinge_lug")
        _add_box(cabinet, (0.020, 0.024, 0.045), (lug_x, hinge_y, hinge_z + 0.3275), brushed_steel, f"{door_name}_upper_hinge_lug")
        door = model.part(door_name)
        _add_box(door, (door_w, door_t, door_h), (panel_x, -door_t / 2.0 - 0.002, 0.0), cabinet_paint, "door_slab")
        _add_box(door, (0.014, 0.006, door_h * 0.88), (panel_x * 2.0 - (0.018 if panel_x > 0 else -0.018), -door_t - 0.002, 0.0), shadow, "vertical_shadow_pull")
        door.visual(
            Cylinder(radius=0.011, length=0.550),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=brushed_steel,
            name="hinge_sleeve",
        )
        model.articulation(
            f"cabinet_to_{door_name}",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=door,
            origin=Origin(xyz=(joint_x, hinge_y, hinge_z)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.65),
        )

    # Symmetric five-control arrangement; the joint frame turns each local
    # knob axis into a front-to-back world axis.
    knob_xs = (-0.420, -0.235, 0.0, 0.235, 0.420)
    knob_mount_y = front_y - 0.035
    knob_z = 0.742
    for index, x in enumerate(knob_xs):
        knob = model.part(f"knob_{index}")
        knob.visual(
            center_knob_mesh if index == 2 else small_knob_mesh,
            origin=Origin(),
            material=knob_material,
            name="knob_body",
        )
        model.articulation(
            f"cabinet_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=knob,
            origin=Origin(xyz=(x, knob_mount_y, knob_z), rpy=(pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.25, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    knob_joints = [object_model.get_articulation(f"cabinet_to_knob_{index}") for index in range(5)]
    ctx.check(
        "five knobs are continuous rotary joints",
        all(joint.articulation_type == ArticulationType.CONTINUOUS for joint in knob_joints),
        details="All five control knobs should be authored as continuous rotary joints.",
    )
    ctx.check(
        "knob axes are front to back",
        all(abs(joint.axis[2] - 1.0) < 1e-9 and abs(joint.origin.rpy[0] - pi / 2.0) < 1e-9 for joint in knob_joints),
        details="Each knob uses a local Z axis rotated onto the cabinet front-to-back axis.",
    )

    knob_positions = [ctx.part_world_position(object_model.get_part(f"knob_{index}")) for index in range(5)]
    knob_xs = [pos[0] for pos in knob_positions if pos is not None]
    ctx.check(
        "five-control layout is symmetric",
        len(knob_xs) == 5
        and abs(knob_xs[0] + knob_xs[4]) < 1e-6
        and abs(knob_xs[1] + knob_xs[3]) < 1e-6
        and abs(knob_xs[2]) < 1e-6
        and knob_xs[0] < knob_xs[1] < knob_xs[2] < knob_xs[3] < knob_xs[4],
        details=f"knob x positions={knob_xs}",
    )
    ctx.expect_gap(
        "cabinet",
        "knob_2",
        axis="y",
        positive_elem="control_fascia",
        negative_elem="knob_body",
        max_gap=0.004,
        max_penetration=0.001,
        name="center knob is seated on front fascia",
    )

    door_joints = [object_model.get_articulation("cabinet_to_door_0"), object_model.get_articulation("cabinet_to_door_1")]
    for door_name in ("door_0", "door_1"):
        ctx.allow_overlap(
            "cabinet",
            door_name,
            elem_a=f"{door_name}_hinge_pin",
            elem_b="hinge_sleeve",
            reason="The cabinet hinge pin is intentionally captured inside the door hinge sleeve proxy.",
        )
        ctx.expect_within(
            "cabinet",
            door_name,
            axes="xy",
            inner_elem=f"{door_name}_hinge_pin",
            outer_elem="hinge_sleeve",
            margin=0.001,
            name=f"{door_name} hinge pin is centered in sleeve",
        )
        ctx.expect_overlap(
            "cabinet",
            door_name,
            axes="z",
            elem_a=f"{door_name}_hinge_pin",
            elem_b="hinge_sleeve",
            min_overlap=0.50,
            name=f"{door_name} hinge sleeve captures pin height",
        )

    ctx.check(
        "two doors use vertical revolute hinges",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            and abs(abs(joint.axis[2]) - 1.0) < 1e-9
            and joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper > 1.2
            for joint in door_joints
        ),
        details="Both lower cabinet doors should swing on vertical revolute hinge axes.",
    )

    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    closed_0 = ctx.part_world_aabb(door_0)
    closed_1 = ctx.part_world_aabb(door_1)
    with ctx.pose({"cabinet_to_door_0": 1.20, "cabinet_to_door_1": 1.20}):
        open_0 = ctx.part_world_aabb(door_0)
        open_1 = ctx.part_world_aabb(door_1)
        ctx.check(
            "doors swing outward from cabinet bay",
            closed_0 is not None
            and closed_1 is not None
            and open_0 is not None
            and open_1 is not None
            and open_0[0][1] < closed_0[0][1] - 0.18
            and open_1[0][1] < closed_1[0][1] - 0.18,
            details=f"closed_0={closed_0}, open_0={open_0}, closed_1={closed_1}, open_1={open_1}",
        )

    return ctx.report()


object_model = build_object_model()
