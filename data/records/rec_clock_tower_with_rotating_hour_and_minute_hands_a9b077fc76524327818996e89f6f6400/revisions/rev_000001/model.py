from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="market_square_clock_tower")

    stone = model.material("warm_grey_limestone", rgba=(0.56, 0.54, 0.49, 1.0))
    trim = model.material("pale_stone_trim", rgba=(0.72, 0.70, 0.64, 1.0))
    slate = model.material("dark_slate_roof", rgba=(0.11, 0.14, 0.17, 1.0))
    clock_ivory = model.material("aged_clock_dial", rgba=(0.92, 0.88, 0.74, 1.0))
    black = model.material("black_enamel", rgba=(0.015, 0.013, 0.011, 1.0))
    bronze = model.material("aged_bronze", rgba=(0.56, 0.36, 0.14, 1.0))
    gold = model.material("dull_gilding", rgba=(0.78, 0.58, 0.22, 1.0))

    tower = model.part("tower")

    # Grounded plinth and the square masonry shaft.
    tower.visual(Box((5.2, 5.2, 0.55)), origin=Origin(xyz=(0.0, 0.0, 0.275)), material=trim, name="broad_plinth")
    tower.visual(Box((4.45, 4.45, 0.45)), origin=Origin(xyz=(0.0, 0.0, 0.775)), material=stone, name="stepped_plinth")
    tower.visual(Box((3.85, 3.85, 17.2)), origin=Origin(xyz=(0.0, 0.0, 9.35)), material=stone, name="square_shaft")

    # Horizontal string courses make the long shaft read as stacked stonework.
    for i, z in enumerate((1.6, 4.8, 8.0, 11.2, 14.4, 17.6)):
        tower.visual(
            Box((4.08, 4.08, 0.16)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=trim,
            name=f"stone_course_{i}",
        )

    # Four slightly proud corner quoins, intentionally embedded into the shaft.
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            tower.visual(
                Box((0.36, 0.36, 16.9)),
                origin=Origin(xyz=(sx * 1.825, sy * 1.825, 9.28)),
                material=trim,
                name=f"corner_quoin_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )

    # Four-faced clock stage with cornices.
    tower.visual(Box((5.35, 5.35, 0.40)), origin=Origin(xyz=(0.0, 0.0, 18.1)), material=trim, name="lower_clock_cornice")
    tower.visual(Box((5.05, 5.05, 3.15)), origin=Origin(xyz=(0.0, 0.0, 19.85)), material=stone, name="clock_stage")
    tower.visual(Box((5.45, 5.45, 0.42)), origin=Origin(xyz=(0.0, 0.0, 21.62)), material=trim, name="upper_clock_cornice")

    dial_radius = 0.82
    dial_thickness = 0.10
    dial_center_z = 19.85
    stage_half = 5.05 / 2.0
    dial_center_offset = stage_half + dial_thickness / 2.0 - 0.006
    face_specs = [
        ("face_0", (0.0, dial_center_offset, dial_center_z), (-math.pi / 2.0, 0.0, 0.0), (0.0, 1.0, 0.0), "y+"),
        ("face_1", (dial_center_offset, 0.0, dial_center_z), (0.0, math.pi / 2.0, 0.0), (1.0, 0.0, 0.0), "x+"),
        ("face_2", (0.0, -dial_center_offset, dial_center_z), (math.pi / 2.0, 0.0, 0.0), (0.0, -1.0, 0.0), "y-"),
        ("face_3", (-dial_center_offset, 0.0, dial_center_z), (0.0, -math.pi / 2.0, 0.0), (-1.0, 0.0, 0.0), "x-"),
    ]

    ring_mesh = mesh_from_geometry(TorusGeometry(dial_radius + 0.055, 0.035, radial_segments=24, tubular_segments=12), "bronze_clock_bezel")
    axle_length = 0.092
    for face_index, (face_name, xyz, rpy, face_axis, side) in enumerate(face_specs):
        tower.visual(
            Cylinder(radius=dial_radius, length=dial_thickness),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=clock_ivory,
            name=f"{face_name}_dial",
        )
        tower.visual(
            ring_mesh,
            origin=Origin(xyz=xyz, rpy=rpy),
            material=bronze,
            name=f"{face_name}_bezel",
        )

        # Twelve raised hour ticks are set into each dial face.
        for hour in range(12):
            angle = 2.0 * math.pi * hour / 12.0
            tangential = angle
            long_tick = hour in (0, 3, 6, 9)
            tick_w = 0.045 if long_tick else 0.030
            tick_l = 0.18 if long_tick else 0.12
            r = 0.64
            u = r * math.sin(angle)
            z = dial_center_z + r * math.cos(angle)
            if side == "y+":
                origin = Origin(
                    xyz=(u, dial_center_offset + dial_thickness * 0.52, z),
                    rpy=(0.0, 0.0, -tangential),
                )
                size = (tick_w, 0.018, tick_l)
            elif side == "y-":
                origin = Origin(
                    xyz=(-u, -dial_center_offset - dial_thickness * 0.52, z),
                    rpy=(0.0, 0.0, tangential),
                )
                size = (tick_w, 0.018, tick_l)
            elif side == "x+":
                origin = Origin(
                    xyz=(dial_center_offset + dial_thickness * 0.52, -u, z),
                    rpy=(0.0, 0.0, -tangential),
                )
                size = (0.018, tick_w, tick_l)
            else:
                origin = Origin(
                    xyz=(-dial_center_offset - dial_thickness * 0.52, u, z),
                    rpy=(0.0, 0.0, tangential),
                )
                size = (0.018, tick_w, tick_l)
            tower.visual(Box(size), origin=origin, material=black, name=f"{face_name}_tick_{hour}")

        axle_offset = dial_thickness / 2.0 + axle_length / 2.0 - 0.004
        tower.visual(
            Cylinder(radius=0.026, length=axle_length),
            origin=Origin(
                xyz=(
                    xyz[0] + face_axis[0] * axle_offset,
                    xyz[1] + face_axis[1] * axle_offset,
                    xyz[2] + face_axis[2] * axle_offset,
                ),
                rpy=rpy,
            ),
            material=black,
            name=f"face_{face_index}_axle",
        )

    # Open bell stage: posts and beams leave large windows, with a visible bronze bell.
    bell_base_z = 21.85
    bell_top_z = 24.80
    tower.visual(Box((5.05, 5.05, 0.35)), origin=Origin(xyz=(0.0, 0.0, bell_base_z)), material=trim, name="bell_stage_floor")
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            tower.visual(
                Box((0.58, 0.58, 2.85)),
                origin=Origin(xyz=(sx * 2.06, sy * 2.06, 23.18)),
                material=stone,
                name=f"belfry_pier_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )
    tower.visual(Box((5.05, 0.42, 0.42)), origin=Origin(xyz=(0.0, 2.06, 24.45)), material=trim, name="front_belfry_lintel")
    tower.visual(Box((5.05, 0.42, 0.42)), origin=Origin(xyz=(0.0, -2.06, 24.45)), material=trim, name="rear_belfry_lintel")
    tower.visual(Box((0.42, 5.05, 0.42)), origin=Origin(xyz=(2.06, 0.0, 24.45)), material=trim, name="side_belfry_lintel_0")
    tower.visual(Box((0.42, 5.05, 0.42)), origin=Origin(xyz=(-2.06, 0.0, 24.45)), material=trim, name="side_belfry_lintel_1")
    tower.visual(Box((4.15, 0.30, 0.26)), origin=Origin(xyz=(0.0, 0.0, 23.82)), material=bronze, name="bell_crossbeam")
    tower.visual(Box((0.30, 4.15, 0.26)), origin=Origin(xyz=(2.06, 0.0, 23.82)), material=bronze, name="side_bell_beam_0")
    tower.visual(Box((0.30, 4.15, 0.26)), origin=Origin(xyz=(-2.06, 0.0, 23.82)), material=bronze, name="side_bell_beam_1")
    tower.visual(Cylinder(radius=0.055, length=0.74), origin=Origin(xyz=(0.0, 0.0, 23.46)), material=bronze, name="bell_hanger")
    bell_profile = [
        (0.0, 0.0),
        (0.70, 0.0),
        (0.66, 0.12),
        (0.56, 0.30),
        (0.43, 0.68),
        (0.26, 0.96),
        (0.13, 1.08),
        (0.0, 1.08),
    ]
    tower.visual(
        mesh_from_geometry(LatheGeometry(bell_profile, segments=40), "hanging_bell"),
        origin=Origin(xyz=(0.0, 0.0, 22.25)),
        material=bronze,
        name="hanging_bell",
    )
    tower.visual(Sphere(radius=0.11), origin=Origin(xyz=(0.0, 0.0, 22.18)), material=black, name="bell_clapper")
    tower.visual(Cylinder(radius=0.018, length=0.95), origin=Origin(xyz=(0.0, 0.0, 22.76)), material=black, name="clapper_rod")

    # Square pyramidal spire with small finial.
    tower.visual(Box((5.35, 5.35, 0.38)), origin=Origin(xyz=(0.0, 0.0, bell_top_z)), material=trim, name="spire_cornice")
    tower.visual(
        mesh_from_geometry(ConeGeometry(1.90, 5.45, radial_segments=4), "slate_square_spire"),
        origin=Origin(xyz=(0.0, 0.0, bell_top_z + 2.82), rpy=(0.0, 0.0, math.pi / 4.0)),
        material=slate,
        name="square_spire",
    )
    tower.visual(Cylinder(radius=0.055, length=0.70), origin=Origin(xyz=(0.0, 0.0, 30.66)), material=gold, name="finial_rod")
    tower.visual(Sphere(radius=0.15), origin=Origin(xyz=(0.0, 0.0, 31.12)), material=gold, name="finial_ball")

    def add_hand(face_index: int, kind: str, face_xyz: tuple[float, float, float], axis: tuple[float, float, float], side: str) -> None:
        is_minute = kind == "minute"
        hand = model.part(f"{kind}_hand_{face_index}")
        length = 0.68 if is_minute else 0.46
        width = 0.045 if is_minute else 0.070
        thickness = 0.022 if is_minute else 0.020
        layer = 0.052 if is_minute else 0.020
        hub_radius = 0.060 if is_minute else 0.085
        base_gap = 0.058 if is_minute else 0.078
        blade_length = length - base_gap
        blade_center_z = base_gap + blade_length / 2.0
        color: Material = black if is_minute else gold

        if side == "y+":
            hand.visual(Box((width, thickness, blade_length)), origin=Origin(xyz=(0.0, layer, blade_center_z)), material=color, name="blade")
            hand.visual(Cylinder(radius=hub_radius, length=0.020), origin=Origin(xyz=(0.0, layer, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=color, name="hub")
        elif side == "y-":
            hand.visual(Box((width, thickness, blade_length)), origin=Origin(xyz=(0.0, -layer, blade_center_z)), material=color, name="blade")
            hand.visual(Cylinder(radius=hub_radius, length=0.020), origin=Origin(xyz=(0.0, -layer, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=color, name="hub")
        elif side == "x+":
            hand.visual(Box((thickness, width, blade_length)), origin=Origin(xyz=(layer, 0.0, blade_center_z)), material=color, name="blade")
            hand.visual(Cylinder(radius=hub_radius, length=0.020), origin=Origin(xyz=(layer, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=color, name="hub")
        else:
            hand.visual(Box((thickness, width, blade_length)), origin=Origin(xyz=(-layer, 0.0, blade_center_z)), material=color, name="blade")
            hand.visual(Cylinder(radius=hub_radius, length=0.020), origin=Origin(xyz=(-layer, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)), material=color, name="hub")

        model.articulation(
            f"tower_to_{kind}_hand_{face_index}",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=hand,
            origin=Origin(xyz=face_xyz),
            axis=axis,
            motion_limits=MotionLimits(effort=0.08, velocity=1.2 if is_minute else 0.1),
        )

    hand_gap = 0.008
    for face_index, (_face_name, xyz, _rpy, axis, side) in enumerate(face_specs):
        x, y, z = xyz
        if side == "y+":
            joint_xyz = (x, y + dial_thickness / 2.0 + hand_gap, z)
        elif side == "y-":
            joint_xyz = (x, y - dial_thickness / 2.0 - hand_gap, z)
        elif side == "x+":
            joint_xyz = (x + dial_thickness / 2.0 + hand_gap, y, z)
        else:
            joint_xyz = (x - dial_thickness / 2.0 - hand_gap, y, z)
        add_hand(face_index, "hour", joint_xyz, axis, side)
        add_hand(face_index, "minute", joint_xyz, axis, side)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tower = object_model.get_part("tower")
    continuous_axes = {
        "tower_to_hour_hand_0": (0.0, 1.0, 0.0),
        "tower_to_minute_hand_0": (0.0, 1.0, 0.0),
        "tower_to_hour_hand_1": (1.0, 0.0, 0.0),
        "tower_to_minute_hand_1": (1.0, 0.0, 0.0),
        "tower_to_hour_hand_2": (0.0, -1.0, 0.0),
        "tower_to_minute_hand_2": (0.0, -1.0, 0.0),
        "tower_to_hour_hand_3": (-1.0, 0.0, 0.0),
        "tower_to_minute_hand_3": (-1.0, 0.0, 0.0),
    }
    for joint_name, expected_axis in continuous_axes.items():
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{joint_name} has coaxial face normal axis",
            tuple(round(v, 6) for v in joint.axis) == expected_axis,
            details=f"axis={joint.axis}, expected={expected_axis}",
        )

    # At rest the two hands on each dial are centered over their dial, separated
    # from the stone face, and staggered in depth so the coaxial hands do not
    # collide.
    for idx in range(4):
        hour = object_model.get_part(f"hour_hand_{idx}")
        minute = object_model.get_part(f"minute_hand_{idx}")
        face_plane_axes = "xz" if idx in (0, 2) else "yz"
        for hand, kind in ((hour, "hour"), (minute, "minute")):
            ctx.allow_overlap(
                hand,
                tower,
                elem_a="hub",
                elem_b=f"face_{idx}_axle",
                reason="The clock hand hub is intentionally captured around the fixed center axle.",
            )
            ctx.expect_overlap(
                hand,
                tower,
                axes=face_plane_axes,
                elem_a="hub",
                elem_b=f"face_{idx}_axle",
                min_overlap=0.02,
                name=f"{kind} hand {idx} is captured on its axle",
            )
        ctx.expect_overlap(hour, tower, axes="z", elem_a="blade", elem_b=f"face_{idx}_dial", min_overlap=0.30, name=f"hour hand {idx} lies on its dial height")
        ctx.expect_overlap(minute, tower, axes="z", elem_a="blade", elem_b=f"face_{idx}_dial", min_overlap=0.45, name=f"minute hand {idx} lies on its dial height")
        positive_hand, negative_hand = (minute, hour) if idx in (0, 1) else (hour, minute)
        ctx.expect_gap(
            positive_hand,
            negative_hand,
            axis=("y" if idx in (0, 2) else "x"),
            min_gap=0.006,
            name=f"minute hand {idx} is proud of hour hand",
        )

    with ctx.pose({"tower_to_minute_hand_0": math.pi / 2.0, "tower_to_hour_hand_0": math.pi / 4.0}):
        ctx.expect_overlap("minute_hand_0", tower, axes="xz", elem_a="blade", elem_b="face_0_dial", min_overlap=0.02, name="rotated minute hand remains centered on face")

    return ctx.report()


object_model = build_object_model()
