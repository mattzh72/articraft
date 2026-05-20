from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CENTER_THICKNESS = 0.022
FORK_THICKNESS = 0.014
FORK_OFFSET = (CENTER_THICKNESS + FORK_THICKNESS) * 0.5
PIN_RADIUS = 0.012
PIVOT_HOLE_RADIUS = 0.017


def _circle_profile(
    cx: float,
    cy: float,
    radius: float,
    *,
    segments: int = 40,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * index / segments),
            cy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _capsule_profile(
    x0: float,
    x1: float,
    width: float,
    *,
    segments: int = 18,
) -> list[tuple[float, float]]:
    radius = width * 0.5
    points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        angle = -math.pi * 0.5 + math.pi * index / segments
        points.append((x1 + radius * math.cos(angle), radius * math.sin(angle)))
    for index in range(segments + 1):
        angle = math.pi * 0.5 + math.pi * index / segments
        points.append((x0 + radius * math.cos(angle), radius * math.sin(angle)))
    return points


def _open_link_plate_mesh(
    name: str,
    *,
    length: float,
    width: float,
    thickness: float,
    proximal_hole: bool = True,
    distal_hole: bool = False,
    central_slot: bool = True,
):
    outer = _capsule_profile(0.0, length, width)
    holes: list[list[tuple[float, float]]] = []
    if proximal_hole:
        holes.append(list(reversed(_circle_profile(0.0, 0.0, PIVOT_HOLE_RADIUS))))
    if distal_hole:
        holes.append(list(reversed(_circle_profile(length, 0.0, PIVOT_HOLE_RADIUS * 0.82))))
    if central_slot:
        slot_width = max(0.030, width - 0.056)
        slot_x0 = width * 0.48
        slot_x1 = length - width * 0.48
        if slot_x1 > slot_x0 + slot_width * 0.35:
            holes.append(list(reversed(_capsule_profile(slot_x0, slot_x1, slot_width, segments=14))))
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, height=thickness, center=True),
        name,
    )


def _fork_cheek_mesh(name: str, *, joint_x: float, width: float, depth: float):
    outer = _capsule_profile(joint_x - depth, joint_x, width)
    holes = [list(reversed(_circle_profile(joint_x, 0.0, PIVOT_HOLE_RADIUS)))]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, height=FORK_THICKNESS, center=True),
        name,
    )


def _tab_mesh(name: str, *, start_x: float, end_x: float, width: float):
    outer = _capsule_profile(start_x, end_x, width)
    holes = [list(reversed(_circle_profile(end_x - 0.010, 0.0, 0.010)))]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, height=CENTER_THICKNESS, center=True),
        name,
    )


def _add_joint_pin(part, *, x: float, material, prefix: str) -> None:
    total_height = CENTER_THICKNESS + 2.0 * FORK_THICKNESS + 0.014
    pin_name = "root_pin" if prefix == "root" else "distal_pin"
    part.visual(
        Cylinder(radius=PIN_RADIUS, length=total_height),
        origin=Origin(xyz=(x, 0.0, 0.0)),
        material=material,
        name=pin_name,
    )
    cap_z = total_height * 0.5 + 0.003
    for z_sign, cap_name in ((1.0, "upper_cap"), (-1.0, "lower_cap")):
        full_cap_name = (
            f"root_{cap_name}" if prefix == "root" else f"distal_{cap_name}"
        )
        part.visual(
            Cylinder(radius=0.026, length=0.006),
            origin=Origin(xyz=(x, 0.0, z_sign * cap_z)),
            material=material,
            name=full_cap_name,
        )


def _add_link_visuals(
    part,
    *,
    mesh_prefix: str,
    length: float,
    width: float,
    material,
    pin_material,
    has_fork: bool,
    end_tab: bool = False,
) -> None:
    if has_fork:
        body_end = length - 0.155
        part.visual(
            _open_link_plate_mesh(
                f"{mesh_prefix}_center_frame",
                length=body_end,
                width=width,
                thickness=CENTER_THICKNESS,
                proximal_hole=True,
                distal_hole=False,
            ),
            material=material,
            name="center_frame",
        )
        cheek = _fork_cheek_mesh(
            f"{mesh_prefix}_fork_cheek",
            joint_x=length,
            width=width * 0.92,
            depth=0.112,
        )
        part.visual(
            cheek,
            origin=Origin(xyz=(0.0, 0.0, FORK_OFFSET)),
            material=material,
            name="upper_fork",
        )
        part.visual(
            cheek,
            origin=Origin(xyz=(0.0, 0.0, -FORK_OFFSET)),
            material=material,
            name="lower_fork",
        )
        part.visual(
            Box((0.050, width * 0.94, CENTER_THICKNESS + 2.0 * FORK_THICKNESS)),
            origin=Origin(xyz=(length - 0.140, 0.0, 0.0)),
            material=material,
            name="fork_bridge",
        )
        _add_joint_pin(part, x=length, material=pin_material, prefix="distal")
    else:
        frame_end = length - 0.065 if end_tab else length
        part.visual(
            _open_link_plate_mesh(
                f"{mesh_prefix}_center_frame",
                length=frame_end,
                width=width,
                thickness=CENTER_THICKNESS,
                proximal_hole=True,
                distal_hole=not end_tab,
            ),
            material=material,
            name="center_frame",
        )
        if end_tab:
            part.visual(
                _tab_mesh(
                    f"{mesh_prefix}_end_tab",
                    start_x=frame_end - 0.010,
                    end_x=length,
                    width=width * 0.72,
                ),
                material=material,
                name="end_tab",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_revolute_chain")

    pedestal_paint = model.material("pedestal_paint", rgba=(0.18, 0.19, 0.20, 1.0))
    link_steel_0 = model.material("link_blue_steel", rgba=(0.18, 0.34, 0.58, 1.0))
    link_steel_1 = model.material("link_teal_steel", rgba=(0.16, 0.47, 0.50, 1.0))
    link_steel_2 = model.material("link_light_steel", rgba=(0.34, 0.58, 0.62, 1.0))
    pin_steel = model.material("brushed_pin_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    foot_rubber = model.material("black_rubber", rgba=(0.035, 0.035, 0.035, 1.0))

    joint_height = 0.360

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.280, 0.220, 0.032)),
        origin=Origin(xyz=(-0.085, 0.0, 0.016)),
        material=pedestal_paint,
        name="base_plate",
    )
    pedestal.visual(
        Box((0.235, 0.175, 0.010)),
        origin=Origin(xyz=(-0.085, 0.0, 0.005)),
        material=foot_rubber,
        name="rubber_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.045, length=0.306),
        origin=Origin(xyz=(-0.085, 0.0, 0.185)),
        material=pedestal_paint,
        name="round_column",
    )
    pedestal.visual(
        Box((0.054, 0.160, CENTER_THICKNESS + 2.0 * FORK_THICKNESS)),
        origin=Origin(xyz=(-0.118, 0.0, joint_height)),
        material=pedestal_paint,
        name="yoke_bridge",
    )
    yoke_mesh = _fork_cheek_mesh("root_yoke_cheek", joint_x=0.0, width=0.155, depth=0.112)
    pedestal.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, joint_height + FORK_OFFSET)),
        material=pedestal_paint,
        name="upper_yoke",
    )
    pedestal.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, joint_height - FORK_OFFSET)),
        material=pedestal_paint,
        name="lower_yoke",
    )
    _add_joint_pin(pedestal, x=0.0, material=pin_steel, prefix="root")
    # Move the pin, authored about the local joint plane, up to the pedestal joint.
    for visual in pedestal.visuals[-3:]:
        visual.origin = Origin(
            xyz=(visual.origin.xyz[0], visual.origin.xyz[1], visual.origin.xyz[2] + joint_height),
            rpy=visual.origin.rpy,
        )

    link_0 = model.part("link_0")
    _add_link_visuals(
        link_0,
        mesh_prefix="link_0",
        length=0.480,
        width=0.140,
        material=link_steel_0,
        pin_material=pin_steel,
        has_fork=True,
    )

    link_1 = model.part("link_1")
    _add_link_visuals(
        link_1,
        mesh_prefix="link_1",
        length=0.380,
        width=0.120,
        material=link_steel_1,
        pin_material=pin_steel,
        has_fork=True,
    )

    link_2 = model.part("link_2")
    _add_link_visuals(
        link_2,
        mesh_prefix="link_2",
        length=0.310,
        width=0.100,
        material=link_steel_2,
        pin_material=pin_steel,
        has_fork=False,
        end_tab=True,
    )

    model.articulation(
        "pedestal_to_link_0",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, joint_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.480, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.8, lower=-1.45, upper=1.45),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-1.55, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [
        object_model.get_articulation("pedestal_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
    ]
    ctx.check(
        "three serial revolute joints",
        len(object_model.articulations) == 3
        and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in joints),
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "joint axes are parallel and vertical",
        all(tuple(round(value, 6) for value in joint.axis) == (0.0, 0.0, 1.0) for joint in joints),
        details=f"axes={[joint.axis for joint in joints]}",
    )

    pedestal = object_model.get_part("pedestal")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")

    ctx.allow_overlap(
        pedestal,
        link_0,
        elem_a="root_pin",
        elem_b="center_frame",
        reason="The root pivot pin is intentionally captured through the first link's proximal bearing lug.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="distal_pin",
        elem_b="center_frame",
        reason="The first link's distal pivot pin is intentionally captured through the second link's proximal bearing lug.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        elem_a="distal_pin",
        elem_b="center_frame",
        reason="The second link's distal pivot pin is intentionally captured through the third link's proximal bearing lug.",
    )

    ctx.expect_gap(
        pedestal,
        link_0,
        axis="z",
        positive_elem="upper_yoke",
        negative_elem="center_frame",
        max_gap=0.001,
        max_penetration=0.0,
        name="root upper yoke bears on the first link lug",
    )
    ctx.expect_gap(
        link_0,
        pedestal,
        axis="z",
        positive_elem="center_frame",
        negative_elem="lower_yoke",
        max_gap=0.001,
        max_penetration=0.0,
        name="root lower yoke bears under the first link lug",
    )
    ctx.expect_overlap(
        pedestal,
        link_0,
        axes="z",
        elem_a="root_pin",
        elem_b="center_frame",
        min_overlap=0.020,
        name="root pin spans the first bearing lug thickness",
    )
    ctx.expect_overlap(
        link_0,
        link_1,
        axes="z",
        elem_a="distal_pin",
        elem_b="center_frame",
        min_overlap=0.020,
        name="first distal pin spans the second bearing lug thickness",
    )
    ctx.expect_overlap(
        link_1,
        link_2,
        axes="z",
        elem_a="distal_pin",
        elem_b="center_frame",
        min_overlap=0.020,
        name="second distal pin spans the third bearing lug thickness",
    )
    ctx.expect_overlap(
        link_1,
        link_0,
        axes="xy",
        min_overlap=0.030,
        elem_a="center_frame",
        elem_b="upper_fork",
        name="second link proximal lug sits inside first fork footprint",
    )
    ctx.expect_overlap(
        link_2,
        link_1,
        axes="xy",
        min_overlap=0.025,
        elem_a="center_frame",
        elem_b="upper_fork",
        name="third link proximal lug sits inside second fork footprint",
    )

    rest_tip = ctx.part_world_position(link_2)
    with ctx.pose(
        {
            joints[0]: 0.55,
            joints[1]: -0.45,
            joints[2]: 0.35,
        }
    ):
        posed_tip = ctx.part_world_position(link_2)
    ctx.check(
        "serial revolute pose moves the end tab laterally",
        rest_tip is not None
        and posed_tip is not None
        and abs(posed_tip[1] - rest_tip[1]) > 0.15,
        details=f"rest={rest_tip}, posed={posed_tip}",
    )

    return ctx.report()


object_model = build_object_model()
