from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ROOT_BEARING_BOTTOM_CAP = "root_bearing_bottom_cap"
ROOT_BEARING_CAP = "root_bearing_cap"


def _arc_points(
    cx: float,
    cy: float,
    radius: float,
    start: float,
    end: float,
    *,
    segments: int = 10,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(start + (end - start) * index / segments),
            cy + radius * math.sin(start + (end - start) * index / segments),
        )
        for index in range(segments + 1)
    ]


def _tapered_finger_body(
    *,
    length: float,
    root_width: float,
    tip_width: float,
    thickness: float,
    mesh_name: str,
):
    """Rounded tapered load-carrying link plate, extruded about local Z."""

    root_radius = root_width * 0.50
    tip_radius = tip_width * 0.50
    profile: list[tuple[float, float]] = []
    # The root half-lobe lives forward of the joint plane while the distal
    # half-lobe lives behind the next joint plane.  Adjacent links can meet at
    # the hinge datum without interpenetrating.
    profile.extend(_arc_points(0.0, 0.0, root_radius, math.pi / 2.0, -math.pi / 2.0))
    profile.extend(
        _arc_points(length, 0.0, tip_radius, -math.pi / 2.0, -3.0 * math.pi / 2.0)
    )
    return mesh_from_geometry(
        ExtrudeGeometry(profile, thickness, center=True, closed=True),
        mesh_name,
    )


def _add_cap_stack(
    part,
    *,
    x: float,
    z: float,
    body_thickness: float,
    radius: float,
    material: Material,
    name: str,
) -> None:
    cap_thickness = 0.006
    bottom_cap_name = ROOT_BEARING_BOTTOM_CAP if name == "root_bearing" else f"{name}_bottom_cap"
    top_cap_name = ROOT_BEARING_CAP if name == "root_bearing" else f"{name}_cap"
    part.visual(
        Cylinder(radius=radius * 0.86, length=cap_thickness),
        origin=Origin(
            xyz=(x, 0.0, z - body_thickness * 0.5 - cap_thickness * 0.5 + 0.001)
        ),
        material=material,
        name=bottom_cap_name,
    )
    part.visual(
        Cylinder(radius=radius, length=cap_thickness),
        origin=Origin(
            xyz=(x, 0.0, z + body_thickness * 0.5 + cap_thickness * 0.5 - 0.001)
        ),
        material=material,
        name=top_cap_name,
    )
    part.visual(
        Cylinder(radius=radius * 0.45, length=body_thickness + cap_thickness * 0.6),
        origin=Origin(xyz=(x, 0.0, z)),
        material=material,
        name=f"{name}_pin_boss",
    )


def _add_finger_link(
    model: ArticulatedObject,
    *,
    part_name: str,
    length: float,
    root_width: float,
    tip_width: float,
    thickness: float,
    z: float,
    body_material: Material,
    cap_material: Material,
    rubber_material: Material | None = None,
    tip_style: str | None = None,
):
    link = model.part(part_name)
    link.visual(
        _tapered_finger_body(
            length=length,
            root_width=root_width,
            tip_width=tip_width,
            thickness=thickness,
            mesh_name=f"{part_name}_body",
        ),
        origin=Origin(xyz=(0.0, 0.0, z)),
        material=body_material,
        name="tapered_plate",
    )
    link.visual(
        Box((0.018, root_width * 0.82, thickness)),
        origin=Origin(xyz=(0.009, 0.0, z)),
        material=body_material,
        name="root_thrust_land",
    )
    link.visual(
        Box((0.012, tip_width * 0.82, thickness)),
        origin=Origin(xyz=(length - 0.006, 0.0, z)),
        material=body_material,
        name="tip_thrust_land",
    )
    _add_cap_stack(
        link,
        x=0.0,
        z=z,
        body_thickness=thickness,
        radius=root_width * 0.36,
        material=cap_material,
        name="root_bearing",
    )
    _add_cap_stack(
        link,
        x=length,
        z=z,
        body_thickness=thickness,
        radius=tip_width * 0.38,
        material=cap_material,
        name="tip_bearing",
    )
    rib_length = max(length - root_width * 0.85 - tip_width * 0.70, 0.015)
    rib_center = root_width * 0.42 + rib_length * 0.5
    rib_z = z + thickness * 0.5 + 0.002
    rib_width = min(root_width, tip_width) * 0.12
    for index, y in enumerate((-min(root_width, tip_width) * 0.23, min(root_width, tip_width) * 0.23)):
        link.visual(
            Box((rib_length, rib_width, 0.004)),
            origin=Origin(xyz=(rib_center, y, rib_z)),
            material=cap_material,
            name=f"machined_rib_{index}",
        )

    if tip_style == "flat_pad" and rubber_material is not None:
        link.visual(
            Box((0.014, tip_width * 1.12, thickness * 1.05)),
            origin=Origin(xyz=(length + 0.0065, 0.0, z)),
            material=rubber_material,
            name="flat_tip_pad",
        )
    elif tip_style == "fork_pad" and rubber_material is not None:
        for index, angle in enumerate((0.38, -0.38)):
            side = 1.0 if index == 0 else -1.0
            link.visual(
                Box((0.030, tip_width * 0.34, thickness * 0.90)),
                origin=Origin(
                    xyz=(length + 0.009, side * tip_width * 0.24, z),
                    rpy=(0.0, 0.0, angle),
                ),
                material=rubber_material,
                name=f"fork_tip_pad_{index}",
            )

    link.inertial = Inertial.from_geometry(
        Box((length + root_width, root_width, thickness + 0.018)),
        mass=0.20 * (length / 0.12) * (root_width / 0.040),
        origin=Origin(xyz=(length * 0.5, 0.0, z)),
    )
    return link


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_gripper_hand_study")

    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.025, 0.026, 0.027, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.56, 0.58, 0.58, 1.0))
    hard_rubber = model.material("hard_rubber", rgba=(0.035, 0.035, 0.032, 1.0))
    warning = model.material("yellow_index_marks", rgba=(0.95, 0.68, 0.12, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((0.500, 0.185, 0.024)),
        origin=Origin(xyz=(0.090, 0.0, 0.012)),
        material=dark_steel,
        name="bench_base",
    )
    palm.visual(
        Box((0.045, 0.190, 0.235)),
        origin=Origin(xyz=(-0.155, 0.0, 0.125)),
        material=dark_steel,
        name="rear_palm_plate",
    )
    palm.visual(
        Box((0.028, 0.175, 0.155)),
        origin=Origin(xyz=(-0.118, 0.0, 0.092)),
        material=black_oxide,
        name="palm_front_web",
    )
    palm.visual(
        Box((0.300, 0.012, 0.022)),
        origin=Origin(xyz=(0.005, 0.083, 0.076)),
        material=brushed,
        name="upper_rail",
    )
    palm.visual(
        Box((0.300, 0.012, 0.022)),
        origin=Origin(xyz=(0.005, -0.083, 0.076)),
        material=brushed,
        name="lower_rail",
    )

    pivot_x = 0.035
    pivot_z = 0.145
    finger_pitch = 0.064
    finger_y = (-finger_pitch * 0.5, finger_pitch * 0.5)
    for index, y in enumerate(finger_y):
        palm.visual(
            Box((0.082, 0.054, 0.054)),
            origin=Origin(xyz=(-0.010, y, 0.050)),
            material=black_oxide,
            name=f"tower_foot_{index}",
        )
        palm.visual(
            Box((0.040, 0.054, 0.135)),
            origin=Origin(xyz=(-0.018, y, 0.106)),
            material=black_oxide,
            name=f"tower_web_{index}",
        )
        for level, z in enumerate((pivot_z - 0.0155, pivot_z + 0.0155)):
            palm.visual(
                Box((0.080, 0.050, 0.007)),
                origin=Origin(xyz=(0.006, y, z)),
                material=brushed,
                name=f"fork_arm_{index}_{level}",
            )
            palm.visual(
                Cylinder(radius=0.017, length=0.008),
                origin=Origin(xyz=(pivot_x, y, z - 0.0005 if level == 0 else z + 0.0005)),
                material=brushed,
                name=f"palm_bearing_cap_{index}_{level}",
            )
        for side_index, side in enumerate((-1.0, 1.0)):
            palm.visual(
                Box((0.034, 0.006, 0.049)),
                origin=Origin(xyz=(-0.010, y + side * 0.028, pivot_z)),
                material=brushed,
                name=f"tower_side_spacer_{index}_{side_index}",
            )
        outward_side = -1.0 if y < 0.0 else 1.0
        palm.visual(
            Box((0.022, 0.010, 0.018)),
            origin=Origin(xyz=(0.045, y + outward_side * 0.028, pivot_z + 0.026)),
            material=warning,
            name=f"travel_stop_{index}",
        )

    palm.inertial = Inertial.from_geometry(
        Box((0.500, 0.190, 0.235)),
        mass=6.0,
        origin=Origin(xyz=(0.035, 0.0, 0.115)),
    )

    lengths = (0.118, 0.094, 0.074)
    widths = ((0.046, 0.034), (0.035, 0.026), (0.027, 0.019))
    thicknesses = (0.014, 0.012, 0.010)
    z_layers = (0.0, 0.023, 0.044)

    for index, y in enumerate(finger_y):
        root = _add_finger_link(
            model,
            part_name=f"root_link_{index}",
            length=lengths[0],
            root_width=widths[0][0],
            tip_width=widths[0][1],
            thickness=thicknesses[0],
            z=z_layers[0],
            body_material=black_oxide,
            cap_material=brushed,
        )
        mid = _add_finger_link(
            model,
            part_name=f"mid_link_{index}",
            length=lengths[1],
            root_width=widths[1][0],
            tip_width=widths[1][1],
            thickness=thicknesses[1],
            z=z_layers[1],
            body_material=dark_steel,
            cap_material=brushed,
        )
        tip = _add_finger_link(
            model,
            part_name=f"tip_link_{index}",
            length=lengths[2],
            root_width=widths[2][0],
            tip_width=widths[2][1],
            thickness=thicknesses[2],
            z=z_layers[2],
            body_material=brushed,
            cap_material=black_oxide,
            rubber_material=hard_rubber,
            tip_style="flat_pad" if index == 0 else "fork_pad",
        )

        outward_sign = -1.0 if y < 0.0 else 1.0
        lower = -0.56 if outward_sign < 0.0 else 0.0
        upper = 0.0 if outward_sign < 0.0 else 0.56
        model.articulation(
            f"palm_to_root_{index}",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=root,
            origin=Origin(xyz=(pivot_x, y, pivot_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=32.0, velocity=1.6, lower=lower, upper=upper),
        )
        model.articulation(
            f"root_to_mid_{index}",
            ArticulationType.REVOLUTE,
            parent=root,
            child=mid,
            origin=Origin(xyz=(lengths[0], 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=lower, upper=upper),
        )
        model.articulation(
            f"mid_to_tip_{index}",
            ArticulationType.REVOLUTE,
            parent=mid,
            child=tip,
            origin=Origin(xyz=(lengths[1], 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=9.0, velocity=2.0, lower=lower, upper=upper),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [object_model.get_articulation(name) for name in (
        "palm_to_root_0",
        "root_to_mid_0",
        "mid_to_tip_0",
        "palm_to_root_1",
        "root_to_mid_1",
        "mid_to_tip_1",
    )]
    ctx.check(
        "six independent revolute joints",
        all(joint is not None and joint.articulation_type == ArticulationType.REVOLUTE for joint in joints)
        and all(getattr(joint, "mimic", None) is None for joint in joints),
        details=f"joints={joints}",
    )

    for link_name in ("root_link", "mid_link", "tip_link"):
        ctx.expect_gap(
            f"{link_name}_1",
            f"{link_name}_0",
            axis="y",
            min_gap=0.006,
            name=f"{link_name} center chains remain separated",
        )

    lower_tip = object_model.get_part("tip_link_0")
    upper_tip = object_model.get_part("tip_link_1")
    rest_lower = ctx.part_world_position(lower_tip)
    rest_upper = ctx.part_world_position(upper_tip)
    with ctx.pose(
        {
            "palm_to_root_0": -0.56,
            "root_to_mid_0": -0.56,
            "mid_to_tip_0": -0.56,
            "palm_to_root_1": 0.56,
            "root_to_mid_1": 0.56,
            "mid_to_tip_1": 0.56,
        }
    ):
        curled_lower = ctx.part_world_position(lower_tip)
        curled_upper = ctx.part_world_position(upper_tip)
        ctx.expect_gap(
            upper_tip,
            lower_tip,
            axis="y",
            min_gap=0.020,
            name="independent curled tips stay clear",
        )
        for index in (0, 1):
            ctx.expect_gap(
                f"mid_link_{index}",
                f"root_link_{index}",
                axis="z",
                min_gap=0.0,
                max_gap=0.001,
                name=f"curled root mid stack clears {index}",
            )
            ctx.expect_gap(
                f"tip_link_{index}",
                f"mid_link_{index}",
                axis="z",
                min_gap=0.0,
                max_gap=0.001,
                name=f"curled mid tip stack clears {index}",
            )
            ctx.expect_gap(
                f"root_link_{index}",
                "palm",
                axis="z",
                positive_elem=ROOT_BEARING_BOTTOM_CAP,
                negative_elem=f"fork_arm_{index}_0",
                max_penetration=0.0001,
                max_gap=0.0002,
                name=f"lower yoke bearing clears {index}",
            )
            ctx.expect_gap(
                "palm",
                f"root_link_{index}",
                axis="z",
                positive_elem=f"fork_arm_{index}_1",
                negative_elem=ROOT_BEARING_CAP,
                max_penetration=0.0001,
                max_gap=0.0002,
                name=f"upper yoke bearing clears {index}",
            )

    ctx.check(
        "outward curl increases chain spacing",
        rest_lower is not None
        and rest_upper is not None
        and curled_lower is not None
        and curled_upper is not None
        and (curled_upper[1] - curled_lower[1]) > (rest_upper[1] - rest_lower[1]) + 0.035,
        details=f"rest=({rest_lower}, {rest_upper}), curled=({curled_lower}, {curled_upper})",
    )

    return ctx.report()


object_model = build_object_model()
