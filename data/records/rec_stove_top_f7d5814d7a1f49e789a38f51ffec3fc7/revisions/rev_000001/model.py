from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


COOKTOP_WIDTH = 0.91
COOKTOP_DEPTH = 0.52
COOKTOP_HEIGHT = 0.055
TOP_Z = COOKTOP_HEIGHT


def _rail_x(part, length: float, radius: float, xyz: tuple[float, float, float], *, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _rail_y(part, length: float, radius: float, xyz: tuple[float, float, float], *, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _post_z(part, length: float, radius: float, xyz: tuple[float, float, float], *, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_burner_gas_cooktop")

    stainless = model.material("stainless", rgba=(0.77, 0.78, 0.79, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.67, 0.68, 0.70, 1.0))
    enamel_black = model.material("enamel_black", rgba=(0.11, 0.11, 0.12, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.18, 0.18, 0.19, 1.0))
    burner_black = model.material("burner_black", rgba=(0.14, 0.14, 0.15, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_hub = model.material("knob_hub", rgba=(0.58, 0.60, 0.62, 1.0))

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.024,
            body_style="skirted",
            top_diameter=0.034,
            skirt=KnobSkirt(0.052, 0.006, flare=0.08),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
            center=False,
        ),
        "cooktop_knob",
    )

    body = model.part("body")
    body.visual(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, COOKTOP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, COOKTOP_HEIGHT * 0.5)),
        material=stainless,
        name="chassis",
    )
    body.visual(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, COOKTOP_HEIGHT - 0.003)),
        material=satin_steel,
        name="top_skin",
    )
    body.visual(
        Box((COOKTOP_WIDTH - 0.02, 0.10, 0.003)),
        origin=Origin(xyz=(0.0, -0.18, COOKTOP_HEIGHT - 0.0015)),
        material=enamel_black,
        name="control_strip",
    )
    for index, marker_x in enumerate((-0.32, -0.16, 0.0, 0.16, 0.32)):
        body.visual(
            Box((0.004, 0.0015, 0.012)),
            origin=Origin(xyz=(marker_x, -(COOKTOP_DEPTH * 0.5) + 0.00075, 0.036)),
            material=burner_black,
            name=f"marker_{index}",
        )

    burner_specs = [
        ("front_left_burner", (-0.28, -0.10), 0.066, 0.041, 0.023),
        ("rear_left_burner", (-0.28, 0.12), 0.066, 0.041, 0.023),
        ("center_burner", (0.0, 0.01), 0.096, 0.062, 0.035),
        ("front_right_burner", (0.28, -0.10), 0.066, 0.041, 0.023),
        ("rear_right_burner", (0.28, 0.12), 0.066, 0.041, 0.023),
    ]

    for burner_name, (burner_x, burner_y), tray_radius, head_radius, cap_radius in burner_specs:
        burner = model.part(burner_name)
        burner.visual(
            Cylinder(radius=tray_radius, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=enamel_black,
            name="tray",
        )
        burner.visual(
            Cylinder(radius=head_radius * 0.74, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=satin_steel,
            name="body",
        )
        burner.visual(
            Cylinder(radius=head_radius, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
            material=burner_black,
            name="head",
        )
        if burner_name == "center_burner":
            burner.visual(
                Cylinder(radius=head_radius * 0.56, length=0.006),
                origin=Origin(xyz=(0.0, 0.0, 0.020)),
                material=satin_steel,
                name="inner_ring",
            )
            cap_z = 0.026
        else:
            cap_z = 0.020
        burner.visual(
            Cylinder(radius=cap_radius, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, cap_z)),
            material=burner_black,
            name="cap",
        )
        model.articulation(
            f"body_to_{burner_name}",
            ArticulationType.FIXED,
            parent=body,
            child=burner,
            origin=Origin(xyz=(burner_x, burner_y, TOP_Z)),
        )

    def add_double_grate(name: str, x_center: float) -> None:
        grate = model.part(name)
        rail_radius = 0.0055
        frame_z = 0.030
        foot_z = 0.013
        frame_x = 0.19
        frame_y = 0.35

        _rail_x(grate, frame_x, rail_radius, (0.0, -0.165, frame_z), material=cast_iron, name="front_rail")
        _rail_x(grate, frame_x, rail_radius, (0.0, 0.165, frame_z), material=cast_iron, name="rear_rail")
        _rail_y(grate, frame_y, rail_radius, (-0.075, 0.0, frame_z), material=cast_iron, name="left_rail")
        _rail_y(grate, frame_y, rail_radius, (0.075, 0.0, frame_z), material=cast_iron, name="right_rail")
        _rail_y(grate, frame_y - 0.04, rail_radius, (0.0, 0.0, frame_z), material=cast_iron, name="center_rail")
        _rail_x(grate, 0.14, rail_radius, (0.0, -0.10, frame_z), material=cast_iron, name="front_bridge")
        _rail_x(grate, 0.14, rail_radius, (0.0, 0.12, frame_z), material=cast_iron, name="rear_bridge")

        foot_offsets = (
            (-0.075, -0.165),
            (0.075, -0.165),
            (-0.075, 0.165),
            (0.075, 0.165),
        )
        for index, (foot_x, foot_y) in enumerate(foot_offsets):
            _post_z(grate, 0.026, 0.0045, (foot_x, foot_y, foot_z), material=cast_iron, name=f"foot_{index}")

        model.articulation(
            f"body_to_{name}",
            ArticulationType.FIXED,
            parent=body,
            child=grate,
            origin=Origin(xyz=(x_center, 0.01, TOP_Z)),
        )

    def add_center_grate() -> None:
        grate = model.part("center_grate")
        rail_radius = 0.0058
        frame_z = 0.031
        foot_z = 0.013

        _rail_x(grate, 0.24, rail_radius, (0.0, -0.10, frame_z), material=cast_iron, name="front_rail")
        _rail_x(grate, 0.24, rail_radius, (0.0, 0.10, frame_z), material=cast_iron, name="rear_rail")
        _rail_y(grate, 0.20, rail_radius, (-0.10, 0.0, frame_z), material=cast_iron, name="left_rail")
        _rail_y(grate, 0.20, rail_radius, (0.10, 0.0, frame_z), material=cast_iron, name="right_rail")
        _rail_x(grate, 0.22, rail_radius, (0.0, 0.0, frame_z), material=cast_iron, name="center_bridge")
        _rail_y(grate, 0.22, rail_radius, (0.0, 0.0, frame_z), material=cast_iron, name="center_spine")

        for index, (foot_x, foot_y) in enumerate(((-0.10, -0.10), (0.10, -0.10), (-0.10, 0.10), (0.10, 0.10))):
            _post_z(grate, 0.026, 0.0045, (foot_x, foot_y, foot_z), material=cast_iron, name=f"foot_{index}")

        model.articulation(
            "body_to_center_grate",
            ArticulationType.FIXED,
            parent=body,
            child=grate,
            origin=Origin(xyz=(0.0, 0.01, TOP_Z)),
        )

    add_double_grate("left_grate", -0.28)
    add_center_grate()
    add_double_grate("right_grate", 0.28)

    knob_positions = (-0.32, -0.16, 0.0, 0.16, 0.32)
    for index, knob_x in enumerate(knob_positions):
        knob = model.part(f"knob_{index}")
        knob.visual(
            Cylinder(radius=0.0065, length=0.008),
            origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_hub,
            name="shaft",
        )
        knob.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_dark,
            name="cap",
        )
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, -(COOKTOP_DEPTH * 0.5), 0.028)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.25, velocity=10.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("cooktop_body_aabb_present", body_aabb is not None, "Expected a world AABB for the cooktop body.")
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("cooktop_width_is_realistic", 0.86 <= size[0] <= 0.96, f"size={size!r}")
        ctx.check("cooktop_depth_is_realistic", 0.48 <= size[1] <= 0.56, f"size={size!r}")
        ctx.check("cooktop_has_front_control_strip", size[2] >= 0.05, f"size={size!r}")

    def part_size(part_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_world_aabb(object_model.get_part(part_name))
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple(float(maxs[i] - mins[i]) for i in range(3))

    center_size = part_size("center_burner")
    outer_size = part_size("front_left_burner")
    ctx.check(
        "burner_sizes_present",
        center_size is not None and outer_size is not None,
        f"center_size={center_size!r}, outer_size={outer_size!r}",
    )
    if center_size is not None and outer_size is not None:
        ctx.check(
            "center_burner_is_wider",
            max(center_size[0], center_size[1]) > max(outer_size[0], outer_size[1]) + 0.04,
            f"center_size={center_size!r}, outer_size={outer_size!r}",
        )

    knob_names = [f"knob_{index}" for index in range(5)]
    knob_joint_names = [f"body_to_knob_{index}" for index in range(5)]
    knob_positions: list[tuple[float, float, float]] = []
    for joint_name in knob_joint_names:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name}_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"joint_type={joint.articulation_type!r}",
        )

    for index, knob_name in enumerate(knob_names):
        knob = object_model.get_part(knob_name)
        ctx.expect_gap(
            body,
            knob,
            axis="y",
            max_gap=0.0025,
            max_penetration=0.0,
            name=f"{knob_name}_seats_on_front_strip",
        )
        position = ctx.part_world_position(knob)
        ctx.check(f"{knob_name}_position_present", position is not None, f"position={position!r}")
        if position is not None:
            knob_positions.append(position)

    ctx.check("five_knob_positions_present", len(knob_positions) == 5, f"knob_positions={knob_positions!r}")
    if len(knob_positions) == 5:
        xs = [position[0] for position in knob_positions]
        ys = [position[1] for position in knob_positions]
        zs = [position[2] for position in knob_positions]
        ctx.check("knobs_form_one_front_row", max(ys) - min(ys) < 1e-6, f"ys={ys!r}")
        ctx.check("knobs_share_common_height", max(zs) - min(zs) < 1e-6, f"zs={zs!r}")
        ctx.check("knob_row_spans_full_front", max(xs) - min(xs) > 0.55, f"xs={xs!r}")

    return ctx.report()


object_model = build_object_model()
