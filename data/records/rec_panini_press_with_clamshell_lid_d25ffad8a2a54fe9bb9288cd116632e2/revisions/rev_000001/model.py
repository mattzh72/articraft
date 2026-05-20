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
    wire_from_points,
)


def _add_grill_ribs(
    part,
    *,
    prefix: str,
    width: float,
    depth: float,
    count: int,
    rib_y_center: float,
    rib_z_center: float,
    rib_y_thickness: float,
    rib_height: float,
    material,
) -> None:
    usable_depth = depth - 0.04
    if count <= 1:
        positions = [rib_y_center]
    else:
        start = rib_y_center - usable_depth / 2.0
        step = usable_depth / (count - 1)
        positions = [start + i * step for i in range(count)]

    for i, y in enumerate(positions):
        part.visual(
            Box((width, rib_y_thickness, rib_height)),
            origin=Origin(xyz=(0.0, y, rib_z_center)),
            material=material,
            name=f"{prefix}_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panini_press")

    body_black = model.material("body_black", rgba=(0.13, 0.13, 0.14, 1.0))
    grill_steel = model.material("grill_steel", rgba=(0.63, 0.64, 0.66, 1.0))
    handle_steel = model.material("handle_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    dial_black = model.material("dial_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.352, 0.34, 0.078)),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=body_black,
        name="base_shell",
    )
    base.visual(
        Box((0.31, 0.27, 0.010)),
        origin=Origin(xyz=(0.0, -0.006, 0.083)),
        material=grill_steel,
        name="lower_plate",
    )
    _add_grill_ribs(
        base,
        prefix="lower_rib",
        width=0.29,
        depth=0.23,
        count=7,
        rib_y_center=-0.006,
        rib_z_center=0.091,
        rib_y_thickness=0.012,
        rib_height=0.006,
        material=grill_steel,
    )
    base.visual(
        Box((0.31, 0.016, 0.022)),
        origin=Origin(xyz=(0.0, 0.156, 0.089)),
        material=body_black,
        name="rear_bracket",
    )
    base.visual(
        Box((0.018, 0.030, 0.094)),
        origin=Origin(xyz=(-0.154, 0.156, 0.147)),
        material=body_black,
        name="rear_cheek_0",
    )
    base.visual(
        Box((0.018, 0.030, 0.094)),
        origin=Origin(xyz=(0.154, 0.156, 0.147)),
        material=body_black,
        name="rear_cheek_1",
    )
    base.visual(
        Cylinder(radius=0.025, length=0.006),
        origin=Origin(xyz=(0.173, 0.070, 0.048), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_black,
        name="dial_bezel",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.002),
        origin=Origin(xyz=(0.177, 0.070, 0.048), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_steel,
        name="dial_spindle",
    )
    for x in (-0.145, 0.145):
        for y in (-0.125, 0.125):
            base.visual(
                Box((0.040, 0.040, 0.010)),
                origin=Origin(xyz=(x, y, 0.005)),
                material=body_black,
                name=f"foot_{int((x > 0))}_{int((y > 0))}",
            )

    lid = model.part("lid")
    lid.visual(
        Box((0.36, 0.25, 0.080)),
        origin=Origin(xyz=(0.0, -0.155, -0.041)),
        material=body_black,
        name="upper_shell_front",
    )
    lid.visual(
        Box((0.27, 0.060, 0.080)),
        origin=Origin(xyz=(0.0, -0.030, -0.041)),
        material=body_black,
        name="upper_shell_rear",
    )
    lid.visual(
        Box((0.31, 0.255, 0.010)),
        origin=Origin(xyz=(0.0, -0.145, -0.085)),
        material=grill_steel,
        name="upper_plate",
    )
    _add_grill_ribs(
        lid,
        prefix="upper_rib",
        width=0.29,
        depth=0.22,
        count=7,
        rib_y_center=-0.145,
        rib_z_center=-0.0915,
        rib_y_thickness=0.012,
        rib_height=0.003,
        material=grill_steel,
    )
    lid.visual(
        Box((0.042, 0.020, 0.030)),
        origin=Origin(xyz=(-0.125, -0.004, -0.010)),
        material=body_black,
        name="hinge_mount_0",
    )
    lid.visual(
        Box((0.042, 0.020, 0.030)),
        origin=Origin(xyz=(0.125, -0.004, -0.010)),
        material=body_black,
        name="hinge_mount_1",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(-0.139, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_steel,
        name="hinge_stub_0",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.139, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_steel,
        name="hinge_stub_1",
    )
    lid.visual(
        Box((0.034, 0.018, 0.025)),
        origin=Origin(xyz=(-0.127, -0.286, -0.036)),
        material=body_black,
        name="handle_mount_0",
    )
    lid.visual(
        Box((0.034, 0.018, 0.025)),
        origin=Origin(xyz=(0.127, -0.286, -0.036)),
        material=body_black,
        name="handle_mount_1",
    )
    handle_path = wire_from_points(
        [
            (-0.122, 0.0, 0.0),
            (-0.095, -0.036, -0.012),
            (0.095, -0.036, -0.012),
            (0.122, 0.0, 0.0),
        ],
        radius=0.007,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.020,
        corner_segments=12,
        radial_segments=22,
    )
    lid.visual(
        mesh_from_geometry(handle_path, "front_handle"),
        origin=Origin(xyz=(0.0, -0.286, -0.036)),
        material=handle_steel,
        name="front_handle",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.041,
                0.026,
                body_style="skirted",
                top_diameter=0.034,
                skirt=KnobSkirt(0.051, 0.005, flare=0.06),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "browning_dial",
        ),
        material=dial_black,
        name="knob_shell",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.155, 0.190)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.178, 0.070, 0.048), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("dial")
    lid_hinge = object_model.get_articulation("base_to_lid")
    dial_joint = object_model.get_articulation("base_to_dial")

    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="upper_plate",
        elem_b="lower_plate",
        min_overlap=0.240,
        name="upper platen covers the lower grill",
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="upper_plate",
        negative_elem="lower_plate",
        min_gap=0.010,
        max_gap=0.026,
        name="closed plates stay close without touching",
    )
    ctx.expect_contact(
        lid,
        base,
        elem_a="hinge_stub_0",
        elem_b="rear_cheek_0",
        name="left hinge stub is carried by the rear bracket",
    )
    ctx.expect_contact(
        lid,
        base,
        elem_a="hinge_stub_1",
        elem_b="rear_cheek_1",
        name="right hinge stub is carried by the rear bracket",
    )
    ctx.expect_gap(
        dial,
        base,
        axis="x",
        positive_elem="knob_shell",
        negative_elem="dial_bezel",
        min_gap=0.001,
        max_gap=0.004,
        name="dial stays visually separate from the housing",
    )
    ctx.expect_overlap(
        dial,
        base,
        axes="yz",
        elem_a="knob_shell",
        elem_b="dial_bezel",
        min_overlap=0.024,
        name="dial remains centered on the side bezel",
    )

    closed_plate = ctx.part_element_world_aabb(lid, elem="upper_plate")
    upper_limit = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if upper_limit is not None:
        with ctx.pose({lid_hinge: upper_limit}):
            ctx.expect_gap(
                lid,
                base,
                axis="z",
                positive_elem="upper_plate",
                negative_elem="lower_plate",
                min_gap=0.095,
                name="opened platen lifts well clear of the lower grill",
            )
            opened_plate = ctx.part_element_world_aabb(lid, elem="upper_plate")
        ctx.check(
            "upper platen rotates upward",
            closed_plate is not None
            and opened_plate is not None
            and float(opened_plate[1][2]) > float(closed_plate[1][2]) + 0.12,
            details=f"closed={closed_plate!r}, opened={opened_plate!r}",
        )

    dial_limits = dial_joint.motion_limits
    ctx.check(
        "dial uses a continuous joint",
        str(dial_joint.articulation_type).endswith("CONTINUOUS"),
        details=f"type={dial_joint.articulation_type!r}",
    )
    ctx.check(
        "dial has no angle stops",
        dial_limits is not None and dial_limits.lower is None and dial_limits.upper is None,
        details=f"limits={dial_limits!r}",
    )

    return ctx.report()


object_model = build_object_model()
