from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


SHOULDER_Z = 0.25
FIRST_LEN = 0.32
SECOND_LEN = 0.55
TERMINAL_LEN = 0.22


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    """Origin for a cylinder whose length runs across the arm width."""
    return Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))


def _add_link_body(
    part,
    *,
    length: float,
    bar_size: tuple[float, float, float],
    hub_radius: float,
    hub_width: float,
    material: str,
    hub_material: str,
    distal_fork: bool,
) -> None:
    """Add a clean flat linkage with a proximal hub and optional distal yoke."""
    part.visual(
        Cylinder(radius=hub_radius, length=hub_width),
        origin=_y_cylinder_origin(0.0, 0.0, 0.0),
        material=hub_material,
        name="proximal_hub",
    )

    bar_start = hub_radius * 0.70
    bar_end = length - (0.075 if distal_fork else 0.0)
    part.visual(
        Box((bar_end - bar_start, bar_size[1], bar_size[2])),
        origin=Origin(xyz=((bar_start + bar_end) / 2.0, 0.0, 0.0)),
        material=material,
        name="link_bar",
    )

    if distal_fork:
        cheek_x = length
        cheek_size = (0.13, 0.024, 0.120)
        for side, y in (("near", -0.055), ("far", 0.055)):
            part.visual(
                Box(cheek_size),
                origin=Origin(xyz=(cheek_x, y, 0.0)),
                material=hub_material,
                name=f"{side}_fork_cheek",
            )
        part.visual(
            Cylinder(radius=0.012, length=0.124),
            origin=_y_cylinder_origin(length, 0.0, 0.0),
            material=hub_material,
            name="distal_pin",
        )
        part.visual(
            Box((0.085, 0.130, bar_size[2])),
            origin=Origin(xyz=(length - 0.097, 0.0, 0.0)),
            material=material,
            name="fork_bridge",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="planar_calibration_arm")

    model.material("matte_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    model.material("satin_aluminum", rgba=(0.66, 0.68, 0.66, 1.0))
    model.material("black_bearing", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("tool_plate_gray", rgba=(0.44, 0.46, 0.47, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.18, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material="matte_graphite",
        name="base_disc",
    )
    pedestal.visual(
        Cylinder(radius=0.055, length=0.155),
        origin=Origin(xyz=(0.0, 0.0, 0.1125)),
        material="matte_graphite",
        name="short_post",
    )
    pedestal.visual(
        Box((0.10, 0.14, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z - 0.075)),
        material="matte_graphite",
        name="shoulder_bridge",
    )
    for side, y in (("near", -0.055), ("far", 0.055)):
        pedestal.visual(
            Box((0.090, 0.025, 0.128)),
            origin=Origin(xyz=(0.0, y, SHOULDER_Z)),
            material="matte_graphite",
            name=f"{side}_shoulder_cheek",
        )
        pedestal.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=_y_cylinder_origin(0.0, y * 1.24, SHOULDER_Z),
            material="black_bearing",
            name=f"{side}_shoulder_cap",
        )
    pedestal.visual(
        Cylinder(radius=0.014, length=0.132),
        origin=_y_cylinder_origin(0.0, 0.0, SHOULDER_Z),
        material="black_bearing",
        name="shoulder_pin",
    )

    first_link = model.part("first_link")
    _add_link_body(
        first_link,
        length=FIRST_LEN,
        bar_size=(0.0, 0.034, 0.034),
        hub_radius=0.052,
        hub_width=0.060,
        material="satin_aluminum",
        hub_material="black_bearing",
        distal_fork=True,
    )

    second_link = model.part("second_link")
    _add_link_body(
        second_link,
        length=SECOND_LEN,
        bar_size=(0.0, 0.032, 0.032),
        hub_radius=0.045,
        hub_width=0.052,
        material="satin_aluminum",
        hub_material="black_bearing",
        distal_fork=True,
    )

    terminal_link = model.part("terminal_link")
    terminal_link.visual(
        Cylinder(radius=0.038, length=0.050),
        origin=_y_cylinder_origin(0.0, 0.0, 0.0),
        material="black_bearing",
        name="wrist_hub",
    )
    terminal_link.visual(
        Box((TERMINAL_LEN - 0.030, 0.024, 0.024)),
        origin=Origin(xyz=((TERMINAL_LEN + 0.030) / 2.0, 0.0, 0.0)),
        material="satin_aluminum",
        name="slim_bar",
    )
    terminal_link.visual(
        Box((0.034, 0.095, 0.074)),
        origin=Origin(xyz=(TERMINAL_LEN - 0.017, 0.0, 0.0)),
        material="satin_aluminum",
        name="end_flange",
    )

    tool_plate = model.part("tool_plate")
    tool_plate.visual(
        Box((0.012, 0.18, 0.12)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material="tool_plate_gray",
        name="plate_face",
    )
    tool_plate.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.013, -0.055, 0.038), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_bearing",
        name="mount_boss_0",
    )
    tool_plate.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.013, 0.055, 0.038), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_bearing",
        name="mount_boss_1",
    )
    tool_plate.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.013, -0.055, -0.038), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_bearing",
        name="mount_boss_2",
    )
    tool_plate.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.013, 0.055, -0.038), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_bearing",
        name="mount_boss_3",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=first_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-1.05, upper=1.20),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(FIRST_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.4, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=terminal_link,
        origin=Origin(xyz=(SECOND_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.8, lower=-2.75, upper=2.75),
    )
    model.articulation(
        "plate_mount",
        ArticulationType.FIXED,
        parent=terminal_link,
        child=tool_plate,
        origin=Origin(xyz=(TERMINAL_LEN, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")
    plate_mount = object_model.get_articulation("plate_mount")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    terminal_link = object_model.get_part("terminal_link")
    tool_plate = object_model.get_part("tool_plate")

    ctx.allow_overlap(
        "pedestal",
        "first_link",
        elem_a="shoulder_pin",
        elem_b="proximal_hub",
        reason="The shoulder pin is intentionally captured inside the first-link bearing bore.",
    )
    ctx.allow_overlap(
        "first_link",
        "second_link",
        elem_a="distal_pin",
        elem_b="proximal_hub",
        reason="The elbow pin is intentionally captured inside the second-link bearing bore.",
    )
    ctx.allow_overlap(
        "second_link",
        "terminal_link",
        elem_a="distal_pin",
        elem_b="wrist_hub",
        reason="The wrist pin is intentionally captured inside the terminal-link bearing bore.",
    )

    serial_ok = (
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
        and wrist.articulation_type == ArticulationType.REVOLUTE
        and plate_mount.articulation_type == ArticulationType.FIXED
        and shoulder.child == "first_link"
        and elbow.parent == "first_link"
        and elbow.child == "second_link"
        and wrist.parent == "second_link"
        and wrist.child == "terminal_link"
    )
    ctx.check("shoulder elbow wrist form a serial revolute chain", serial_ok)

    same_plane_axes = all(
        tuple(round(v, 6) for v in joint.axis) == (0.0, 1.0, 0.0)
        for joint in (shoulder, elbow, wrist)
    )
    ctx.check("revolute axes are parallel for planar motion", same_plane_axes)

    ctx.expect_origin_gap(
        second_link,
        first_link,
        axis="x",
        min_gap=FIRST_LEN - 0.001,
        max_gap=FIRST_LEN + 0.001,
        name="short first link sets the elbow spacing",
    )
    ctx.expect_origin_gap(
        terminal_link,
        second_link,
        axis="x",
        min_gap=SECOND_LEN - 0.001,
        max_gap=SECOND_LEN + 0.001,
        name="second link is the longer span",
    )
    ctx.expect_origin_gap(
        tool_plate,
        terminal_link,
        axis="x",
        min_gap=TERMINAL_LEN - 0.001,
        max_gap=TERMINAL_LEN + 0.001,
        name="slim terminal link carries the tool plate",
    )
    ctx.check(
        "link proportions match calibration arm prompt",
        FIRST_LEN < SECOND_LEN and TERMINAL_LEN < FIRST_LEN,
        details=f"first={FIRST_LEN}, second={SECOND_LEN}, terminal={TERMINAL_LEN}",
    )

    ctx.expect_overlap(
        "pedestal",
        "first_link",
        axes="xyz",
        elem_a="shoulder_pin",
        elem_b="proximal_hub",
        min_overlap=0.020,
        name="shoulder pin passes through first-link hub",
    )
    ctx.expect_overlap(
        "first_link",
        "second_link",
        axes="xyz",
        elem_a="distal_pin",
        elem_b="proximal_hub",
        min_overlap=0.020,
        name="elbow pin passes through second-link hub",
    )
    ctx.expect_overlap(
        "second_link",
        "terminal_link",
        axes="xyz",
        elem_a="distal_pin",
        elem_b="wrist_hub",
        min_overlap=0.020,
        name="wrist pin passes through terminal-link hub",
    )

    ctx.expect_contact(
        terminal_link,
        tool_plate,
        elem_a="end_flange",
        elem_b="plate_face",
        contact_tol=1e-5,
        name="tool plate is seated on the terminal flange",
    )
    ctx.expect_overlap(
        terminal_link,
        tool_plate,
        axes="yz",
        elem_a="end_flange",
        elem_b="plate_face",
        min_overlap=0.070,
        name="tool plate face overlaps flange footprint",
    )

    with ctx.pose({shoulder: 0.45, elbow: -0.65, wrist: 0.35}):
        positions = [
            ctx.part_world_position(part)
            for part in (first_link, second_link, terminal_link, tool_plate)
        ]
        ctx.check(
            "posed chain remains in the same vertical plane",
            all(pos is not None and abs(pos[1]) < 1e-6 for pos in positions),
            details=f"positions={positions}",
        )

    return ctx.report()


object_model = build_object_model()
