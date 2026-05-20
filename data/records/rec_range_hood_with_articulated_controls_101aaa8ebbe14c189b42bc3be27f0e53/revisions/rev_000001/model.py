from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
)


CANOPY_WIDTH = 0.90
CANOPY_TOP = 0.46
CHIMNEY_HEIGHT = 0.34
GLASS_WIDTH = 0.82
GLASS_HEIGHT = 0.19
GLASS_CLOSED_ROLL = 0.02
TOP_RAIL_DEPTH = 0.020
TOP_RAIL_HEIGHT = 0.024


def _build_canopy_shell() -> object:
    outer_profile = [
        (-0.18, 0.10),
        (-0.18, CANOPY_TOP),
        (0.12, CANOPY_TOP),
        (0.23, 0.15),
        (0.23, 0.10),
        (-0.02, 0.10),
    ]
    outer = cq.Workplane("YZ").polyline(outer_profile).close().extrude(CANOPY_WIDTH / 2.0, both=True)

    cavity = (
        cq.Workplane("XY")
        .box(CANOPY_WIDTH - 0.06, 0.37, 0.40)
        .translate((0.0, 0.025, 0.225))
    )
    return outer.cut(cavity)


def _rolled_center(distance_from_hinge: float, roll: float) -> tuple[float, float, float]:
    return (0.0, distance_from_hinge * math.sin(roll), -distance_from_hinge * math.cos(roll))


def _rolled_yz_half_extents(depth: float, height: float, roll: float) -> tuple[float, float]:
    half_depth = depth * 0.5
    half_height = height * 0.5
    return (
        half_depth * math.cos(roll) + half_height * math.sin(roll),
        half_height * math.cos(roll) + half_depth * math.sin(roll),
    )


def _aabb_size(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(float(maxs[i] - mins[i]) for i in range(3))


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple(float((mins[i] + maxs[i]) * 0.5) for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="angled_range_hood")

    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.36, 0.39, 0.42, 1.0))
    black = model.material("black", rgba=(0.12, 0.12, 0.13, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.45, 0.52, 0.56, 0.35))
    hinge_center_y = 0.252
    hinge_center_z = 0.164
    top_rail_center = _rolled_center(0.012, GLASS_CLOSED_ROLL)
    top_rail_half_y, top_rail_half_z = _rolled_yz_half_extents(
        TOP_RAIL_DEPTH,
        TOP_RAIL_HEIGHT,
        GLASS_CLOSED_ROLL,
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_canopy_shell(), "range_hood_canopy"),
        material=steel,
        name="canopy_shell",
    )
    body.visual(
        Box((0.28, 0.16, CHIMNEY_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.10, CANOPY_TOP + CHIMNEY_HEIGHT * 0.5)),
        material=steel,
        name="chimney",
    )
    body.visual(
        Box((0.84, 0.032, 0.024)),
        origin=Origin(xyz=(0.0, 0.205, 0.150)),
        material=dark_steel,
        name="front_rail",
    )
    body.visual(
        Box((0.72, 0.024, TOP_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                hinge_center_y + top_rail_center[1] - top_rail_half_y - 0.012,
                hinge_center_z + top_rail_center[2],
            )
        ),
        material=dark_steel,
        name="hinge_strip",
    )
    body.visual(
        Box((0.018, 0.140, 0.120)),
        origin=Origin(xyz=(CANOPY_WIDTH * 0.5 - 0.009, 0.090, 0.225)),
        material=black,
        name="control_bezel",
    )
    for index, x_pos in enumerate((-0.27, 0.27)):
        body.visual(
            Cylinder(radius=0.0085, length=0.06),
            origin=Origin(xyz=(x_pos, 0.194, 0.150), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"hinge_mount_{index}",
        )

    deflector = model.part("deflector")
    deflector.visual(
        Box((GLASS_WIDTH, TOP_RAIL_DEPTH, TOP_RAIL_HEIGHT)),
        origin=Origin(
            xyz=_rolled_center(0.012, GLASS_CLOSED_ROLL),
            rpy=(GLASS_CLOSED_ROLL, 0.0, 0.0),
        ),
        material=dark_steel,
        name="top_rail",
    )
    deflector.visual(
        Box((GLASS_WIDTH - 0.02, 0.005, GLASS_HEIGHT)),
        origin=Origin(
            xyz=_rolled_center(0.114, GLASS_CLOSED_ROLL),
            rpy=(GLASS_CLOSED_ROLL, 0.0, 0.0),
        ),
        material=glass_tint,
        name="glass_panel",
    )
    for index, x_pos in enumerate((-GLASS_WIDTH * 0.5 + 0.010, GLASS_WIDTH * 0.5 - 0.010)):
        deflector.visual(
            Box((0.016, 0.018, GLASS_HEIGHT - 0.018)),
            origin=Origin(
                xyz=(
                    x_pos,
                    _rolled_center(0.108, GLASS_CLOSED_ROLL)[1],
                    _rolled_center(0.108, GLASS_CLOSED_ROLL)[2],
                ),
                rpy=(GLASS_CLOSED_ROLL, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"side_trim_{index}",
        )

    model.articulation(
        "body_to_deflector",
        ArticulationType.REVOLUTE,
        parent=body,
        child=deflector,
        origin=Origin(xyz=(0.0, hinge_center_y, hinge_center_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=0.0, upper=0.31),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.020,
                body_style="tapered",
                top_diameter=0.032,
                skirt=KnobSkirt(0.046, 0.004, flare=0.04),
                grip=KnobGrip(style="fluted", count=14, depth=0.0011),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "range_hood_dial",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="dial_cap",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(CANOPY_WIDTH * 0.5, 0.090, 0.258)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0),
    )

    button_positions = (0.206, 0.170)
    for index, z_pos in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.012, 0.026, 0.012)),
            origin=Origin(xyz=(0.006, 0.0, 0.0)),
            material=dark_steel,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(CANOPY_WIDTH * 0.5, 0.090, z_pos)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    deflector = object_model.get_part("deflector")
    dial = object_model.get_part("dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    deflector_joint = object_model.get_articulation("body_to_deflector")
    dial_joint = object_model.get_articulation("body_to_dial")
    button_0_joint = object_model.get_articulation("body_to_button_0")
    button_1_joint = object_model.get_articulation("body_to_button_1")

    body_size = _aabb_size(ctx.part_world_aabb(body))
    ctx.check("hood_body_size_present", body_size is not None, "Expected body world AABB.")
    if body_size is not None:
        ctx.check("hood_width_realistic", 0.86 <= body_size[0] <= 0.94, f"size={body_size!r}")
        ctx.check("hood_depth_realistic", 0.39 <= body_size[1] <= 0.48, f"size={body_size!r}")
        ctx.check("hood_height_realistic", 0.68 <= body_size[2] <= 0.76, f"size={body_size!r}")

    with ctx.pose({deflector_joint: 0.0}):
        ctx.expect_overlap(
            deflector,
            body,
            axes="x",
            min_overlap=0.78,
            name="deflector spans the hood width",
        )
        ctx.expect_contact(
            deflector,
            body,
            elem_a="top_rail",
            elem_b="hinge_strip",
            name="deflector is carried by the hinge strip",
        )
        closed_panel_aabb = ctx.part_element_world_aabb(deflector, elem="glass_panel")

    upper = deflector_joint.motion_limits.upper if deflector_joint.motion_limits is not None else None
    with ctx.pose({deflector_joint: 0.70 if upper is None else min(0.70, upper)}):
        raised_panel_aabb = ctx.part_element_world_aabb(deflector, elem="glass_panel")

    closed_center = _aabb_center(closed_panel_aabb)
    raised_center = _aabb_center(raised_panel_aabb)
    closed_low_z = None if closed_panel_aabb is None else float(closed_panel_aabb[0][2])
    raised_low_z = None if raised_panel_aabb is None else float(raised_panel_aabb[0][2])
    ctx.check(
        "deflector_raises_upward",
        closed_center is not None
        and raised_center is not None
        and closed_low_z is not None
        and raised_low_z is not None
        and raised_low_z > closed_low_z + 0.006
        and raised_center[1] < closed_center[1] - 0.03,
        details=(
            f"closed_center={closed_center!r}, raised_center={raised_center!r}, "
            f"closed_low_z={closed_low_z!r}, raised_low_z={raised_low_z!r}"
        ),
    )

    ctx.check(
        "dial_is_continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=f"type={dial_joint.articulation_type!r}, limits={dial_joint.motion_limits!r}",
    )
    ctx.expect_origin_gap(
        dial,
        body,
        axis="x",
        min_gap=0.44,
        max_gap=0.46,
        name="dial sits on the side of the hood",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_0_joint: 0.004}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_steady = ctx.part_world_position(button_1)
    with ctx.pose({button_1_joint: 0.004}):
        button_0_steady = ctx.part_world_position(button_0)
        button_1_pressed = ctx.part_world_position(button_1)

    ctx.check(
        "button_0_depresses_independently",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_1_rest is not None
        and button_1_steady is not None
        and button_0_pressed[0] < button_0_rest[0] - 0.003
        and abs(button_1_steady[0] - button_1_rest[0]) < 1e-6,
        details=(
            f"button_0_rest={button_0_rest!r}, button_0_pressed={button_0_pressed!r}, "
            f"button_1_rest={button_1_rest!r}, button_1_steady={button_1_steady!r}"
        ),
    )
    ctx.check(
        "button_1_depresses_independently",
        button_1_rest is not None
        and button_1_pressed is not None
        and button_0_rest is not None
        and button_0_steady is not None
        and button_1_pressed[0] < button_1_rest[0] - 0.003
        and abs(button_0_steady[0] - button_0_rest[0]) < 1e-6,
        details=(
            f"button_1_rest={button_1_rest!r}, button_1_pressed={button_1_pressed!r}, "
            f"button_0_rest={button_0_rest!r}, button_0_steady={button_0_steady!r}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
