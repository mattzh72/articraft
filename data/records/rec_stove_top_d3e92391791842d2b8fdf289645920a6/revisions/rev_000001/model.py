from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


COOKTOP_WIDTH = 0.76
COOKTOP_DEPTH = 0.52
GLASS_THICKNESS = 0.006
BODY_TOP_Z = 0.044
TOP_SURFACE_Z = BODY_TOP_Z + GLASS_THICKNESS
CONTROL_STRIP_WIDTH = 0.74
CONTROL_STRIP_HEIGHT = 0.036
CONTROL_STRIP_THICKNESS = 0.003
CONTROL_STRIP_BOTTOM_Z = 0.008
CONTROL_STRIP_CENTER_Z = CONTROL_STRIP_BOTTOM_Z + CONTROL_STRIP_HEIGHT * 0.5
CONTROL_STRIP_OUTER_Y = -0.257
CONTROL_STRIP_CENTER_Y = CONTROL_STRIP_OUTER_Y + CONTROL_STRIP_THICKNESS * 0.5

KNOB_XS = (-0.24, -0.08, 0.08, 0.24)
BUTTON_XS = (0.312, 0.347)
CONTROL_Z = 0.024


def _circle_profile(radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _zone_ring_mesh(name: str, outer_diameter: float, ring_width: float, thickness: float):
    outer_radius = outer_diameter * 0.5
    inner_radius = max(outer_radius - ring_width, outer_radius * 0.4)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=64),
            [_circle_profile(inner_radius, segments=64)],
            thickness,
            center=True,
        ),
        name,
    )


def _control_strip_mesh():
    hole_profiles: list[list[tuple[float, float]]] = []
    spindle_radius = 0.0044

    for knob_x in KNOB_XS:
        hole_profiles.append(_offset_profile(_circle_profile(spindle_radius, segments=32), dx=knob_x))

    button_hole = rounded_rect_profile(0.010, 0.014, 0.002, corner_segments=4)
    for button_x in BUTTON_XS:
        hole_profiles.append(_offset_profile(button_hole, dx=button_x, dy=CONTROL_Z - CONTROL_STRIP_CENTER_Z))

    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(CONTROL_STRIP_WIDTH, CONTROL_STRIP_HEIGHT, 0.004, corner_segments=6),
            hole_profiles,
            CONTROL_STRIP_THICKNESS,
            center=True,
        ),
        "control_strip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceramic_electric_cooktop")

    glass_black = model.material("glass_black", rgba=(0.06, 0.06, 0.07, 1.0))
    chassis_dark = model.material("chassis_dark", rgba=(0.18, 0.18, 0.19, 1.0))
    strip_steel = model.material("strip_steel", rgba=(0.56, 0.58, 0.61, 1.0))
    zone_mark = model.material("zone_mark", rgba=(0.34, 0.29, 0.29, 1.0))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))
    button_metal = model.material("button_metal", rgba=(0.74, 0.75, 0.77, 1.0))
    button_dark = model.material("button_dark", rgba=(0.24, 0.25, 0.27, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.72, 0.44, BODY_TOP_Z)),
        origin=Origin(xyz=(0.0, 0.02, BODY_TOP_Z * 0.5)),
        material=chassis_dark,
        name="underside_pan",
    )
    body.visual(
        Box((0.72, 0.054, 0.012)),
        origin=Origin(xyz=(0.0, -0.227, 0.038)),
        material=chassis_dark,
        name="control_bridge_top",
    )
    body.visual(
        Box((0.72, 0.054, 0.010)),
        origin=Origin(xyz=(0.0, -0.227, 0.005)),
        material=chassis_dark,
        name="control_bridge_bottom",
    )
    body.visual(
        _control_strip_mesh(),
        origin=Origin(
            xyz=(0.0, CONTROL_STRIP_CENTER_Y, CONTROL_STRIP_CENTER_Z),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=strip_steel,
        name="control_strip",
    )
    body.visual(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, GLASS_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z + GLASS_THICKNESS * 0.5)),
        material=glass_black,
        name="glass_top",
    )

    zone_specs = (
        ("zone_0", (-0.165, 0.120), 0.180),
        ("zone_1", (0.185, 0.120), 0.155),
        ("zone_2", (-0.175, -0.090), 0.210),
        ("zone_3", (0.185, -0.095), 0.180),
    )
    for zone_name, (zone_x, zone_y), zone_diameter in zone_specs:
        zone = model.part(zone_name)
        zone.visual(
            _zone_ring_mesh(f"{zone_name}_ring", zone_diameter, 0.006, 0.0007),
            origin=Origin(xyz=(0.0, 0.0, 0.00035)),
            material=zone_mark,
            name="ring",
        )
        model.articulation(
            f"body_to_{zone_name}",
            ArticulationType.FIXED,
            parent=body,
            child=zone,
            origin=Origin(xyz=(zone_x, zone_y, TOP_SURFACE_Z)),
        )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.042,
            0.022,
            body_style="skirted",
            top_diameter=0.034,
            skirt=KnobSkirt(0.052, 0.006, flare=0.06),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
            bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            center=False,
        ),
        "cooktop_knob",
    )

    for index, knob_x in enumerate(KNOB_XS):
        knob = model.part(f"knob_{index}")
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=knob_black,
            name="knob_shell",
        )
        knob.visual(
            Cylinder(radius=0.0032, length=0.014),
            origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=button_dark,
            name="spindle",
        )
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, CONTROL_STRIP_OUTER_Y, CONTROL_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=8.0),
        )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.018, 0.004, 0.008)),
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
            material=button_metal,
            name="button_cap",
        )
        button.visual(
            Box((0.006, 0.003, 0.006)),
            origin=Origin(xyz=(0.0, -0.0015, 0.0)),
            material=button_dark,
            name="button_neck",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, CONTROL_STRIP_OUTER_Y, CONTROL_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0025,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    body_aabb = ctx.part_world_aabb(body)
    ctx.check("body_aabb_present", body_aabb is not None, "Expected a world AABB for the cooktop body.")
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check("cooktop_width_scale", 0.74 <= size[0] <= 0.78, f"size={size!r}")
        ctx.check("cooktop_depth_scale", 0.50 <= size[1] <= 0.54, f"size={size!r}")
        ctx.check("cooktop_height_scale", 0.045 <= size[2] <= 0.055, f"size={size!r}")

    for zone_index in range(4):
        zone = object_model.get_part(f"zone_{zone_index}")
        ctx.expect_gap(
            zone,
            body,
            axis="z",
            positive_elem="ring",
            negative_elem="glass_top",
            max_penetration=1e-5,
            max_gap=0.001,
            name=f"zone_{zone_index}_sits_on_glass",
        )

    for index in range(4):
        knob = object_model.get_part(f"knob_{index}")
        joint = object_model.get_articulation(f"body_to_knob_{index}")
        limits = joint.motion_limits
        axis = joint.axis
        ctx.check(
            f"knob_{index}_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"type={joint.articulation_type!r}",
        )
        ctx.check(
            f"knob_{index}_axis_front_facing",
            axis is not None and abs(axis[1]) > 0.99 and abs(axis[0]) < 1e-6 and abs(axis[2]) < 1e-6,
            f"axis={axis!r}",
        )
        ctx.check(
            f"knob_{index}_has_unbounded_rotation",
            limits is not None and limits.lower is None and limits.upper is None,
            f"limits={limits!r}",
        )
        ctx.expect_gap(
            body,
            knob,
            axis="y",
            positive_elem="control_strip",
            negative_elem="knob_shell",
            min_gap=0.0,
            max_gap=0.001,
            name=f"knob_{index}_seats_on_strip",
        )

    for index in range(2):
        button = object_model.get_part(f"button_{index}")
        joint = object_model.get_articulation(f"body_to_button_{index}")
        rest_pos = ctx.part_world_position(button)

        ctx.expect_gap(
            body,
            button,
            axis="y",
            positive_elem="control_strip",
            negative_elem="button_cap",
            min_gap=0.002,
            max_gap=0.0032,
            name=f"button_{index}_rests_proud_of_strip",
        )

        with ctx.pose({joint: 0.0025}):
            pressed_pos = ctx.part_world_position(button)
            ctx.expect_gap(
                body,
                button,
                axis="y",
                positive_elem="control_strip",
                negative_elem="button_cap",
                min_gap=0.0,
                max_gap=0.0006,
                name=f"button_{index}_presses_nearly_flush",
            )

        ctx.check(
            f"button_{index}_moves_inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[1] > rest_pos[1] + 0.002,
            f"rest={rest_pos!r}, pressed={pressed_pos!r}",
        )

    return ctx.report()


object_model = build_object_model()
