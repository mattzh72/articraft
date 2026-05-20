from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
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
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 48,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _annulus_mesh(
    outer_radius: float,
    inner_radius: float,
    height: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=64),
            [_circle_profile(inner_radius, segments=64)],
            height,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_induction_cooktop")

    counter_mat = model.material("warm_white_counter", rgba=(0.82, 0.80, 0.74, 1.0))
    cabinet_mat = model.material("plain_white_cabinet", rgba=(0.90, 0.88, 0.82, 1.0))
    shadow_mat = model.material("cabinet_shadow", rgba=(0.10, 0.10, 0.09, 1.0))
    support_mat = model.material("black_support_rail", rgba=(0.02, 0.02, 0.02, 1.0))
    glass_mat = model.material("smoked_black_glass", rgba=(0.015, 0.018, 0.022, 0.88))
    glass_edge_mat = model.material("black_enamel_body", rgba=(0.01, 0.01, 0.012, 1.0))
    zone_mat = model.material("subtle_blue_zone_print", rgba=(0.20, 0.42, 0.54, 0.74))
    tick_mat = model.material("pale_control_ticks", rgba=(0.78, 0.86, 0.88, 0.90))
    collar_mat = model.material("brushed_steel_trim", rgba=(0.58, 0.57, 0.52, 1.0))
    button_mat = model.material("satin_button_caps", rgba=(0.78, 0.77, 0.72, 1.0))
    dark_button_mat = model.material("button_dark_inlay", rgba=(0.03, 0.03, 0.032, 1.0))
    knob_mat = model.material("charcoal_knob", rgba=(0.045, 0.045, 0.048, 1.0))
    indicator_mat = model.material("white_knob_indicator", rgba=(0.92, 0.92, 0.86, 1.0))

    counter = model.part("cabinet")

    # Thin counter slab split into a connected ring around the cooktop cutout.
    counter_thickness = 0.045
    counter_z = -counter_thickness / 2.0
    counter.visual(
        Box((1.08, 0.085, counter_thickness)),
        origin=Origin(xyz=(0.0, 0.3175, counter_z)),
        material=counter_mat,
        name="rear_counter_strip",
    )
    counter.visual(
        Box((1.08, 0.085, counter_thickness)),
        origin=Origin(xyz=(0.0, -0.3175, counter_z)),
        material=counter_mat,
        name="front_counter_strip",
    )
    counter.visual(
        Box((0.095, 0.55, counter_thickness)),
        origin=Origin(xyz=(-0.4925, 0.0, counter_z)),
        material=counter_mat,
        name="side_counter_strip_0",
    )
    counter.visual(
        Box((0.095, 0.55, counter_thickness)),
        origin=Origin(xyz=(0.4925, 0.0, counter_z)),
        material=counter_mat,
        name="side_counter_strip_1",
    )

    # Plain open-front cabinet underneath the counter.
    counter.visual(
        Box((0.035, 0.52, 0.70)),
        origin=Origin(xyz=(-0.475, 0.02, -0.395)),
        material=cabinet_mat,
        name="cabinet_side_0",
    )
    counter.visual(
        Box((0.035, 0.52, 0.70)),
        origin=Origin(xyz=(0.475, 0.02, -0.395)),
        material=cabinet_mat,
        name="cabinet_side_1",
    )
    counter.visual(
        Box((0.95, 0.035, 0.70)),
        origin=Origin(xyz=(0.0, 0.2975, -0.395)),
        material=cabinet_mat,
        name="cabinet_back",
    )
    counter.visual(
        Box((0.95, 0.52, 0.035)),
        origin=Origin(xyz=(0.0, 0.02, -0.7275)),
        material=cabinet_mat,
        name="bottom_shelf",
    )
    counter.visual(
        Box((0.86, 0.012, 0.50)),
        origin=Origin(xyz=(0.0, 0.276, -0.425)),
        material=shadow_mat,
        name="dark_opening_back",
    )

    # Small ledges inside the counter cutout carry the flush glass panel.
    for name, x, y, size in (
        ("left_ledge", -0.4325, 0.0, (0.025, 0.52, 0.006)),
        ("right_ledge", 0.4325, 0.0, (0.025, 0.52, 0.006)),
        ("front_ledge", 0.0, -0.2625, (0.86, 0.025, 0.006)),
        ("rear_ledge", 0.0, 0.2625, (0.86, 0.025, 0.006)),
    ):
        counter.visual(
            Box(size),
            origin=Origin(xyz=(x, y, -0.015)),
            material=support_mat,
            name=name,
        )

    cooktop = model.part("cooktop")
    glass_thickness = 0.012
    button_positions = [
        (-0.30, -0.205),
        (-0.22, -0.205),
        (0.22, -0.205),
        (0.30, -0.205),
    ]
    knob_position = (0.0, -0.205)
    glass_holes = [
        _circle_profile(0.023, center=pos, segments=48) for pos in button_positions
    ] + [_circle_profile(0.018, center=knob_position, segments=64)]
    cooktop.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.86, 0.52, 0.026, corner_segments=8),
                glass_holes,
                glass_thickness,
                center=True,
            ),
            "glass_panel",
        ),
        origin=Origin(xyz=(0.0, 0.0, -glass_thickness / 2.0)),
        material=glass_mat,
        name="glass_panel",
    )
    cooktop.visual(
        Box((0.78, 0.30, 0.032)),
        origin=Origin(xyz=(0.0, 0.05, -0.028)),
        material=glass_edge_mat,
        name="shallow_underbody",
    )

    # Five printed induction zones on the glass surface.
    zone_specs = [
        ("zone_0", -0.245, 0.125, 0.088),
        ("zone_1", -0.265, -0.045, 0.072),
        ("zone_2", 0.000, 0.155, 0.080),
        ("zone_3", 0.250, 0.115, 0.092),
        ("zone_4", 0.255, -0.055, 0.070),
    ]
    for name, x, y, radius in zone_specs:
        cooktop.visual(
            _annulus_mesh(radius, radius - 0.006, 0.0005, name),
            origin=Origin(xyz=(x, y, 0.00005)),
            material=zone_mat,
            name=name,
        )
        cooktop.visual(
            Cylinder(radius=0.010, length=0.0005),
            origin=Origin(xyz=(x, y, 0.00005)),
            material=zone_mat,
            name=f"{name}_center",
        )

    # Metal collars around the four push buttons, plus scale marks around the knob.
    for i, pos in enumerate(button_positions):
        cooktop.visual(
            _annulus_mesh(0.030, 0.023, 0.0020, f"button_collar_{i}"),
            origin=Origin(xyz=(pos[0], pos[1], 0.0010)),
            material=collar_mat,
            name=f"button_collar_{i}",
        )

    for i in range(12):
        angle = 2.0 * math.pi * i / 12.0
        r = 0.058
        tick_length = 0.014 if i % 3 == 0 else 0.010
        cooktop.visual(
            Box((0.0032, tick_length, 0.0007)),
            origin=Origin(
                xyz=(r * math.sin(angle), knob_position[1] + r * math.cos(angle), 0.0001),
                rpy=(0.0, 0.0, -angle),
            ),
            material=tick_mat,
            name=f"knob_tick_{i}",
        )

    model.articulation(
        "cabinet_to_cooktop",
        ArticulationType.FIXED,
        parent=counter,
        child=cooktop,
        origin=Origin(),
    )

    knob = model.part("knob")
    knob_body = KnobGeometry(
        0.076,
        0.027,
        body_style="skirted",
        top_diameter=0.060,
        skirt=KnobSkirt(0.086, 0.005, flare=0.05, chamfer=0.0012),
        grip=KnobGrip(style="fluted", count=24, depth=0.0010),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(knob_body, "rotary_knob_body"),
        origin=Origin(),
        material=knob_mat,
        name="body",
    )
    knob.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=knob_mat,
        name="shaft",
    )
    knob.visual(
        Box((0.006, 0.030, 0.0012)),
        origin=Origin(xyz=(0.0, 0.014, 0.0276)),
        material=indicator_mat,
        name="indicator",
    )
    model.articulation(
        "cooktop_to_knob",
        ArticulationType.CONTINUOUS,
        parent=cooktop,
        child=knob,
        origin=Origin(xyz=(knob_position[0], knob_position[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )

    button_names = ["left_button_0", "left_button_1", "right_button_0", "right_button_1"]
    for name, pos in zip(button_names, button_positions):
        button = model.part(name)
        button.visual(
            Cylinder(radius=0.0205, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=button_mat,
            name="cap",
        )
        button.visual(
            Cylinder(radius=0.010, length=0.001),
            origin=Origin(xyz=(0.0, 0.0, 0.0085)),
            material=dark_button_mat,
            name="top_inlay",
        )
        button.visual(
            Cylinder(radius=0.011, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.007)),
            material=dark_button_mat,
            name="plunger",
        )
        model.articulation(
            f"cooktop_to_{name}",
            ArticulationType.PRISMATIC,
            parent=cooktop,
            child=button,
            origin=Origin(xyz=(pos[0], pos[1], 0.0)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    cooktop = object_model.get_part("cooktop")
    knob = object_model.get_part("knob")
    knob_joint = object_model.get_articulation("cooktop_to_knob")

    button_names = ("left_button_0", "left_button_1", "right_button_0", "right_button_1")
    button_joints = [
        object_model.get_articulation(f"cooktop_to_{name}") for name in button_names
    ]

    ctx.allow_overlap(
        cooktop,
        knob,
        elem_a="glass_panel",
        elem_b="shaft",
        reason=(
            "The rotary knob shaft intentionally passes through the glass "
            "control aperture and is locally captured below the panel."
        ),
    )
    for name in button_names:
        ctx.allow_overlap(
            cooktop,
            name,
            elem_a="glass_panel",
            elem_b="plunger",
            reason=(
                "The push-button plunger intentionally passes through its "
                "glass control aperture for inward travel."
            ),
        )

    ctx.check(
        "only knob and four buttons articulate",
        len(button_joints) == 4
        and knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in button_joints),
        details="Expected one continuous rotary knob and four prismatic push buttons.",
    )

    counter_top = ctx.part_element_world_aabb(cabinet, elem="front_counter_strip")
    glass_panel = ctx.part_element_world_aabb(cooktop, elem="glass_panel")
    ctx.check(
        "glass sits flush with counter top",
        counter_top is not None
        and glass_panel is not None
        and abs(counter_top[1][2] - glass_panel[1][2]) <= 0.001,
        details=f"counter={counter_top}, glass={glass_panel}",
    )
    ctx.expect_contact(
        cooktop,
        cabinet,
        elem_a="glass_panel",
        elem_b="left_ledge",
        contact_tol=0.001,
        name="glass rests on side support ledge",
    )

    ctx.expect_contact(
        knob,
        cooktop,
        elem_a="body",
        elem_b="glass_panel",
        contact_tol=0.0015,
        name="rotary knob is seated on glass",
    )
    ctx.expect_overlap(
        knob,
        cooktop,
        axes="z",
        elem_a="shaft",
        elem_b="glass_panel",
        min_overlap=0.006,
        name="knob shaft remains captured through glass",
    )

    indicator_rest = ctx.part_element_world_aabb(knob, elem="indicator")
    with ctx.pose({knob_joint: math.pi / 2.0}):
        indicator_rotated = ctx.part_element_world_aabb(knob, elem="indicator")
    ctx.check(
        "knob indicator follows continuous rotation",
        indicator_rest is not None
        and indicator_rotated is not None
        and (indicator_rotated[1][0] - indicator_rotated[0][0])
        > (indicator_rest[1][0] - indicator_rest[0][0]) + 0.015,
        details=f"rest={indicator_rest}, rotated={indicator_rotated}",
    )

    for name, joint in zip(button_names, button_joints):
        button = object_model.get_part(name)
        ctx.expect_gap(
            button,
            cooktop,
            axis="z",
            positive_elem="cap",
            negative_elem="glass_panel",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{name} cap starts at glass surface",
        )
        ctx.expect_overlap(
            button,
            cooktop,
            axes="z",
            elem_a="plunger",
            elem_b="glass_panel",
            min_overlap=0.006,
            name=f"{name} plunger remains captured through glass",
        )
        rest_position = ctx.part_world_position(button)
        with ctx.pose({joint: 0.004}):
            pressed_position = ctx.part_world_position(button)
        ctx.check(
            f"{name} moves inward",
            rest_position is not None
            and pressed_position is not None
            and pressed_position[2] < rest_position[2] - 0.003,
            details=f"rest={rest_position}, pressed={pressed_position}",
        )

    return ctx.report()


object_model = build_object_model()
