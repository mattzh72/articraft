from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_laptop")

    graphite = Material("dark graphite magnesium", rgba=(0.055, 0.060, 0.062, 1.0))
    rubber = Material("matte black rubber", rgba=(0.010, 0.011, 0.012, 1.0))
    key_mat = Material("charcoal key plastic", rgba=(0.018, 0.020, 0.023, 1.0))
    key_edge = Material("key edge bevel", rgba=(0.045, 0.047, 0.052, 1.0))
    glass = Material("dark smoked display glass", rgba=(0.005, 0.010, 0.015, 1.0))
    blue_glow = Material("dim blue screen sheen", rgba=(0.020, 0.065, 0.105, 1.0))
    pad_mat = Material("slate clickpad surface", rgba=(0.075, 0.080, 0.085, 1.0))
    metal = Material("dark hinge metal", rgba=(0.095, 0.090, 0.080, 1.0))

    base = model.part("base")

    chassis_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.382, 0.268, 0.018, corner_segments=8),
            0.045,
            cap=True,
            center=True,
        ),
        "rounded_lower_chassis",
    )
    base.visual(
        chassis_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=graphite,
        name="lower_chassis",
    )

    # Rugged overmolded corner armor and perimeter lips overlap the chassis
    # slightly, as real rubber armor wraps around a magnesium shell.
    for ix, x in enumerate((-0.156, 0.156)):
        for iy, y in enumerate((-0.103, 0.103)):
            base.visual(
                Box((0.078, 0.058, 0.020)),
                origin=Origin(xyz=(x, y, 0.049)),
                material=rubber,
                name=f"corner_bumper_{ix}_{iy}",
            )
    base.visual(
        Box((0.320, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.126, 0.050)),
        material=rubber,
        name="front_rubber_lip",
    )
    base.visual(
        Box((0.318, 0.034, 0.036)),
        origin=Origin(xyz=(0.0, 0.130, 0.054)),
        material=rubber,
        name="rear_spine",
    )
    base.visual(
        Box((0.030, 0.175, 0.015)),
        origin=Origin(xyz=(-0.184, 0.0, 0.047)),
        material=rubber,
        name="side_rail_0",
    )
    base.visual(
        Box((0.030, 0.175, 0.015)),
        origin=Origin(xyz=(0.184, 0.0, 0.047)),
        material=rubber,
        name="side_rail_1",
    )

    base.visual(
        Box((0.310, 0.132, 0.003)),
        origin=Origin(xyz=(0.0, 0.026, 0.0465)),
        material=rubber,
        name="keyboard_well",
    )
    base.visual(
        Box((0.132, 0.080, 0.003)),
        origin=Origin(xyz=(0.0, -0.077, 0.0465)),
        material=rubber,
        name="clickpad_well",
    )
    base.visual(
        Box((0.018, 0.018, 0.004)),
        origin=Origin(xyz=(0.155, -0.045, 0.047)),
        material=metal,
        name="fingerprint_button",
    )

    # A pair of exposed barrel covers on the rear spine marks the real hinge
    # line; the revolute display joint below uses the same axis.
    for i, x in enumerate((-0.108, 0.108)):
        base.visual(
            Cylinder(radius=0.012, length=0.070),
            origin=Origin(xyz=(x, 0.149, 0.067), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal,
            name=f"hinge_barrel_{i}",
        )

    # Individual keycaps are separate prismatic slider parts.  A compact
    # six-row rugged keyboard gives enough visible keys to read as a laptop
    # keyboard while keeping the mechanical model tractable.
    key_specs: list[tuple[str, float, float, float, float]] = []
    rows = [
        (8, -0.098, 0.082, 0.023, 0.014),
        (10, -0.126, 0.061, 0.023, 0.016),
        (9, -0.113, 0.038, 0.024, 0.016),
        (9, -0.113, 0.015, 0.024, 0.016),
        (8, -0.099, -0.008, 0.025, 0.016),
    ]
    for row_index, (count, start_x, y, sx, sy) in enumerate(rows):
        for col in range(count):
            x = start_x + col * 0.028
            key_specs.append((f"key_{row_index}_{col}", x, y, sx, sy))
    key_specs.extend(
        [
            ("key_space", 0.0, -0.030, 0.118, 0.017),
            ("key_fn_0", -0.145, -0.030, 0.026, 0.017),
            ("key_fn_1", 0.145, -0.030, 0.026, 0.017),
        ]
    )

    for key_name, x, y, sx, sy in key_specs:
        key = model.part(key_name)
        key.visual(
            Box((sx, sy, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=key_mat,
            name="cap",
        )
        key.visual(
            Box((sx * 0.72, sy * 0.54, 0.0012)),
            origin=Origin(xyz=(0.0, 0.0, 0.0066)),
            material=key_edge,
            name="top_land",
        )
        model.articulation(
            f"base_to_{key_name}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=key,
            origin=Origin(xyz=(x, y, 0.048)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.12, lower=-0.0035, upper=0.0),
        )

    clickpad = model.part("clickpad")
    clickpad.visual(
        Box((0.118, 0.072, 0.004)),
        origin=Origin(xyz=(0.0, 0.036, 0.002)),
        material=pad_mat,
        name="pad_panel",
    )
    clickpad.visual(
        Box((0.112, 0.0025, 0.001)),
        origin=Origin(xyz=(0.0, 0.071, 0.0045)),
        material=rubber,
        name="rear_click_line",
    )
    model.articulation(
        "base_to_clickpad",
        ArticulationType.REVOLUTE,
        parent=base,
        child=clickpad,
        # The child frame lies on the shallow pivot at the clickpad's front edge.
        origin=Origin(xyz=(0.0, -0.113, 0.048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.7, lower=-0.055, upper=0.010),
    )

    display = model.part("display")
    lid_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.366, 0.238, 0.017, corner_segments=8),
            0.026,
            cap=True,
            center=True,
        ),
        "reinforced_display_housing",
    )
    display.visual(
        lid_mesh,
        # Mesh local X/Y are display width/height; rotating +90 deg about X
        # maps mesh local Y to world/part Z and mesh depth to lid thickness.
        origin=Origin(xyz=(0.0, 0.0, 0.133), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="housing",
    )
    display.visual(
        Box((0.305, 0.004, 0.178)),
        origin=Origin(xyz=(0.0, -0.015, 0.137)),
        material=glass,
        name="screen_glass",
    )
    display.visual(
        Box((0.284, 0.0015, 0.155)),
        origin=Origin(xyz=(0.0, -0.01775, 0.139)),
        material=blue_glow,
        name="screen_sheen",
    )
    display.visual(
        Box((0.342, 0.009, 0.024)),
        origin=Origin(xyz=(0.0, -0.019, 0.239)),
        material=rubber,
        name="top_bezel",
    )
    display.visual(
        Box((0.342, 0.009, 0.028)),
        origin=Origin(xyz=(0.0, -0.019, 0.036)),
        material=rubber,
        name="bottom_bezel",
    )
    display.visual(
        Box((0.026, 0.009, 0.205)),
        origin=Origin(xyz=(-0.171, -0.019, 0.138)),
        material=rubber,
        name="side_bezel_0",
    )
    display.visual(
        Box((0.026, 0.009, 0.205)),
        origin=Origin(xyz=(0.171, -0.019, 0.138)),
        material=rubber,
        name="side_bezel_1",
    )
    for i, x in enumerate((-0.108, 0.108)):
        display.visual(
            Box((0.066, 0.034, 0.024)),
            origin=Origin(xyz=(x, -0.002, 0.024)),
            material=metal,
            name=f"hinge_leaf_{i}",
        )

    model.articulation(
        "base_to_display",
        ArticulationType.REVOLUTE,
        parent=base,
        child=display,
        origin=Origin(xyz=(0.0, 0.149, 0.067)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=1.6, lower=-0.42, upper=1.62),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    display = object_model.get_part("display")
    clickpad = object_model.get_part("clickpad")
    sample_key = object_model.get_part("key_2_4")
    display_hinge = object_model.get_articulation("base_to_display")
    key_slide = object_model.get_articulation("base_to_key_2_4")
    pad_pivot = object_model.get_articulation("base_to_clickpad")

    ctx.expect_gap(
        sample_key,
        base,
        axis="z",
        positive_elem="cap",
        negative_elem="keyboard_well",
        min_gap=0.0,
        max_gap=0.0005,
        name="keycap sits just above recessed keyboard well",
    )
    ctx.expect_gap(
        clickpad,
        base,
        axis="z",
        positive_elem="pad_panel",
        negative_elem="clickpad_well",
        min_gap=0.0,
        max_gap=0.0005,
        name="clickpad panel sits proud of its recess",
    )
    ctx.expect_overlap(
        display,
        base,
        axes="x",
        elem_a="housing",
        elem_b="rear_spine",
        min_overlap=0.28,
        name="display housing spans the rear spine",
    )

    key_rest = ctx.part_world_position(sample_key)
    with ctx.pose({key_slide: -0.003}):
        key_down = ctx.part_world_position(sample_key)
    ctx.check(
        "representative key slides downward",
        key_rest is not None and key_down is not None and key_down[2] < key_rest[2] - 0.0025,
        details=f"rest={key_rest}, down={key_down}",
    )

    pad_rest = ctx.part_element_world_aabb(clickpad, elem="pad_panel")
    with ctx.pose({pad_pivot: -0.045}):
        pad_clicked = ctx.part_element_world_aabb(clickpad, elem="pad_panel")
    ctx.check(
        "clickpad rocks downward about front edge",
        pad_rest is not None
        and pad_clicked is not None
        and pad_clicked[0][2] < pad_rest[0][2] - 0.001,
        details=f"rest={pad_rest}, clicked={pad_clicked}",
    )

    screen_rest = ctx.part_element_world_aabb(display, elem="screen_glass")
    with ctx.pose({display_hinge: 1.0}):
        screen_forward = ctx.part_element_world_aabb(display, elem="screen_glass")
    ctx.check(
        "display lid swings forward on rear axis",
        screen_rest is not None
        and screen_forward is not None
        and screen_forward[0][1] < screen_rest[0][1] - 0.08,
        details=f"rest={screen_rest}, forward={screen_forward}",
    )

    return ctx.report()


object_model = build_object_model()
