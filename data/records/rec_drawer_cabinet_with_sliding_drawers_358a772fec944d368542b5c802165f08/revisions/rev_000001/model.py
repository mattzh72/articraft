from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WIDTH = 0.92
DEPTH = 0.54
HEIGHT = 0.80
WALL = 0.028
FRONT_THICKNESS = 0.022
FRONT_GAP = 0.002
FRONT_Y = -DEPTH / 2.0 - FRONT_THICKNESS / 2.0 - FRONT_GAP


def _front_panel_mesh(width: float, height: float, name: str):
    """Powder-coated drawer front with a shallow recessed pull pocket."""
    recess_w = width * 0.72
    recess_h = min(0.042, height * 0.40)
    recess_depth = 0.008
    handle_z = 0.0 if height < 0.13 else height * 0.18

    panel = cq.Workplane("XY").box(width, FRONT_THICKNESS, height)
    pocket = (
        cq.Workplane("XY")
        .box(recess_w, recess_depth + 0.004, recess_h)
        .translate((0.0, -FRONT_THICKNESS / 2.0 + recess_depth / 2.0 - 0.001, handle_z))
    )
    panel = panel.cut(pocket)

    # Small edge radii keep the steel fronts from reading as raw placeholder boxes.
    try:
        panel = panel.edges("|Y").fillet(0.002)
    except Exception:
        pass
    return mesh_from_cadquery(panel, name, tolerance=0.0008, angular_tolerance=0.08)


def _add_box(part, name: str, size, xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_rail_bearings(body, x: float, z: float, material, prefix: str) -> None:
    """A few partially-captured bright balls on each fixed runner."""
    sign = 1.0 if x >= 0.0 else -1.0
    for idx, y in enumerate((-0.18, -0.055, 0.070)):
        body.visual(
            Sphere(0.0042),
            origin=Origin(xyz=(x - sign * 0.0065, y, z + 0.010)),
            material=material,
            name=f"{prefix}_bearing_{idx}",
        )


def _drawer_specs():
    """Return drawer layout from low to high."""
    usable_w = 0.84
    gap = 0.012
    shallow_h = 0.105
    deep_h = 0.160
    bottom = 0.070
    col_w = (usable_w - gap) / 2.0
    col_x = col_w / 2.0 + gap / 2.0

    specs = []
    z = bottom + deep_h / 2.0
    for row in range(2):
        specs.append(
            {
                "name": f"deep_drawer_{row}",
                "kind": "deep",
                "x": 0.0,
                "z": z,
                "front_w": usable_w,
                "front_h": deep_h,
                "tray_w": 0.760,
                "tray_h": 0.120,
                "rail_fixed_offset": 0.412,
                "rail_moving_offset": 0.401,
                "travel": 0.34,
            }
        )
        z += deep_h + gap

    for row in range(3):
        for side, x in (("0", -col_x), ("1", col_x)):
            specs.append(
                {
                    "name": f"shallow_drawer_{row}_{side}",
                    "kind": "shallow",
                    "x": x,
                    "z": z,
                    "front_w": col_w,
                    "front_h": shallow_h,
                    "tray_w": 0.345,
                    "tray_h": 0.065,
                    "rail_fixed_offset": 0.194,
                    "rail_moving_offset": 0.183,
                    "travel": 0.32,
                }
            )
        z += shallow_h + gap
    return specs


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_tool_chest")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.115, 0.12, 1.0))
    red = model.material("red_powder_coat", rgba=(0.72, 0.035, 0.025, 1.0))
    rail_steel = model.material("brushed_rail_steel", rgba=(0.64, 0.66, 0.63, 1.0))
    black = model.material("black_recess_shadow", rgba=(0.006, 0.007, 0.008, 1.0))
    rubber = model.material("rubber_black", rgba=(0.018, 0.018, 0.016, 1.0))

    body = model.part("body")

    # Heavy welded steel carcass: box-like panels leave the front open for drawers.
    _add_box(body, "left_side", (WALL, DEPTH, HEIGHT), (-WIDTH / 2 + WALL / 2, 0.0, HEIGHT / 2), dark_steel)
    _add_box(body, "right_side", (WALL, DEPTH, HEIGHT), (WIDTH / 2 - WALL / 2, 0.0, HEIGHT / 2), dark_steel)
    _add_box(body, "back_panel", (WIDTH, WALL, HEIGHT), (0.0, DEPTH / 2 - WALL / 2, HEIGHT / 2), dark_steel)
    _add_box(body, "top_panel", (WIDTH, DEPTH, WALL), (0.0, 0.0, HEIGHT - WALL / 2), dark_steel)
    _add_box(body, "bottom_panel", (WIDTH, DEPTH, WALL), (0.0, 0.0, WALL / 2), dark_steel)

    front_y = -DEPTH / 2 + 0.016
    _add_box(body, "front_left_post", (0.040, 0.032, HEIGHT), (-WIDTH / 2 + 0.020, front_y, HEIGHT / 2), dark_steel)
    _add_box(body, "front_right_post", (0.040, 0.032, HEIGHT), (WIDTH / 2 - 0.020, front_y, HEIGHT / 2), dark_steel)
    _add_box(body, "front_top_lip", (WIDTH, 0.032, 0.035), (0.0, front_y, HEIGHT - 0.0175), dark_steel)
    _add_box(body, "front_bottom_lip", (WIDTH, 0.032, 0.045), (0.0, front_y, 0.0225), dark_steel)

    # Upper center divider and visible drawer-row separators.
    _add_box(body, "upper_center_divider", (0.020, DEPTH - 0.060, 0.355), (0.0, -0.005, 0.585), dark_steel)
    for idx, z in enumerate((0.242, 0.414, 0.524, 0.641)):
        _add_box(body, f"front_separator_{idx}", (0.840, 0.032, 0.010), (0.0, front_y, z), dark_steel)

    # Stout base pads make the chest read as a heavy workshop cabinet, not a wall unit.
    for idx, x in enumerate((-0.34, 0.34)):
        for jdx, y in enumerate((-0.19, 0.19)):
            _add_box(body, f"foot_bracket_{idx}_{jdx}", (0.090, 0.070, 0.025), (x, y, -0.0125), dark_steel)
            _add_box(body, f"rubber_foot_{idx}_{jdx}", (0.070, 0.050, 0.018), (x, y, -0.034), rubber)

    # Fixed ball-bearing drawer runners, attached to side walls or the upper divider.
    rail_len = 0.420
    rail_y = -0.030
    rail_size = (0.012, rail_len, 0.018)
    for spec in _drawer_specs():
        z = spec["z"]
        x0 = spec["x"]
        for side, sign in (("a", -1.0), ("b", 1.0)):
            fixed_x = x0 + sign * spec["rail_fixed_offset"]
            _add_box(body, f"{spec['name']}_fixed_rail_{side}", rail_size, (fixed_x, rail_y, z), rail_steel)
            rail_min = fixed_x - rail_size[0] / 2.0
            rail_max = fixed_x + rail_size[0] / 2.0
            if abs(fixed_x) < 0.08:
                support_face = -0.010 if fixed_x < 0.0 else 0.010
                span = abs(support_face - (rail_max if fixed_x < 0.0 else rail_min)) + 0.004
                center_x = (support_face + (rail_max if fixed_x < 0.0 else rail_min)) / 2.0
            else:
                support_face = -(WIDTH / 2.0 - WALL) if fixed_x < 0.0 else (WIDTH / 2.0 - WALL)
                span = abs((rail_min if fixed_x < 0.0 else rail_max) - support_face) + 0.004
                center_x = ((rail_min if fixed_x < 0.0 else rail_max) + support_face) / 2.0
            _add_box(body, f"{spec['name']}_rail_mount_{side}", (span, 0.050, 0.020), (center_x, rail_y, z), rail_steel)
            _add_rail_bearings(body, fixed_x, z, rail_steel, f"{spec['name']}_{side}")

    # Each drawer is a separate moving link on its own prismatic slide.
    for spec in _drawer_specs():
        drawer = model.part(spec["name"])
        front_w = spec["front_w"]
        front_h = spec["front_h"]
        tray_w = spec["tray_w"]
        tray_h = spec["tray_h"]
        handle_w = front_w * 0.72
        handle_h = min(0.042, front_h * 0.40)
        handle_z = 0.0 if front_h < 0.13 else front_h * 0.18

        drawer.visual(
            Box((front_w, FRONT_THICKNESS, front_h)),
            origin=Origin(),
            material=red,
            name="front_panel",
        )
        _add_box(
            drawer,
            "handle_shadow",
            (handle_w * 0.98, 0.003, handle_h * 0.88),
            (0.0, -FRONT_THICKNESS / 2 - 0.00125, handle_z),
            black,
        )
        drawer.visual(
            Cylinder(radius=0.006, length=handle_w * 0.82),
            origin=Origin(
                xyz=(0.0, -FRONT_THICKNESS / 2 - 0.0055, handle_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=rail_steel,
            name="handle_bar",
        )
        for idx, sx in enumerate((-1.0, 1.0)):
            _add_box(
                drawer,
                f"handle_stanchion_{idx}",
                (0.014, 0.010, handle_h * 0.62),
                (sx * handle_w * 0.35, -FRONT_THICKNESS / 2 - 0.0045, handle_z),
                rail_steel,
            )

        tray_depth = 0.420
        tray_y = FRONT_THICKNESS / 2 + 0.045 + tray_depth / 2.0
        bottom_z = -front_h / 2.0 + 0.014
        side_z = bottom_z + 0.006 + tray_h / 2.0
        _add_box(drawer, "tray_bottom", (tray_w, tray_depth, 0.012), (0.0, tray_y, bottom_z), dark_steel)
        _add_box(drawer, "tray_left_wall", (0.012, tray_depth, tray_h), (-tray_w / 2 + 0.006, tray_y, side_z), dark_steel)
        _add_box(drawer, "tray_right_wall", (0.012, tray_depth, tray_h), (tray_w / 2 - 0.006, tray_y, side_z), dark_steel)
        _add_box(drawer, "tray_back_wall", (tray_w, 0.014, tray_h), (0.0, FRONT_THICKNESS / 2 + 0.045 + tray_depth - 0.007, side_z), dark_steel)
        _add_box(drawer, "tray_front_wall", (tray_w, 0.090, tray_h * 0.72), (0.0, FRONT_THICKNESS / 2 + 0.045, side_z), dark_steel)

        # Moving slide members ride just inside the fixed rails; small brackets tie them to the tray sides.
        moving_offset = spec["rail_moving_offset"]
        for idx, sx in enumerate((-1.0, 1.0)):
            rail_x = sx * moving_offset
            _add_box(drawer, f"moving_rail_{idx}", (0.010, rail_len, 0.016), (rail_x, rail_y - FRONT_Y, side_z), rail_steel)
            _add_box(
                drawer,
                f"rail_bracket_{idx}",
                (0.020, 0.040, 0.018),
                (sx * (tray_w / 2 + 0.008), FRONT_THICKNESS / 2 + 0.070, side_z),
                rail_steel,
            )

        model.articulation(
            f"body_to_{spec['name']}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(spec["x"], FRONT_Y, spec["z"])),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=180.0, velocity=0.45, lower=0.0, upper=spec["travel"]),
            motion_properties=MotionProperties(damping=8.0, friction=2.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    specs = _drawer_specs()
    ctx.check("eight drawers present", len(specs) == 8)
    ctx.check(
        "six shallow and two deep drawers",
        sum(1 for spec in specs if spec["kind"] == "shallow") == 6
        and sum(1 for spec in specs if spec["kind"] == "deep") == 2,
    )

    for spec in specs:
        drawer = object_model.get_part(spec["name"])
        joint = object_model.get_articulation(f"body_to_{spec['name']}")
        ctx.check(
            f"{spec['name']} uses a prismatic rail",
            joint.articulation_type == ArticulationType.PRISMATIC,
        )
        ctx.expect_gap(
            body,
            drawer,
            axis="y",
            min_gap=0.001,
            max_gap=0.004,
            positive_elem="left_side",
            negative_elem="front_panel",
            name=f"{spec['name']} front sits just proud of the carcass",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="tray_bottom",
            elem_b="left_side",
            min_overlap=0.36,
            name=f"{spec['name']} tray is fully inserted when closed",
        )
        ctx.expect_within(
            drawer,
            body,
            axes="z",
            inner_elem="tray_bottom",
            outer_elem="left_side",
            margin=0.0,
            name=f"{spec['name']} tray height stays inside body",
        )

        closed_pos = ctx.part_world_position(drawer)
        with ctx.pose({joint: spec["travel"]}):
            open_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                elem_a="tray_bottom",
                elem_b="left_side",
                min_overlap=0.08,
                name=f"{spec['name']} rails retain insertion at full extension",
            )
        ctx.check(
            f"{spec['name']} extends outward",
            closed_pos is not None and open_pos is not None and open_pos[1] < closed_pos[1] - 0.25,
            details=f"closed={closed_pos}, open={open_pos}",
        )

    return ctx.report()


object_model = build_object_model()
