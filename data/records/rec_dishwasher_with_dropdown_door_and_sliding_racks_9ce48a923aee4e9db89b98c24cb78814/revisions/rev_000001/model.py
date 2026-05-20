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


def _rod_orientation(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi / 2.0, 0.0)
    if axis == "y":
        return (-math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_rod(part, name: str, axis: str, center, length: float, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=_rod_orientation(axis)),
        material=material,
        name=name,
    )


def _cq_box(size, center):
    return cq.Workplane("XY").box(*size).translate(center)


def _build_hollow_tub_mesh():
    """One fused open-front stainless wash chamber in dishwasher scale."""
    width = 0.66
    depth = 0.62
    height = 0.82
    wall = 0.030
    zc = 0.47
    tub = _cq_box((wall, depth, height), (-width / 2 + wall / 2, 0.0, zc))
    tub = tub.union(_cq_box((wall, depth, height), (width / 2 - wall / 2, 0.0, zc)))
    tub = tub.union(_cq_box((width, wall, height), (0.0, depth / 2 - wall / 2, zc)))
    tub = tub.union(_cq_box((width, depth, wall), (0.0, 0.0, 0.06 + wall / 2)))
    tub = tub.union(_cq_box((width, depth, wall), (0.0, 0.0, 0.06 + height - wall / 2)))
    tub = tub.union(_cq_box((width, 0.050, 0.080), (0.0, -depth / 2 + 0.025, 0.040)))
    return tub


def _add_tub_details(tub, materials) -> None:
    steel = materials["steel"]
    gasket = materials["gasket"]
    rail = materials["rail"]

    tub.visual(Box((0.022, 0.012, 0.720)), origin=Origin(xyz=(-0.298, -0.312, 0.465)), material=gasket, name="gasket_0")
    tub.visual(Box((0.022, 0.012, 0.720)), origin=Origin(xyz=(0.298, -0.312, 0.465)), material=gasket, name="gasket_1")
    tub.visual(Box((0.610, 0.012, 0.022)), origin=Origin(xyz=(0.0, -0.312, 0.815)), material=gasket, name="gasket_top")
    tub.visual(Box((0.610, 0.012, 0.022)), origin=Origin(xyz=(0.0, -0.312, 0.115)), material=gasket, name="gasket_bottom")

    for idx, z in enumerate((0.225, 0.505)):
        tub.visual(Box((0.034, 0.500, 0.014)), origin=Origin(xyz=(-0.284, 0.000, z)), material=rail, name=f"rail_{idx}_0")
        tub.visual(Box((0.034, 0.500, 0.014)), origin=Origin(xyz=(0.284, 0.000, z)), material=rail, name=f"rail_{idx}_1")

    # Hinge blocks reach forward from the toe kick and support the door pin without
    # filling the front loading opening.
    tub.visual(Box((0.060, 0.090, 0.022)), origin=Origin(xyz=(-0.250, -0.345, 0.060)), material=steel, name="hinge_block_0")
    tub.visual(Box((0.060, 0.090, 0.022)), origin=Origin(xyz=(0.250, -0.345, 0.060)), material=steel, name="hinge_block_1")

    _add_rod(tub, "feed_riser", "z", (0.275, 0.205, 0.350), 0.520, 0.014, steel)
    _add_rod(tub, "lower_boss", "z", (0.000, -0.020, 0.112), 0.062, 0.028, steel)
    _add_rod(tub, "upper_boss", "z", (0.000, -0.015, 0.428), 0.058, 0.022, steel)
    _add_rod(tub, "upper_feed", "x", (0.137, 0.095, 0.430), 0.285, 0.008, steel)
    _add_rod(tub, "upper_feed_drop", "y", (0.000, 0.040, 0.430), 0.115, 0.008, steel)
    _add_rod(tub, "upper_feed_side", "y", (0.275, 0.150, 0.430), 0.125, 0.008, steel)


def _add_rack(part, materials, *, upper: bool) -> None:
    wire = materials["rack_wire"]
    radius = 0.0045
    width = 0.520
    depth = 0.480
    side_h = 0.135 if upper else 0.155

    _add_rod(part, "front_wire", "x", (0.0, -depth / 2, 0.000), width + 0.020, radius, wire)
    _add_rod(part, "rear_wire", "x", (0.0, depth / 2, 0.000), width + 0.020, radius, wire)
    _add_rod(part, "side_wire_0", "y", (-width / 2, 0.0, 0.000), depth + 0.020, radius, wire)
    _add_rod(part, "side_wire_1", "y", (width / 2, 0.0, 0.000), depth + 0.020, radius, wire)
    _add_rod(part, "top_front_wire", "x", (0.0, -depth / 2, side_h), width + 0.020, radius, wire)
    _add_rod(part, "top_rear_wire", "x", (0.0, depth / 2, side_h), width + 0.020, radius, wire)
    _add_rod(part, "top_side_wire_0", "y", (-width / 2, 0.0, side_h), depth + 0.020, radius, wire)
    _add_rod(part, "top_side_wire_1", "y", (width / 2, 0.0, side_h), depth + 0.020, radius, wire)
    _add_rod(part, "runner_0", "y", (-0.253, 0.0, -0.006), 0.500, 0.007, wire)
    _add_rod(part, "runner_1", "y", (0.253, 0.0, -0.006), 0.500, 0.007, wire)

    for i, x in enumerate((-width / 2, width / 2)):
        for j, y in enumerate((-depth / 2, depth / 2)):
            _add_rod(part, f"corner_post_{i}_{j}", "z", (x, y, side_h / 2), side_h, radius, wire)

    for idx, x in enumerate((-0.195, -0.130, -0.065, 0.000, 0.065, 0.130, 0.195)):
        _add_rod(part, f"floor_y_{idx}", "y", (x, 0.0, 0.000), depth * 1.04, radius * 0.78, wire)
    for idx, y in enumerate((-0.180, -0.120, -0.060, 0.000, 0.060, 0.120, 0.180)):
        _add_rod(part, f"floor_x_{idx}", "x", (0.0, y, 0.000), width * 1.02, radius * 0.78, wire)

    if upper:
        tine_rows = ((-0.110, -0.135), (-0.035, -0.135), (0.040, 0.120), (0.115, 0.120))
        tine_len = 0.075
    else:
        tine_rows = ((-0.150, -0.140), (-0.075, -0.140), (0.000, 0.135), (0.075, 0.135), (0.150, 0.135))
        tine_len = 0.100
    for idx, (x, y0) in enumerate(tine_rows):
        _add_rod(part, f"tine_spine_{idx}", "y", (x, 0.0, 0.000), depth * 0.92, radius * 0.68, wire)
        for k in range(5):
            y = y0 + k * 0.055 * (1 if y0 < 0 else -1)
            _add_rod(part, f"tine_{idx}_{k}", "z", (x, y, tine_len / 2), tine_len, radius * 0.70, wire)


def _add_caddy(part, materials) -> None:
    plastic = materials["caddy"]
    r = 0.004
    w = 0.165
    d = 0.235
    h = 0.130

    for z, suffix in ((0.0, "bottom"), (h, "top")):
        _add_rod(part, f"{suffix}_front", "x", (0.0, -d / 2, z), w, r, plastic)
        _add_rod(part, f"{suffix}_rear", "x", (0.0, d / 2, z), w, r, plastic)
        _add_rod(part, f"{suffix}_side_0", "y", (-w / 2, 0.0, z), d, r, plastic)
        _add_rod(part, f"{suffix}_side_1", "y", (w / 2, 0.0, z), d, r, plastic)
    for i, x in enumerate((-w / 2, w / 2)):
        for j, y in enumerate((-d / 2, d / 2)):
            _add_rod(part, f"post_{i}_{j}", "z", (x, y, h / 2), h, r, plastic)
    for idx, x in enumerate((-0.045, 0.000, 0.045)):
        _add_rod(part, f"divider_{idx}", "y", (x, 0.0, h * 0.58), d, r * 0.85, plastic)
        _add_rod(part, f"floor_slot_{idx}", "y", (x, 0.0, 0.000), d, r * 0.8, plastic)
    for idx, y in enumerate((-0.075, 0.000, 0.075)):
        _add_rod(part, f"side_slat_{idx}_0", "x", (0.0, y, h * 0.58), w, r * 0.85, plastic)
    _add_rod(part, "mid_side_0", "y", (-w / 2, 0.0, h * 0.58), d, r * 0.85, plastic)
    _add_rod(part, "mid_side_1", "y", (w / 2, 0.0, h * 0.58), d, r * 0.85, plastic)
    _add_rod(part, "mid_front", "x", (0.0, -d / 2, h * 0.58), w, r * 0.85, plastic)
    _add_rod(part, "mid_rear", "x", (0.0, d / 2, h * 0.58), w, r * 0.85, plastic)

    _add_rod(part, "handle_base_0", "y", (-0.055, 0.0, h), d, r * 0.85, plastic)
    _add_rod(part, "handle_base_1", "y", (0.055, 0.0, h), d, r * 0.85, plastic)
    _add_rod(part, "handle_post_0", "z", (-0.055, 0.0, h + 0.030), 0.068, r, plastic)
    _add_rod(part, "handle_post_1", "z", (0.055, 0.0, h + 0.030), 0.068, r, plastic)
    _add_rod(part, "handle_grip", "x", (0.0, 0.0, h + 0.060), 0.110, r, plastic)


def _add_spray_arm(part, materials, *, upper: bool) -> None:
    plastic = materials["spray"]
    hub_r = 0.024 if not upper else 0.020
    arm_len = 0.470 if not upper else 0.400
    arm_w = 0.036 if not upper else 0.030
    thickness = 0.014

    _add_rod(part, "hub", "z", (0.0, 0.0, 0.0), thickness * 1.25, hub_r, plastic)
    part.visual(Box((arm_len, arm_w, thickness)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=plastic, name="main_bar")
    part.visual(Box((arm_len * 0.58, arm_w * 0.70, thickness * 0.82)), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, math.radians(90))), material=plastic, name="cross_bar")
    for idx, x in enumerate((-0.165, -0.090, 0.090, 0.165)):
        part.visual(Box((0.022, 0.010, 0.004)), origin=Origin(xyz=(x, 0.010 if x > 0 else -0.010, 0.009)), material=materials["dark"], name=f"jet_{idx}")


def _add_cup_shelf(part, materials) -> None:
    wire = materials["rack_wire"]
    r = 0.0038
    shelf_x = 0.185
    shelf_y = 0.285
    _add_rod(part, "pivot", "y", (0.0, 0.0, 0.0), shelf_y, r * 1.15, wire)
    _add_rod(part, "front_edge", "y", (shelf_x, 0.0, 0.0), shelf_y, r, wire)
    _add_rod(part, "side_edge_0", "x", (shelf_x / 2, -shelf_y / 2, 0.0), shelf_x, r, wire)
    _add_rod(part, "side_edge_1", "x", (shelf_x / 2, shelf_y / 2, 0.0), shelf_x, r, wire)
    for idx, y in enumerate((-0.095, -0.045, 0.005, 0.055, 0.105)):
        _add_rod(part, f"shelf_wire_{idx}", "x", (shelf_x / 2, y, 0.0), shelf_x + 0.012, r * 0.80, wire)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="black_glass_dishwasher")

    materials = {
        "steel": model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0)),
        "dark": model.material("dark_plastic", rgba=(0.025, 0.027, 0.030, 1.0)),
        "glass": model.material("black_glass", rgba=(0.002, 0.004, 0.007, 0.78)),
        "gasket": model.material("rubber_gasket", rgba=(0.004, 0.004, 0.004, 1.0)),
        "rail": model.material("polished_rail", rgba=(0.80, 0.82, 0.78, 1.0)),
        "rack_wire": model.material("nylon_coated_wire", rgba=(0.83, 0.86, 0.88, 1.0)),
        "caddy": model.material("pale_gray_plastic", rgba=(0.68, 0.70, 0.70, 1.0)),
        "spray": model.material("blue_gray_spray", rgba=(0.30, 0.36, 0.42, 1.0)),
        "button": model.material("soft_gray_button", rgba=(0.36, 0.38, 0.39, 1.0)),
    }

    tub = model.part("tub")
    tub.visual(
        mesh_from_cadquery(_build_hollow_tub_mesh(), "open_stainless_tub", tolerance=0.004, angular_tolerance=0.25),
        material=materials["steel"],
        name="hollow_tub",
    )
    _add_tub_details(tub, materials)

    door = model.part("door")
    door.visual(Box((0.640, 0.050, 0.760)), origin=Origin(xyz=(0.0, -0.030, 0.380)), material=materials["dark"], name="door_slab")
    door.visual(Box((0.570, 0.006, 0.500)), origin=Origin(xyz=(0.0, -0.057, 0.370)), material=materials["glass"], name="glass_panel")
    door.visual(Box((0.620, 0.008, 0.125)), origin=Origin(xyz=(0.0, -0.059, 0.690)), material=materials["dark"], name="control_fascia")
    door.visual(Box((0.560, 0.006, 0.600)), origin=Origin(xyz=(0.0, -0.003, 0.390)), material=materials["steel"], name="inner_liner")
    _add_rod(door, "handle_bar", "x", (0.0, -0.078, 0.600), 0.460, 0.012, materials["steel"])
    _add_rod(door, "handle_stem_0", "y", (-0.200, -0.066, 0.600), 0.030, 0.006, materials["steel"])
    _add_rod(door, "handle_stem_1", "y", (0.200, -0.066, 0.600), 0.030, 0.006, materials["steel"])
    _add_rod(door, "hinge_pin", "x", (0.0, -0.030, 0.000), 0.560, 0.014, materials["steel"])
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=door,
        origin=Origin(xyz=(0.0, -0.335, 0.085)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.0, lower=0.0, upper=1.55),
    )

    lower_rack = model.part("lower_rack")
    _add_rack(lower_rack, materials, upper=False)
    lower_slide = model.articulation(
        "lower_rack_slide",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lower_rack,
        origin=Origin(xyz=(0.0, -0.015, 0.245)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.45, lower=0.0, upper=0.430),
    )

    upper_rack = model.part("upper_rack")
    _add_rack(upper_rack, materials, upper=True)
    upper_slide = model.articulation(
        "upper_rack_slide",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=upper_rack,
        origin=Origin(xyz=(0.0, -0.005, 0.535)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.40, lower=0.0, upper=0.360),
    )

    caddy = model.part("caddy")
    _add_caddy(caddy, materials)
    model.articulation("caddy_mount", ArticulationType.FIXED, parent=lower_rack, child=caddy, origin=Origin(xyz=(0.150, -0.045, 0.018)))

    lower_spray = model.part("lower_spray")
    _add_spray_arm(lower_spray, materials, upper=False)
    model.articulation(
        "lower_spray_spin",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=lower_spray,
        origin=Origin(xyz=(0.0, -0.020, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )

    upper_spray = model.part("upper_spray")
    _add_spray_arm(upper_spray, materials, upper=True)
    model.articulation(
        "upper_spray_spin",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=upper_spray,
        origin=Origin(xyz=(0.0, -0.015, 0.465)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )

    dial = model.part("dial")
    knob = KnobGeometry(
        0.078,
        0.025,
        body_style="skirted",
        top_diameter=0.060,
        skirt=KnobSkirt(0.088, 0.006, flare=0.06, chamfer=0.0015),
        grip=KnobGrip(style="fluted", count=24, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    dial.visual(mesh_from_geometry(knob, "cycle_dial"), origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=materials["steel"], name="dial_cap")
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(-0.195, -0.062, 0.700)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=8.0),
    )

    for idx, x in enumerate((0.040, 0.120, 0.200)):
        button = model.part(f"button_{idx}")
        button.visual(Box((0.052, 0.012, 0.028)), origin=Origin(xyz=(0.0, -0.006, 0.0)), material=materials["button"], name="button_cap")
        model.articulation(
            f"button_{idx}_press",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x, -0.062, 0.700)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.010),
        )

    cup_shelf = model.part("cup_shelf")
    _add_cup_shelf(cup_shelf, materials)
    model.articulation(
        "cup_shelf_pivot",
        ArticulationType.REVOLUTE,
        parent=upper_rack,
        child=cup_shelf,
        origin=Origin(xyz=(-0.254, -0.010, 0.135)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tub = object_model.get_part("tub")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    caddy = object_model.get_part("caddy")

    door_hinge = object_model.get_articulation("door_hinge")
    lower_slide = object_model.get_articulation("lower_rack_slide")
    upper_slide = object_model.get_articulation("upper_rack_slide")
    cup_pivot = object_model.get_articulation("cup_shelf_pivot")

    tub_aabb = ctx.part_world_aabb(tub)
    ctx.check(
        "domestic dishwasher scale",
        tub_aabb is not None and (tub_aabb[1][2] - tub_aabb[0][2]) > 0.80 and (tub_aabb[1][0] - tub_aabb[0][0]) > 0.60,
        details=f"tub_aabb={tub_aabb}",
    )

    ctx.expect_gap(
        tub,
        door,
        axis="y",
        min_gap=0.003,
        max_gap=0.045,
        positive_elem="hollow_tub",
        negative_elem="inner_liner",
        name="closed door is just in front of tub opening",
    )
    ctx.expect_within(lower_rack, tub, axes="xz", margin=0.010, name="lower rack fits inside chamber width and height")
    ctx.expect_within(upper_rack, tub, axes="xz", margin=0.010, name="upper rack fits inside chamber width and height")
    ctx.expect_within(caddy, lower_rack, axes="xy", margin=0.020, name="silverware caddy rides within lower rack footprint")

    closed_glass = ctx.part_element_world_aabb(door, elem="glass_panel")
    with ctx.pose({door_hinge: 1.25}):
        open_glass = ctx.part_element_world_aabb(door, elem="glass_panel")
    ctx.check(
        "door drops outward on lower hinge",
        closed_glass is not None
        and open_glass is not None
        and open_glass[0][1] < closed_glass[0][1] - 0.25
        and open_glass[1][2] < closed_glass[1][2] - 0.20,
        details=f"closed={closed_glass}, open={open_glass}",
    )

    lower_rest = ctx.part_world_position(lower_rack)
    upper_rest = ctx.part_world_position(upper_rack)
    with ctx.pose({lower_slide: 0.40, upper_slide: 0.32, door_hinge: 1.25}):
        lower_out = ctx.part_world_position(lower_rack)
        upper_out = ctx.part_world_position(upper_rack)
    ctx.check(
        "racks slide out toward the open door",
        lower_rest is not None
        and lower_out is not None
        and upper_rest is not None
        and upper_out is not None
        and lower_out[1] < lower_rest[1] - 0.35
        and upper_out[1] < upper_rest[1] - 0.28,
        details=f"lower={lower_rest}->{lower_out}, upper={upper_rest}->{upper_out}",
    )

    shelf_rest = ctx.part_world_aabb("cup_shelf")
    with ctx.pose({cup_pivot: 1.20}):
        shelf_folded = ctx.part_world_aabb("cup_shelf")
    ctx.check(
        "cup shelf folds upward on side pivots",
        shelf_rest is not None and shelf_folded is not None and shelf_folded[1][2] > shelf_rest[1][2] + 0.08,
        details=f"rest={shelf_rest}, folded={shelf_folded}",
    )

    for name in ("lower_spray_spin", "upper_spray_spin", "dial_spin"):
        joint = object_model.get_articulation(name)
        ctx.check(f"{name} is continuous", joint.articulation_type == ArticulationType.CONTINUOUS)

    for idx in range(3):
        joint = object_model.get_articulation(f"button_{idx}_press")
        button = object_model.get_part(f"button_{idx}")
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: 0.010}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"button_{idx} presses inward",
            joint.articulation_type == ArticulationType.PRISMATIC and rest is not None and pressed is not None and pressed[1] > rest[1] + 0.008,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
