from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cyl_origin(center: tuple[float, float, float], axis: str) -> Origin:
    """Return an origin for a cylinder whose local +Z should read along axis."""
    if axis == "x":
        return Origin(xyz=center, rpy=(0.0, pi / 2.0, 0.0))
    if axis == "y":
        return Origin(xyz=center, rpy=(pi / 2.0, 0.0, 0.0))
    return Origin(xyz=center)


def _rod(part, name: str, *, center, length: float, radius: float, axis: str, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_cyl_origin(center, axis),
        material=material,
        name=name,
    )


def _make_rack(part, *, material, width: float = 0.50, depth: float = 0.48, height: float = 0.12) -> None:
    """Open welded-wire dishwasher rack centered on the child part frame."""
    r = 0.004
    top_z = height
    # Bottom perimeter basket frame.
    _rod(part, "front_rail", center=(0.0, -depth / 2.0, 0.0), length=width + 0.020, radius=r, axis="x", material=material)
    _rod(part, "rear_rail", center=(0.0, depth / 2.0, 0.0), length=width + 0.020, radius=r, axis="x", material=material)
    _rod(part, "side_rail_0", center=(-width / 2.0, 0.0, 0.0), length=depth + 0.020, radius=r, axis="y", material=material)
    _rod(part, "side_rail_1", center=(width / 2.0, 0.0, 0.0), length=depth + 0.020, radius=r, axis="y", material=material)
    # Runner shoes touch the fixed tub rails at rest and are welded into the
    # basket side wires.
    part.visual(Box((0.022, depth * 0.94, 0.014)), origin=Origin(xyz=(-0.262, 0.010, -0.005)), material=material, name="runner_0")
    part.visual(Box((0.022, depth * 0.94, 0.014)), origin=Origin(xyz=(0.262, 0.010, -0.005)), material=material, name="runner_1")

    # A real rack reads as an open grid, not a tray.
    for i, y in enumerate((-0.16, -0.08, 0.0, 0.08, 0.16)):
        _rod(part, f"cross_wire_{i}", center=(0.0, y, 0.0), length=width + 0.010, radius=0.0022, axis="x", material=material)
    for i, x in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
        _rod(part, f"foreaft_wire_{i}", center=(x, 0.0, 0.0), length=depth + 0.010, radius=0.0022, axis="y", material=material)

    # Uprights and top rim make the basket self-supporting and visibly connected.
    for ix, x in enumerate((-width / 2.0, width / 2.0)):
        for iy, y in enumerate((-depth / 2.0, depth / 2.0)):
            _rod(part, f"corner_post_{ix}_{iy}", center=(x, y, top_z / 2.0), length=top_z + 0.014, radius=r, axis="z", material=material)
    _rod(part, "top_front_rail", center=(0.0, -depth / 2.0, top_z), length=width + 0.020, radius=r, axis="x", material=material)
    _rod(part, "top_rear_rail", center=(0.0, depth / 2.0, top_z), length=width + 0.020, radius=r, axis="x", material=material)
    _rod(part, "top_side_rail_0", center=(-width / 2.0, 0.0, top_z), length=depth + 0.020, radius=r, axis="y", material=material)
    _rod(part, "top_side_rail_1", center=(width / 2.0, 0.0, top_z), length=depth + 0.020, radius=r, axis="y", material=material)

    # Tines are short vertical wires welded to the lower grid.
    for row, y in enumerate((-0.12, 0.0, 0.12)):
        for col, x in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
            _rod(
                part,
                f"tine_{row}_{col}",
                center=(x, y, 0.045),
                length=0.090,
                radius=0.0018,
                axis="z",
                material=material,
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_dishwasher")

    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_gap = model.material("shadowed_cavity", rgba=(0.04, 0.045, 0.05, 1.0))
    liner = model.material("molded_gray_liner", rgba=(0.64, 0.66, 0.65, 1.0))
    rack_wire = model.material("nylon_coated_wire", rgba=(0.88, 0.90, 0.88, 1.0))
    black = model.material("black_control", rgba=(0.015, 0.015, 0.018, 1.0))
    blue = model.material("blue_option_buttons", rgba=(0.08, 0.18, 0.35, 1.0))

    tub = model.part("tub")

    # Thin-walled open-front built-in shell: side walls, back, top, bottom pan,
    # and front flange leave the interior visibly hollow.
    tub.visual(Box((0.025, 0.62, 0.84)), origin=Origin(xyz=(-0.2975, 0.0, 0.43)), material=liner, name="side_wall_0")
    tub.visual(Box((0.025, 0.62, 0.84)), origin=Origin(xyz=(0.2975, 0.0, 0.43)), material=liner, name="side_wall_1")
    tub.visual(Box((0.62, 0.025, 0.84)), origin=Origin(xyz=(0.0, 0.2975, 0.43)), material=liner, name="back_wall")
    tub.visual(Box((0.62, 0.62, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.845)), material=liner, name="top_wall")
    tub.visual(Box((0.62, 0.62, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.020)), material=liner, name="bottom_pan")
    tub.visual(Box((0.060, 0.018, 0.84)), origin=Origin(xyz=(-0.295, -0.319, 0.43)), material=stainless, name="front_stile_0")
    tub.visual(Box((0.060, 0.018, 0.84)), origin=Origin(xyz=(0.295, -0.319, 0.43)), material=stainless, name="front_stile_1")
    tub.visual(Box((0.62, 0.018, 0.055)), origin=Origin(xyz=(0.0, -0.319, 0.832)), material=stainless, name="front_header")
    tub.visual(Box((0.62, 0.018, 0.055)), origin=Origin(xyz=(0.0, -0.319, 0.062)), material=stainless, name="front_sill")
    tub.visual(Box((0.48, 0.46, 0.006)), origin=Origin(xyz=(0.0, 0.015, 0.052)), material=dark_gap, name="recessed_floor_shadow")
    tub.visual(Cylinder(radius=0.020, length=0.0725), origin=Origin(xyz=(0.0, 0.010, 0.07125)), material=stainless, name="lower_spray_supply_post")
    tub.visual(Box((0.040, 0.245, 0.030)), origin=Origin(xyz=(0.0, 0.1625, 0.45499)), material=stainless, name="upper_spray_feed_duct")

    # Fixed side rails for the two pull-out racks, welded into the tub walls.
    for level, z in enumerate((0.245, 0.545)):
        tub.visual(Box((0.012, 0.49, 0.014)), origin=Origin(xyz=(-0.279, 0.010, z)), material=stainless, name=f"rack_rail_{level}_0")
        tub.visual(Box((0.012, 0.49, 0.014)), origin=Origin(xyz=(0.279, 0.010, z)), material=stainless, name=f"rack_rail_{level}_1")

    # Door hinged on the lower front edge.  The child frame is the hinge line.
    door = model.part("door")
    door.visual(Box((0.60, 0.045, 0.760)), origin=Origin(xyz=(0.0, -0.018, 0.380)), material=stainless, name="outer_slab")
    door.visual(Box((0.53, 0.008, 0.610)), origin=Origin(xyz=(0.0, 0.008, 0.395)), material=liner, name="inner_liner")
    door.visual(Box((0.36, 0.010, 0.018)), origin=Origin(xyz=(0.0, 0.015, 0.330)), material=dark_gap, name="detergent_pocket")
    door.visual(Box((0.56, 0.012, 0.050)), origin=Origin(xyz=(0.0, -0.046, 0.725)), material=black, name="control_strip")
    _rod(door, "lower_hinge_barrel", center=(0.0, 0.004, 0.0), length=0.54, radius=0.013, axis="x", material=stainless)

    model.articulation(
        "tub_to_door",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=door,
        origin=Origin(xyz=(0.0, -0.345, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.9, lower=0.0, upper=1.75),
    )

    # Inner drying vent flap directly below the control strip.
    vent_flap = model.part("vent_flap")
    vent_flap.visual(Box((0.22, 0.008, 0.050)), origin=Origin(xyz=(0.0, 0.004, -0.025)), material=liner, name="flap_panel")
    _rod(vent_flap, "flap_hinge_barrel", center=(0.0, 0.004, 0.0), length=0.24, radius=0.004, axis="x", material=stainless)
    model.articulation(
        "door_to_vent_flap",
        ArticulationType.REVOLUTE,
        parent=door,
        child=vent_flap,
        origin=Origin(xyz=(0.0, 0.012, 0.670)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=0.85),
    )

    # Rotary cycle knob mounted through the front control strip.
    knob = model.part("cycle_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.060,
            0.024,
            body_style="skirted",
            grip=KnobGrip(style="fluted", count=22, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0008, angle_deg=0.0),
        ),
        "cycle_knob",
    )
    knob.visual(knob_mesh, origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)), material=stainless, name="dial_cap")
    model.articulation(
        "door_to_cycle_knob",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=knob,
        origin=Origin(xyz=(-0.205, -0.064, 0.725)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=5.0),
    )

    # Four independent push buttons proud of the slim control strip.
    for i, x in enumerate((-0.070, -0.020, 0.030, 0.080)):
        button = model.part(f"button_{i}")
        button.visual(Box((0.038, 0.012, 0.024)), origin=Origin(), material=blue, name="button_cap")
        model.articulation(
            f"door_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x, -0.058, 0.725)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.04, lower=0.0, upper=0.006),
        )

    # Sliding racks as open wire baskets.  Their frames are centered at the rail
    # levels and travel out through the open front.
    lower_rack = model.part("lower_rack")
    _make_rack(lower_rack, material=rack_wire, height=0.125)
    model.articulation(
        "tub_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lower_rack,
        origin=Origin(xyz=(0.0, 0.010, 0.250)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=0.350),
    )

    upper_rack = model.part("upper_rack")
    _make_rack(upper_rack, material=rack_wire, height=0.105)
    model.articulation(
        "tub_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=upper_rack,
        origin=Origin(xyz=(0.0, 0.010, 0.550)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.35, lower=0.0, upper=0.330),
    )

    # Continuously rotating wash arms on vertical hubs inside the hollow tub.
    lower_arm = model.part("lower_spray_arm")
    lower_arm.visual(Cylinder(radius=0.035, length=0.035), origin=Origin(), material=stainless, name="hub")
    lower_arm.visual(Box((0.49, 0.045, 0.018)), origin=Origin(), material=liner, name="spray_blade")
    lower_arm.visual(Box((0.18, 0.032, 0.020)), origin=Origin(rpy=(0.0, 0.0, pi / 2.0)), material=liner, name="cross_blade")
    model.articulation(
        "tub_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.010, 0.125)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=18.0),
    )

    upper_arm = model.part("upper_spray_arm")
    upper_arm.visual(Cylinder(radius=0.030, length=0.030), origin=Origin(), material=stainless, name="hub")
    upper_arm.visual(Box((0.42, 0.038, 0.016)), origin=Origin(), material=liner, name="spray_blade")
    upper_arm.visual(Box((0.14, 0.028, 0.018)), origin=Origin(rpy=(0.0, 0.0, pi / 2.0)), material=liner, name="cross_blade")
    model.articulation(
        "tub_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.010, 0.425)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tub = object_model.get_part("tub")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    lower_arm = object_model.get_part("lower_spray_arm")
    upper_arm = object_model.get_part("upper_spray_arm")
    vent_flap = object_model.get_part("vent_flap")

    door_joint = object_model.get_articulation("tub_to_door")
    lower_slide = object_model.get_articulation("tub_to_lower_rack")
    upper_slide = object_model.get_articulation("tub_to_upper_rack")
    vent_joint = object_model.get_articulation("door_to_vent_flap")

    # The shell stays hollow: racks and spray arms live inside the tub footprint
    # with open front clearance rather than being fused into a block.
    ctx.expect_within(lower_rack, tub, axes="x", margin=0.004, name="lower rack fits between side walls")
    ctx.expect_within(upper_rack, tub, axes="x", margin=0.004, name="upper rack fits between side walls")
    ctx.expect_gap(lower_rack, lower_arm, axis="z", min_gap=0.055, name="lower rack clears lower spray arm")
    ctx.expect_gap(upper_rack, upper_arm, axis="z", min_gap=0.070, name="upper rack clears upper spray arm")
    ctx.expect_gap(upper_arm, lower_rack, axis="z", min_gap=0.010, name="upper spray arm is above lower rack")

    rest_lower = ctx.part_world_position(lower_rack)
    rest_upper = ctx.part_world_position(upper_rack)
    with ctx.pose({lower_slide: 0.32, upper_slide: 0.30}):
        pulled_lower = ctx.part_world_position(lower_rack)
        pulled_upper = ctx.part_world_position(upper_rack)
        ctx.expect_overlap(lower_rack, tub, axes="y", min_overlap=0.12, name="extended lower rack remains retained")
        ctx.expect_overlap(upper_rack, tub, axes="y", min_overlap=0.12, name="extended upper rack remains retained")
    ctx.check(
        "racks slide outward through the front",
        rest_lower is not None
        and pulled_lower is not None
        and pulled_lower[1] < rest_lower[1] - 0.25
        and rest_upper is not None
        and pulled_upper is not None
        and pulled_upper[1] < rest_upper[1] - 0.23,
        details=f"lower={rest_lower}->{pulled_lower}, upper={rest_upper}->{pulled_upper}",
    )

    closed_top = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.25}):
        open_top = ctx.part_world_aabb(door)
    ctx.check(
        "door drops downward and outward",
        closed_top is not None and open_top is not None and open_top[0][1] < closed_top[0][1] - 0.10 and open_top[1][2] < closed_top[1][2] - 0.15,
        details=f"closed={closed_top}, open={open_top}",
    )

    vent_rest = ctx.part_world_aabb(vent_flap)
    with ctx.pose({vent_joint: 0.6}):
        vent_open = ctx.part_world_aabb(vent_flap)
    ctx.check(
        "vent flap tips inward from the liner",
        vent_rest is not None and vent_open is not None and vent_open[1][1] > vent_rest[1][1] + 0.010,
        details=f"rest={vent_rest}, open={vent_open}",
    )

    return ctx.report()


object_model = build_object_model()
