from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _hollow_tub_shell() -> cq.Workplane:
    """A one-piece stainless tub shell: open front, thin sides/top/bottom/back."""
    outer = (
        cq.Workplane("XY")
        .box(0.600, 0.580, 0.760)
        .translate((0.0, 0.0, 0.440))
    )
    cutter = (
        cq.Workplane("XY")
        .box(0.540, 0.615, 0.680)
        # Extends through the front, but stops short of the rear to leave a back wall.
        .translate((0.0, -0.035, 0.435))
    )
    return outer.cut(cutter)


def _door_liner_pan() -> cq.Workplane:
    """Shallow hollow inner door liner pan, open toward the dishwasher tub."""
    outer = (
        cq.Workplane("XY")
        .box(0.520, 0.045, 0.620)
        .translate((0.0, 0.018, 0.400))
    )
    cutter = (
        cq.Workplane("XY")
        .box(0.465, 0.055, 0.535)
        # Opens the pan through +Y while leaving a thin back plate and lip.
        .translate((0.0, 0.030, 0.405))
    )
    return outer.cut(cutter)


def _add_rack_wires(part, *, material: Material, name_prefix: str) -> None:
    """Wire basket made of intersecting stainless rods; local origin is rack center."""
    w = 0.485
    d = 0.455
    rod = 0.007
    upper_z = 0.085
    lower_z = -0.018

    # Lower and upper perimeter rectangles.
    for z, layer in ((lower_z, "lower"), (upper_z, "upper")):
        part.visual(Box((w, rod, rod)), origin=Origin(xyz=(0.0, -d / 2.0, z)), material=material, name=f"{name_prefix}_{layer}_front")
        part.visual(Box((w, rod, rod)), origin=Origin(xyz=(0.0, d / 2.0, z)), material=material, name=f"{name_prefix}_{layer}_rear")
        part.visual(Box((rod, d, rod)), origin=Origin(xyz=(-w / 2.0, 0.0, z)), material=material, name=f"{name_prefix}_{layer}_side_0")
        part.visual(Box((rod, d, rod)), origin=Origin(xyz=(w / 2.0, 0.0, z)), material=material, name=f"{name_prefix}_{layer}_side_1")

    # Vertical corner and mid-side posts tie the basket together.
    for x in (-w / 2.0, w / 2.0):
        for y in (-d / 2.0, d / 2.0, 0.0):
            part.visual(
                Box((rod, rod, upper_z - lower_z + rod)),
                origin=Origin(xyz=(x, y, (upper_z + lower_z) / 2.0)),
                material=material,
                name=f"{name_prefix}_post_{x:.2f}_{y:.2f}",
            )

    # Bottom cross wires and upright tines.
    for i, y in enumerate((-0.165, -0.105, -0.045, 0.015, 0.075, 0.135, 0.195)):
        part.visual(
            Box((w + 0.010, rod * 0.75, rod)),
            origin=Origin(xyz=(0.0, y, lower_z)),
            material=material,
            name=f"{name_prefix}_cross_{i}",
        )
        for j, x in enumerate((-0.150, -0.075, 0.000, 0.075, 0.150)):
            part.visual(
                Box((rod * 0.72, rod * 0.72, 0.075)),
                origin=Origin(xyz=(x, y, lower_z + 0.037)),
                material=material,
                name=f"{name_prefix}_tine_{i}_{j}",
            )

    # Outboard runners sit directly on the fixed tub rails.
    for x, side, conn_x in ((-0.2485, "side_0", -0.2455), (0.2485, "side_1", 0.2455)):
        part.visual(
            Box((rod, d, rod)),
            origin=Origin(xyz=(x, 0.0, 0.005)),
            material=material,
            name=f"{name_prefix}_runner_{side}",
        )
        for y, end in ((-d / 2.0, "front"), (d / 2.0, "rear")):
            part.visual(
                Box((0.030, rod, rod)),
                origin=Origin(xyz=(conn_x, y, 0.005)),
                material=material,
                name=f"{name_prefix}_runner_bridge_{side}_{end}",
            )


def _add_mug_shelf(part, *, direction: float, material: Material, name_prefix: str) -> None:
    """Small wire cup shelf; child frame is the side hinge line."""
    width = 0.165
    depth = 0.370
    rod = 0.006
    part.visual(
        Cylinder(radius=0.0065, length=depth),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=f"{name_prefix}_pivot_rod",
    )
    for y, label in ((-depth / 2.0, "front"), (depth / 2.0, "rear")):
        part.visual(
            Box((width, rod, rod)),
            origin=Origin(xyz=(direction * width / 2.0, y, 0.0)),
            material=material,
            name=f"{name_prefix}_{label}_wire",
        )
    part.visual(
        Box((rod, depth, rod)),
        origin=Origin(xyz=(direction * width, 0.0, 0.0)),
        material=material,
        name=f"{name_prefix}_outer_wire",
    )
    for i, x in enumerate((0.045, 0.085, 0.125)):
        part.visual(
            Box((rod * 0.75, depth, rod * 0.75)),
            origin=Origin(xyz=(direction * x, 0.0, 0.0)),
            material=material,
            name=f"{name_prefix}_slat_{i}",
        )


def _add_spray_arm(part, *, material: Material, accent: Material, length: float, name_prefix: str) -> None:
    part.visual(Cylinder(radius=0.034, length=0.030), origin=Origin(xyz=(0.0, 0.0, 0.000)), material=material, name=f"{name_prefix}_hub")
    part.visual(Box((length, 0.046, 0.014)), origin=Origin(xyz=(0.0, 0.0, 0.004)), material=material, name=f"{name_prefix}_blade")
    for i, x in enumerate((-length * 0.42, -length * 0.18, length * 0.18, length * 0.42)):
        part.visual(
            Box((0.030, 0.009, 0.006)),
            origin=Origin(xyz=(x, 0.012 if i % 2 else -0.012, 0.012)),
            material=accent,
            name=f"{name_prefix}_jet_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_dishwasher")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    dark = model.material("black_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    shadow = model.material("deep_shadow", rgba=(0.03, 0.035, 0.040, 1.0))
    rack_wire = model.material("polished_rack_wire", rgba=(0.88, 0.90, 0.88, 1.0))
    grey = model.material("molded_grey", rgba=(0.50, 0.52, 0.51, 1.0))
    blue = model.material("blue_start_button", rgba=(0.10, 0.33, 0.78, 1.0))
    rinse_blue = model.material("rinse_aid_blue", rgba=(0.18, 0.38, 0.70, 1.0))

    tub = model.part("tub")
    tub.visual(mesh_from_cadquery(_hollow_tub_shell(), "hollow_tub_shell"), material=stainless, name="hollow_tub_shell")
    tub.visual(Box((0.610, 0.105, 0.060)), origin=Origin(xyz=(0.0, -0.240, 0.030)), material=dark, name="toe_kick")
    tub.visual(Box((0.555, 0.018, 0.025)), origin=Origin(xyz=(0.0, -0.304, 0.058)), material=dark, name="lower_hinge_sill")
    tub.visual(
        Cylinder(radius=0.010, length=0.540),
        origin=Origin(xyz=(0.0, -0.315, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="lower_hinge_pin",
    )
    for x, label in ((-0.220, "side_0"), (0.220, "side_1")):
        tub.visual(
            Box((0.040, 0.032, 0.024)),
            origin=Origin(xyz=(x, -0.300, 0.060)),
            material=dark,
            name=f"hinge_knuckle_{label}",
        )
    tub.visual(
        Cylinder(radius=0.020, length=0.065),
        origin=Origin(xyz=(0.0, 0.000, 0.1275)),
        material=grey,
        name="lower_spray_socket",
    )
    tub.visual(
        Box((0.038, 0.276, 0.020)),
        origin=Origin(xyz=(0.0, 0.137, 0.455)),
        material=grey,
        name="upper_spray_feed",
    )
    for z, level in ((0.305, "lower"), (0.595, "upper")):
        for x, side in ((-0.258, "side_0"), (0.258, "side_1")):
            tub.visual(
                Box((0.012, 0.500, 0.014)),
                origin=Origin(xyz=(x, 0.015, z)),
                material=grey,
                name=f"{level}_rack_rail_{side}",
            )
            for y, end in ((-0.200, "front"), (0.225, "rear")):
                support_x = -0.267 if x < 0.0 else 0.267
                tub.visual(
                    Box((0.018, 0.020, 0.018)),
                    origin=Origin(xyz=(support_x, y, z)),
                    material=grey,
                    name=f"{level}_rack_rail_bracket_{side}_{end}",
                )

    door = model.part("door")
    door.visual(Box((0.600, 0.035, 0.735)), origin=Origin(xyz=(0.0, -0.020, 0.392)), material=stainless, name="outer_door_panel")
    door.visual(mesh_from_cadquery(_door_liner_pan(), "door_liner_pan"), material=grey, name="door_liner_pan")
    door.visual(Box((0.540, 0.044, 0.018)), origin=Origin(xyz=(0.0, -0.016, 0.755)), material=dark, name="concealed_top_strip")
    # A compressible-looking front gasket touches the tub lip at the closed pose.
    door.visual(Box((0.012, 0.006, 0.650)), origin=Origin(xyz=(-0.276, 0.022, 0.430)), material=dark, name="gasket_side_0")
    door.visual(Box((0.012, 0.006, 0.650)), origin=Origin(xyz=(0.276, 0.022, 0.430)), material=dark, name="gasket_side_1")
    door.visual(Box((0.540, 0.006, 0.012)), origin=Origin(xyz=(0.0, 0.022, 0.752)), material=dark, name="gasket_top")
    door.visual(Box((0.540, 0.006, 0.012)), origin=Origin(xyz=(0.0, 0.022, 0.108)), material=dark, name="gasket_bottom")
    door.visual(Box((0.205, 0.048, 0.120)), origin=Origin(xyz=(0.125, 0.023, 0.330)), material=grey, name="detergent_dispenser")

    model.articulation(
        "tub_to_door",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=door,
        origin=Origin(xyz=(0.0, -0.315, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=70.0, velocity=1.2),
    )

    lower_rack = model.part("lower_rack")
    _add_rack_wires(lower_rack, material=rack_wire, name_prefix="lower_rack")
    model.articulation(
        "tub_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lower_rack,
        origin=Origin(xyz=(0.0, 0.015, 0.300)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.360, effort=45.0, velocity=0.35),
    )

    upper_rack = model.part("upper_rack")
    _add_rack_wires(upper_rack, material=rack_wire, name_prefix="upper_rack")
    model.articulation(
        "tub_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=upper_rack,
        origin=Origin(xyz=(0.0, 0.010, 0.590)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.340, effort=38.0, velocity=0.35),
    )

    for idx, (x, direction, axis) in enumerate(((-0.235, 1.0, (0.0, -1.0, 0.0)), (0.235, -1.0, (0.0, 1.0, 0.0)))):
        shelf = model.part(f"mug_shelf_{idx}")
        _add_mug_shelf(shelf, direction=direction, material=rack_wire, name_prefix=f"mug_shelf_{idx}")
        model.articulation(
            f"upper_rack_to_mug_shelf_{idx}",
            ArticulationType.REVOLUTE,
            parent=upper_rack,
            child=shelf,
            origin=Origin(xyz=(x, 0.0, 0.082)),
            axis=axis,
            motion_limits=MotionLimits(lower=0.0, upper=1.55, effort=4.0, velocity=1.8),
        )

    lower_spray_arm = model.part("lower_spray_arm")
    _add_spray_arm(lower_spray_arm, material=grey, accent=shadow, length=0.500, name_prefix="lower_spray")
    model.articulation(
        "tub_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.0, 0.000, 0.175)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=12.0),
    )

    upper_spray_arm = model.part("upper_spray_arm")
    _add_spray_arm(upper_spray_arm, material=grey, accent=shadow, length=0.430, name_prefix="upper_spray")
    model.articulation(
        "tub_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.0, 0.000, 0.480)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.5, velocity=14.0),
    )

    # Four separate concealed top-edge push buttons.
    button_specs = (
        ("start_button", -0.185, 0.058, blue),
        ("option_button_0", -0.085, 0.038, dark),
        ("option_button_1", -0.025, 0.038, dark),
        ("option_button_2", 0.035, 0.038, dark),
    )
    for name, x, sx, mat in button_specs:
        button = model.part(name)
        button.visual(Box((sx, 0.026, 0.008)), origin=Origin(xyz=(0.0, 0.0, 0.004)), material=mat, name="button_cap")
        model.articulation(
            f"door_to_{name}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x, -0.016, 0.764)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.005, effort=8.0, velocity=0.05),
        )

    rinse_cap = model.part("rinse_aid_cap")
    rinse_cap.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rinse_blue,
        name="cap_disk",
    )
    rinse_cap.visual(
        Box((0.008, 0.004, 0.040)),
        origin=Origin(xyz=(0.0, 0.013, 0.0)),
        material=shadow,
        name="grip_slot",
    )
    model.articulation(
        "door_to_rinse_aid_cap",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=rinse_cap,
        origin=Origin(xyz=(0.160, 0.047, 0.330)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tub = object_model.get_part("tub")
    door = object_model.get_part("door")
    upper_rack = object_model.get_part("upper_rack")
    lower_rack = object_model.get_part("lower_rack")

    door_joint = object_model.get_articulation("tub_to_door")
    upper_slide = object_model.get_articulation("tub_to_upper_rack")
    lower_slide = object_model.get_articulation("tub_to_lower_rack")
    start_push = object_model.get_articulation("door_to_start_button")

    ctx.expect_contact(door, tub, elem_a="gasket_side_0", elem_b="hollow_tub_shell", contact_tol=0.001, name="closed door gasket seats on tub lip")
    ctx.expect_within(upper_rack, tub, axes="xz", margin=0.010, name="upper rack fits between tub walls")
    ctx.expect_within(lower_rack, tub, axes="xz", margin=0.010, name="lower rack fits between tub walls")

    closed_door_aabb = ctx.part_world_aabb(door)
    closed_upper_pos = ctx.part_world_position(upper_rack)
    closed_lower_pos = ctx.part_world_position(lower_rack)
    with ctx.pose({door_joint: 1.20, upper_slide: 0.300, lower_slide: 0.320}):
        opened_door_aabb = ctx.part_world_aabb(door)
        extended_upper_pos = ctx.part_world_position(upper_rack)
        extended_lower_pos = ctx.part_world_position(lower_rack)
        ctx.expect_overlap(upper_rack, tub, axes="y", min_overlap=0.060, name="upper rack remains engaged on rails")
        ctx.expect_overlap(lower_rack, tub, axes="y", min_overlap=0.060, name="lower rack remains engaged on rails")
    ctx.check(
        "door opens outward and downward",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[0][1] < closed_door_aabb[0][1] - 0.20
        and opened_door_aabb[1][2] < closed_door_aabb[1][2] - 0.10,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )
    ctx.check(
        "upper rack slides out toward the user",
        closed_upper_pos is not None and extended_upper_pos is not None and extended_upper_pos[1] < closed_upper_pos[1] - 0.20,
        details=f"closed={closed_upper_pos}, extended={extended_upper_pos}",
    )
    ctx.check(
        "lower rack slides out toward the user",
        closed_lower_pos is not None and extended_lower_pos is not None and extended_lower_pos[1] < closed_lower_pos[1] - 0.20,
        details=f"closed={closed_lower_pos}, extended={extended_lower_pos}",
    )
    start_rest = ctx.part_world_position(object_model.get_part("start_button"))
    with ctx.pose({start_push: 0.004}):
        start_pressed = ctx.part_world_position(object_model.get_part("start_button"))
    ctx.check(
        "start button depresses into the concealed strip",
        start_rest is not None and start_pressed is not None and start_pressed[2] < start_rest[2] - 0.003,
        details=f"rest={start_rest}, pressed={start_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
