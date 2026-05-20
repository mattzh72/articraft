from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_toaster_oven")

    painted_metal = model.material("warm_graphite_painted_metal", rgba=(0.24, 0.25, 0.25, 1.0))
    dark_enamel = model.material("dark_enamel_interior", rgba=(0.025, 0.026, 0.027, 1.0))
    brushed_steel = model.material("brushed_stainless_steel", rgba=(0.72, 0.72, 0.69, 1.0))
    black_polymer = model.material("satin_black_polymer", rgba=(0.055, 0.057, 0.060, 1.0))
    graphite_polymer = model.material("graphite_polymer_panel", rgba=(0.12, 0.125, 0.13, 1.0))
    smoked_glass = model.material("smoked_tempered_glass", rgba=(0.10, 0.14, 0.16, 0.42))
    white_mark = model.material("soft_white_markings", rgba=(0.86, 0.86, 0.82, 1.0))
    rubber = model.material("matte_black_elastomer", rgba=(0.025, 0.025, 0.026, 1.0))

    body = model.part("body")

    # Main cabinet: separate metal panels touch/overlap as a continuous folded
    # sheet-metal appliance shell, leaving a real front oven opening.
    body.visual(Box((0.500, 0.360, 0.050)), origin=Origin(xyz=(0.000, 0.000, 0.025)), material=painted_metal, name="bottom_pan")
    body.visual(Box((0.500, 0.360, 0.036)), origin=Origin(xyz=(0.000, 0.000, 0.282)), material=painted_metal, name="top_cover")
    body.visual(Box((0.035, 0.360, 0.250)), origin=Origin(xyz=(-0.2325, 0.000, 0.165)), material=painted_metal, name="side_wall_0")
    body.visual(Box((0.035, 0.360, 0.250)), origin=Origin(xyz=(0.2325, 0.000, 0.165)), material=painted_metal, name="side_wall_1")
    body.visual(Box((0.500, 0.030, 0.250)), origin=Origin(xyz=(0.000, 0.165, 0.165)), material=painted_metal, name="rear_wall")
    body.visual(Box((0.500, 0.030, 0.055)), origin=Origin(xyz=(0.000, -0.182, 0.0475)), material=painted_metal, name="front_lower_rail")
    body.visual(Box((0.500, 0.030, 0.050)), origin=Origin(xyz=(0.000, -0.182, 0.265)), material=painted_metal, name="front_upper_rail")
    body.visual(Box((0.034, 0.030, 0.210)), origin=Origin(xyz=(-0.233, -0.182, 0.155)), material=painted_metal, name="front_side_jamb")
    body.visual(Box((0.098, 0.032, 0.218)), origin=Origin(xyz=(0.185, -0.182, 0.155)), material=graphite_polymer, name="control_panel_face")

    opening_trim = mesh_from_geometry(
        BezelGeometry(
            (0.318, 0.158),
            (0.376, 0.216),
            0.016,
            opening_corner_radius=0.010,
            outer_corner_radius=0.018,
        ),
        "front_opening_trim",
    )
    body.visual(
        opening_trim,
        origin=Origin(xyz=(-0.055, -0.204, 0.155), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="front_opening_trim",
    )

    # Dark, hollow-looking oven liner and a supported wire rack.
    body.visual(Box((0.348, 0.248, 0.006)), origin=Origin(xyz=(-0.055, -0.050, 0.077)), material=dark_enamel, name="cavity_floor")
    body.visual(Box((0.348, 0.248, 0.006)), origin=Origin(xyz=(-0.055, -0.050, 0.233)), material=dark_enamel, name="cavity_ceiling")
    body.visual(Box((0.006, 0.248, 0.160)), origin=Origin(xyz=(-0.226, -0.050, 0.155)), material=dark_enamel, name="cavity_side_0")
    body.visual(Box((0.006, 0.248, 0.160)), origin=Origin(xyz=(0.116, -0.050, 0.155)), material=dark_enamel, name="cavity_side_1")
    body.visual(Box((0.348, 0.006, 0.160)), origin=Origin(xyz=(-0.055, 0.071, 0.155)), material=dark_enamel, name="cavity_back")
    body.visual(Cylinder(radius=0.003, length=0.242), origin=Origin(xyz=(-0.222, -0.050, 0.149), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed_steel, name="rack_side_0")
    body.visual(Cylinder(radius=0.003, length=0.242), origin=Origin(xyz=(0.112, -0.050, 0.149), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed_steel, name="rack_side_1")
    for index, y in enumerate((-0.150, -0.112, -0.074, -0.036, 0.002, 0.040)):
        body.visual(Cylinder(radius=0.0022, length=0.334), origin=Origin(xyz=(-0.055, y, 0.151), rpy=(0.0, math.pi / 2.0, 0.0)), material=brushed_steel, name=f"rack_wire_{index}")

    # Tight seam strategy and restrained vent detailing: shallow dark insets sit
    # on the top cover instead of becoming unsupported loose pieces.
    body.visual(Box((0.490, 0.004, 0.004)), origin=Origin(xyz=(0.000, -0.198, 0.291)), material=black_polymer, name="front_shadow_seam")
    for index, y in enumerate((-0.080, -0.052, -0.024, 0.004, 0.032, 0.060, 0.088)):
        body.visual(Box((0.230, 0.006, 0.004)), origin=Origin(xyz=(-0.045, y, 0.299)), material=dark_enamel, name=f"top_vent_slot_{index}")

    # Elastomer feet are molded into the lower pan.
    for index, (x, y) in enumerate(((-0.185, -0.115), (0.185, -0.115), (-0.185, 0.120), (0.185, 0.120))):
        body.visual(Cylinder(radius=0.024, length=0.018), origin=Origin(xyz=(x, y, -0.007)), material=rubber, name=f"foot_{index}")

    # Fixed bushings and dial markings on the control panel. The moving knob
    # shafts below rotate inside these bushings.
    knob_specs = (
        ("temp_knob", 0.218),
        ("mode_knob", 0.156),
        ("timer_knob", 0.094),
    )
    for name, z in knob_specs:
        body.visual(
            Cylinder(radius=0.020, length=0.010),
            origin=Origin(xyz=(0.185, -0.203, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"{name}_bushing",
        )
        body.visual(Box((0.020, 0.003, 0.003)), origin=Origin(xyz=(0.185, -0.199, z + 0.032)), material=white_mark, name=f"{name}_tick_top")
        body.visual(Box((0.003, 0.003, 0.016)), origin=Origin(xyz=(0.153, -0.199, z)), material=white_mark, name=f"{name}_tick_low")
        body.visual(Box((0.003, 0.003, 0.016)), origin=Origin(xyz=(0.217, -0.199, z)), material=white_mark, name=f"{name}_tick_high")

    # Stationary hinge pins: local overlap with the door barrels is intentional
    # and proved in run_tests as a captured pin/barrel fit.
    for index, x in enumerate((-0.200, -0.020)):
        body.visual(Cylinder(radius=0.005, length=0.068), origin=Origin(xyz=(x, -0.220, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)), material=brushed_steel, name=f"body_hinge_pin_{index}")
        body.visual(Box((0.078, 0.012, 0.014)), origin=Origin(xyz=(x, -0.202, 0.017)), material=brushed_steel, name=f"body_hinge_leaf_{index}")
        for end_index, end_x in enumerate((x - 0.034, x + 0.034)):
            body.visual(
                Box((0.008, 0.034, 0.032)),
                origin=Origin(xyz=(end_x, -0.207, 0.027)),
                material=brushed_steel,
                name=f"body_hinge_support_{index}_{end_index}",
            )

    door = model.part("door")
    door_frame = mesh_from_geometry(
        BezelGeometry(
            (0.272, 0.126),
            (0.352, 0.190),
            0.016,
            opening_corner_radius=0.010,
            outer_corner_radius=0.018,
        ),
        "door_frame",
    )
    door.visual(door_frame, origin=Origin(xyz=(0.0, -0.004, 0.131), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=brushed_steel, name="door_frame")
    door.visual(Box((0.292, 0.004, 0.142)), origin=Origin(xyz=(0.0, -0.008, 0.139)), material=smoked_glass, name="glass_pane")
    door.visual(Cylinder(radius=0.012, length=0.260), origin=Origin(xyz=(0.0, -0.046, 0.192), rpy=(0.0, math.pi / 2.0, 0.0)), material=black_polymer, name="handle_bar")
    for index, x in enumerate((-0.105, 0.105)):
        door.visual(Cylinder(radius=0.007, length=0.042), origin=Origin(xyz=(x, -0.026, 0.192), rpy=(math.pi / 2.0, 0.0, 0.0)), material=black_polymer, name=f"handle_standoff_{index}")
    for index, x in enumerate((-0.145, 0.035)):
        door.visual(Cylinder(radius=0.012, length=0.054), origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=brushed_steel, name=f"door_hinge_barrel_{index}")
        door.visual(Box((0.050, 0.008, 0.070)), origin=Origin(xyz=(x, -0.012, 0.035)), material=brushed_steel, name=f"door_hinge_leaf_{index}")

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.055, -0.220, 0.032)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=7.5, velocity=1.2, lower=0.0, upper=math.radians(105.0)),
    )

    knob_meshes = {
        "temp_knob": mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.030,
                body_style="skirted",
                top_diameter=0.038,
                skirt=KnobSkirt(0.055, 0.0055, flare=0.06, chamfer=0.001),
                grip=KnobGrip(style="fine_fluted", count=24, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=18.0),
                bore=KnobBore(style="d_shaft", diameter=0.007, flat_depth=0.0012),
                center=False,
            ),
            "temp_knob_cap",
        ),
        "mode_knob": mesh_from_geometry(
            KnobGeometry(
                0.046,
                0.029,
                body_style="skirted",
                top_diameter=0.037,
                skirt=KnobSkirt(0.053, 0.005, flare=0.05, chamfer=0.001),
                grip=KnobGrip(style="ribbed", count=18, depth=0.0009),
                indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
                bore=KnobBore(style="d_shaft", diameter=0.007, flat_depth=0.0012),
                center=False,
            ),
            "mode_knob_cap",
        ),
        "timer_knob": mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.030,
                body_style="skirted",
                top_diameter=0.038,
                skirt=KnobSkirt(0.055, 0.0055, flare=0.06, chamfer=0.001),
                grip=KnobGrip(style="fine_fluted", count=24, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=-25.0),
                bore=KnobBore(style="d_shaft", diameter=0.007, flat_depth=0.0012),
                center=False,
            ),
            "timer_knob_cap",
        ),
    }

    for name, z in knob_specs:
        knob = model.part(name)
        knob.visual(Cylinder(radius=0.0055, length=0.024), origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed_steel, name="shaft")
        knob.visual(knob_meshes[name], origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=black_polymer, name="cap")
        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(0.185, -0.209, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.18, velocity=4.0, lower=-2.35, upper=2.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("body_to_door")

    for index in range(2):
        ctx.allow_overlap(
            body,
            door,
            elem_a=f"body_hinge_pin_{index}",
            elem_b=f"door_hinge_barrel_{index}",
            reason="The fixed hinge pin is intentionally captured inside the rotating door barrel.",
        )
        ctx.expect_within(
            body,
            door,
            axes="yz",
            inner_elem=f"body_hinge_pin_{index}",
            outer_elem=f"door_hinge_barrel_{index}",
            margin=0.001,
            name=f"hinge_pin_{index}_is_concentric_with_barrel",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="x",
            elem_a=f"body_hinge_pin_{index}",
            elem_b=f"door_hinge_barrel_{index}",
            min_overlap=0.045,
            name=f"hinge_pin_{index}_retains_barrel_length",
        )

    for name in ("temp_knob", "mode_knob", "timer_knob"):
        knob = object_model.get_part(name)
        ctx.allow_overlap(
            body,
            knob,
            elem_a=f"{name}_bushing",
            elem_b="shaft",
            reason="The metal control shaft intentionally passes through the fixed panel bushing.",
        )
        ctx.expect_within(
            knob,
            body,
            axes="xz",
            inner_elem="shaft",
            outer_elem=f"{name}_bushing",
            margin=0.001,
            name=f"{name}_shaft_centered_in_bushing",
        )
        ctx.expect_overlap(
            knob,
            body,
            axes="y",
            elem_a="shaft",
            elem_b=f"{name}_bushing",
            min_overlap=0.002,
            name=f"{name}_shaft_enters_bushing",
        )

    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="front_opening_trim",
        negative_elem="door_frame",
        min_gap=0.002,
        name="closed_door_sits_proud_of_front_trim",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        elem_a="glass_pane",
        elem_b="front_opening_trim",
        min_overlap=0.110,
        name="door_glass_covers_oven_opening",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
    with ctx.pose({door_joint: math.radians(90.0)}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
    if closed_aabb is not None and open_aabb is not None:
        closed_min, closed_max = closed_aabb
        open_min, open_max = open_aabb
        ctx.check(
            "door_opens_forward_and_down",
            open_min[1] < closed_min[1] - 0.10 and open_max[2] < closed_max[2] - 0.045,
            details=f"closed={closed_aabb!r}, open={open_aabb!r}",
        )
    else:
        ctx.fail("door_open_pose_aabb_available", "Expected door frame AABBs in closed and open poses.")

    return ctx.report()


object_model = build_object_model()
