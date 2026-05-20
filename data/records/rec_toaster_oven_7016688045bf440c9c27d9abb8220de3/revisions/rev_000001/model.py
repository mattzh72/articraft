from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_oven")

    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_enamel = model.material("dark_enamel", rgba=(0.035, 0.038, 0.040, 1.0))
    black = model.material("black_plastic", rgba=(0.01, 0.01, 0.012, 1.0))
    warm_glass = model.material("smoky_glass", rgba=(0.30, 0.46, 0.56, 0.38))
    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.82, 1.0))
    heat = model.material("heating_element", rgba=(1.0, 0.30, 0.05, 1.0))
    white = model.material("white_marking", rgba=(0.94, 0.94, 0.88, 1.0))

    body = model.part("body")

    # Countertop-oven shell: 58 cm wide, 42 cm deep, 32 cm tall above small feet.
    body.visual(Box((0.58, 0.42, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0525)), material=brushed_steel, name="bottom_pan")
    body.visual(Box((0.58, 0.42, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.340)), material=brushed_steel, name="top_skin")
    body.visual(Box((0.030, 0.42, 0.29)), origin=Origin(xyz=(-0.275, 0.0, 0.1975)), material=brushed_steel, name="side_wall_0")
    body.visual(Box((0.030, 0.42, 0.29)), origin=Origin(xyz=(0.275, 0.0, 0.1975)), material=brushed_steel, name="side_wall_1")
    body.visual(Box((0.58, 0.030, 0.29)), origin=Origin(xyz=(0.0, 0.195, 0.1975)), material=brushed_steel, name="rear_wall")

    # Front surround leaves a broad oven opening and a right-hand control tower.
    body.visual(Box((0.425, 0.020, 0.065)), origin=Origin(xyz=(-0.070, -0.217, 0.3275)), material=brushed_steel, name="front_top_rail")
    body.visual(Box((0.425, 0.020, 0.070)), origin=Origin(xyz=(-0.070, -0.217, 0.062)), material=brushed_steel, name="front_bottom_rail")
    body.visual(Box((0.040, 0.020, 0.290)), origin=Origin(xyz=(-0.275, -0.217, 0.1975)), material=brushed_steel, name="front_side_rail")
    body.visual(Box((0.030, 0.020, 0.275)), origin=Origin(xyz=(0.130, -0.217, 0.1975)), material=brushed_steel, name="control_divider")
    body.visual(Box((0.130, 0.022, 0.285)), origin=Origin(xyz=(0.215, -0.218, 0.1975)), material=dark_enamel, name="control_panel")

    # Dark inner liner and visible baking hardware behind the glass door.
    body.visual(Box((0.390, 0.010, 0.230)), origin=Origin(xyz=(-0.075, 0.165, 0.195)), material=dark_enamel, name="cavity_back")
    body.visual(Box((0.405, 0.260, 0.010)), origin=Origin(xyz=(-0.075, -0.040, 0.092)), material=dark_enamel, name="cavity_floor")
    for index, z in enumerate((0.117, 0.283)):
        body.visual(
            Cylinder(radius=0.006, length=0.390),
            origin=Origin(xyz=(-0.075, -0.060, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=heat,
            name=f"heater_{index}",
        )
    for index, y in enumerate((-0.120, -0.065, -0.010, 0.045)):
        body.visual(
            Cylinder(radius=0.0022, length=0.400),
            origin=Origin(xyz=(-0.075, y, 0.185), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name=f"rack_rod_{index}",
        )

    # Four squat appliance feet are part of the fixed body and contact the bottom pan.
    for index, (x, y) in enumerate(((-0.220, -0.150), (0.220, -0.150), (-0.220, 0.150), (0.220, 0.150))):
        body.visual(Cylinder(radius=0.022, length=0.035), origin=Origin(xyz=(x, y, 0.0175)), material=black, name=f"foot_{index}")

    # Lower front hinge knuckles on the fixed body; the moving door carries the center knuckle.
    for index, x in enumerate((-0.250, 0.065)):
        body.visual(Box((0.090, 0.012, 0.014)), origin=Origin(xyz=(x, -0.229, 0.067)), material=black, name=f"door_hinge_leaf_{index}")
        body.visual(
            Cylinder(radius=0.007, length=0.090),
            origin=Origin(xyz=(x, -0.240, 0.067), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"door_hinge_knuckle_{index}",
        )
    body.visual(
        Cylinder(radius=0.003, length=0.405),
        origin=Origin(xyz=(-0.075, -0.240, 0.067), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="door_hinge_pin",
    )

    # A top vent grille is partially covered by the hinged steam flap.
    body.visual(Box((0.215, 0.085, 0.004)), origin=Origin(xyz=(0.0, 0.085, 0.357)), material=dark_enamel, name="vent_recess")
    for index, y in enumerate((0.055, 0.075, 0.095, 0.115)):
        body.visual(Box((0.170, 0.007, 0.003)), origin=Origin(xyz=(0.0, y, 0.359)), material=black, name=f"vent_slot_{index}")
    for index, x in enumerate((-0.140, 0.140)):
        body.visual(Box((0.048, 0.012, 0.010)), origin=Origin(xyz=(x, 0.153, 0.357)), material=black, name=f"vent_hinge_leaf_{index}")
        body.visual(
            Cylinder(radius=0.006, length=0.045),
            origin=Origin(xyz=(x, 0.145, 0.363), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"vent_hinge_knuckle_{index}",
        )
    body.visual(
        Cylinder(radius=0.0025, length=0.335),
        origin=Origin(xyz=(0.0, 0.145, 0.363), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="vent_hinge_pin",
    )

    # Tick marks around the control knobs, seated on the fixed control tower.
    for knob_index, zc in enumerate((0.255, 0.155)):
        for tick_index, (dx, dz, sx, sz) in enumerate(((0.0, 0.038, 0.004, 0.014), (0.0, -0.038, 0.004, 0.014), (-0.038, 0.0, 0.014, 0.004), (0.038, 0.0, 0.014, 0.004))):
            body.visual(
                Box((sx, 0.0020, sz)),
                origin=Origin(xyz=(0.215 + dx, -0.2285, zc + dz)),
                material=white,
                name=f"knob_{knob_index}_tick_{tick_index}",
            )

    door = model.part("door")
    door_frame = BezelGeometry(
        opening_size=(0.315, 0.200),
        outer_size=(0.430, 0.290),
        depth=0.018,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.014,
        outer_corner_radius=0.025,
    )
    door.visual(
        mesh_from_geometry(door_frame, "door_frame"),
        origin=Origin(xyz=(0.0, -0.008, 0.155), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="door_frame",
    )
    door.visual(Box((0.335, 0.004, 0.218)), origin=Origin(xyz=(0.0, -0.014, 0.155)), material=warm_glass, name="glass_pane")
    door.visual(Box((0.180, 0.008, 0.020)), origin=Origin(xyz=(0.0, -0.003, 0.012)), material=black, name="door_hinge_leaf")
    door.visual(
        Cylinder(radius=0.008, length=0.175),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="door_hinge_knuckle",
    )
    door.visual(Cylinder(radius=0.012, length=0.300), origin=Origin(xyz=(0.0, -0.040, 0.245), rpy=(0.0, math.pi / 2.0, 0.0)), material=chrome, name="handle_bar")
    for index, x in enumerate((-0.110, 0.110)):
        door.visual(Box((0.026, 0.032, 0.026)), origin=Origin(xyz=(x, -0.026, 0.245)), material=chrome, name=f"handle_standoff_{index}")

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.075, -0.240, 0.067)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.55),
    )

    for index, zc in enumerate((0.255, 0.155)):
        knob = model.part(f"knob_{index}")
        knob.visual(Cylinder(radius=0.007, length=0.035), origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=chrome, name="short_shaft")
        knob.visual(Cylinder(radius=0.030, length=0.007), origin=Origin(xyz=(0.0, -0.0035, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed_steel, name="knob_skirt")
        knob.visual(Cylinder(radius=0.023, length=0.024), origin=Origin(xyz=(0.0, -0.019, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brushed_steel, name="knob_cap")
        knob.visual(Box((0.005, 0.0014, 0.023)), origin=Origin(xyz=(0.0, -0.032, 0.009)), material=white, name="pointer_line")
        model.articulation(
            f"knob_{index}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(0.215, -0.229, zc)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=8.0),
        )

    steam_flap = model.part("steam_flap")
    steam_flap.visual(Box((0.220, 0.110, 0.008)), origin=Origin(xyz=(0.0, -0.055, -0.004)), material=brushed_steel, name="flap_plate")
    steam_flap.visual(
        Cylinder(radius=0.006, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="flap_hinge_knuckle",
    )
    steam_flap.visual(Box((0.185, 0.010, 0.006)), origin=Origin(xyz=(0.0, -0.105, 0.001)), material=black, name="flap_lip")
    for index, y in enumerate((-0.030, -0.050, -0.070)):
        steam_flap.visual(Box((0.150, 0.006, 0.0020)), origin=Origin(xyz=(0.0, y, 0.0005)), material=black, name=f"flap_slot_{index}")

    model.articulation(
        "flap_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_flap,
        origin=Origin(xyz=(0.0, 0.145, 0.363)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    knob_part_0 = object_model.get_part("knob_0")
    knob_part_1 = object_model.get_part("knob_1")
    steam_flap = object_model.get_part("steam_flap")
    door_hinge = object_model.get_articulation("door_hinge")
    flap_hinge = object_model.get_articulation("flap_hinge")
    knob_0 = object_model.get_articulation("knob_0_spin")
    knob_1 = object_model.get_articulation("knob_1_spin")

    ctx.allow_overlap(
        body,
        door,
        elem_a="door_hinge_pin",
        elem_b="door_hinge_knuckle",
        reason="A small metal pin is intentionally captured inside the rotating lower door hinge barrel.",
    )
    ctx.allow_overlap(
        body,
        steam_flap,
        elem_a="vent_hinge_pin",
        elem_b="flap_hinge_knuckle",
        reason="The top steam-flap hinge uses a retained pin passing through the flap knuckle.",
    )
    for knob_part in (knob_part_0, knob_part_1):
        ctx.allow_overlap(
            body,
            knob_part,
            elem_a="control_panel",
            elem_b="short_shaft",
            reason="The rotary control shaft is intentionally seated a few millimeters into the panel bushing.",
        )

    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "countertop baking scale",
        body_aabb is not None
        and 0.54 <= (body_aabb[1][0] - body_aabb[0][0]) <= 0.62
        and 0.38 <= (body_aabb[1][1] - body_aabb[0][1]) <= 0.46
        and 0.32 <= (body_aabb[1][2] - body_aabb[0][2]) <= 0.39,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "two continuous rotary controls",
        knob_0.articulation_type == ArticulationType.CONTINUOUS
        and knob_1.articulation_type == ArticulationType.CONTINUOUS
        and knob_0.axis == (0.0, -1.0, 0.0)
        and knob_1.axis == (0.0, -1.0, 0.0),
        details=f"knob_0={knob_0.articulation_type}/{knob_0.axis}, knob_1={knob_1.articulation_type}/{knob_1.axis}",
    )
    ctx.expect_overlap(
        body,
        door,
        axes="x",
        min_overlap=0.14,
        elem_a="door_hinge_pin",
        elem_b="door_hinge_knuckle",
        name="door hinge pin spans the moving hinge knuckle",
    )
    ctx.expect_within(
        body,
        door,
        axes="yz",
        inner_elem="door_hinge_pin",
        outer_elem="door_hinge_knuckle",
        margin=0.0,
        name="door hinge pin is concentric inside knuckle",
    )
    ctx.expect_overlap(
        body,
        steam_flap,
        axes="x",
        min_overlap=0.07,
        elem_a="vent_hinge_pin",
        elem_b="flap_hinge_knuckle",
        name="vent hinge pin spans the flap knuckle",
    )
    ctx.expect_within(
        body,
        steam_flap,
        axes="yz",
        inner_elem="vent_hinge_pin",
        outer_elem="flap_hinge_knuckle",
        margin=0.0,
        name="vent hinge pin is concentric inside knuckle",
    )
    for index, knob_part in enumerate((knob_part_0, knob_part_1)):
        ctx.expect_overlap(
            body,
            knob_part,
            axes="y",
            min_overlap=0.002,
            elem_a="control_panel",
            elem_b="short_shaft",
            name=f"knob {index} shaft remains inserted in panel",
        )
        ctx.expect_within(
            knob_part,
            body,
            axes="xz",
            inner_elem="short_shaft",
            outer_elem="control_panel",
            margin=0.0,
            name=f"knob {index} shaft is centered on control panel",
        )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        min_gap=0.004,
        max_gap=0.030,
        positive_elem="front_top_rail",
        negative_elem="door_frame",
        name="closed glass door sits just proud of front face",
    )
    ctx.expect_gap(
        steam_flap,
        body,
        axis="z",
        min_gap=-0.001,
        max_gap=0.003,
        positive_elem="flap_plate",
        negative_elem="top_skin",
        name="steam flap rests on top skin",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.35}):
        lowered_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door rotates downward and outward",
        closed_door_aabb is not None
        and lowered_door_aabb is not None
        and lowered_door_aabb[1][2] < closed_door_aabb[1][2] - 0.10
        and lowered_door_aabb[0][1] < closed_door_aabb[0][1] - 0.10,
        details=f"closed={closed_door_aabb}, lowered={lowered_door_aabb}",
    )

    closed_flap_aabb = ctx.part_world_aabb(steam_flap)
    with ctx.pose({flap_hinge: 0.95}):
        raised_flap_aabb = ctx.part_world_aabb(steam_flap)
    ctx.check(
        "steam flap rotates upward",
        closed_flap_aabb is not None
        and raised_flap_aabb is not None
        and raised_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.055,
        details=f"closed={closed_flap_aabb}, raised={raised_flap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
