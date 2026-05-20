from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_oven")

    stainless = Material("brushed_stainless", rgba=(0.72, 0.70, 0.66, 1.0))
    dark = Material("black_enamel", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_panel = Material("dark_control_panel", rgba=(0.055, 0.055, 0.060, 1.0))
    glass = Material("smoked_glass", rgba=(0.06, 0.09, 0.10, 0.42))
    chrome = Material("chrome", rgba=(0.88, 0.87, 0.82, 1.0))
    warm = Material("heating_element", rgba=(1.0, 0.28, 0.05, 1.0))
    rubber = Material("black_rubber", rgba=(0.005, 0.005, 0.005, 1.0))
    white = Material("white_markings", rgba=(0.92, 0.92, 0.86, 1.0))

    body = model.part("body")

    width = 0.48
    depth = 0.34
    height = 0.285
    bottom_z = 0.035
    wall = 0.018
    front_x = -depth / 2.0
    rear_x = depth / 2.0
    top_z = bottom_z + height

    # Main five-sided oven shell.  The front is intentionally open, with a
    # separate face frame and control panel mounted around the oven cavity.
    body.visual(
        Box((depth, width, wall)),
        origin=Origin(xyz=(0.0, 0.0, bottom_z + wall / 2.0)),
        material=stainless,
        name="bottom_shell",
    )
    body.visual(
        Box((depth, width, wall)),
        origin=Origin(xyz=(0.0, 0.0, top_z - wall / 2.0)),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall / 2.0, bottom_z + height / 2.0)),
        material=stainless,
        name="side_wall_0",
    )
    body.visual(
        Box((depth, wall, height)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall / 2.0, bottom_z + height / 2.0)),
        material=stainless,
        name="side_wall_1",
    )
    body.visual(
        Box((wall, width, height)),
        origin=Origin(xyz=(rear_x - wall / 2.0, 0.0, bottom_z + height / 2.0)),
        material=stainless,
        name="rear_wall",
    )

    oven_y_min = -0.215
    oven_y_max = 0.105
    oven_center_y = (oven_y_min + oven_y_max) / 2.0
    oven_width = oven_y_max - oven_y_min
    opening_bottom = bottom_z + 0.045
    opening_top = bottom_z + 0.225
    opening_height = opening_top - opening_bottom
    face_depth = 0.030
    face_x = front_x + face_depth / 2.0

    # Black front frame around the opening; the control panel closes the right
    # side of the face and carries the three dials.
    body.visual(
        Box((face_depth, oven_width + 0.050, 0.026)),
        origin=Origin(xyz=(face_x, oven_center_y, opening_top + 0.013)),
        material=dark,
        name="front_top_rail",
    )
    body.visual(
        Box((face_depth, oven_width + 0.050, 0.025)),
        origin=Origin(xyz=(face_x, oven_center_y, opening_bottom - 0.0125)),
        material=dark,
        name="front_bottom_rail",
    )
    body.visual(
        Box((face_depth, 0.026, opening_height + 0.050)),
        origin=Origin(xyz=(face_x, oven_y_min - 0.013, (opening_bottom + opening_top) / 2.0)),
        material=dark,
        name="front_side_rail_0",
    )
    body.visual(
        Box((face_depth, 0.026, opening_height + 0.050)),
        origin=Origin(xyz=(face_x, oven_y_max + 0.013, (opening_bottom + opening_top) / 2.0)),
        material=dark,
        name="front_side_rail_1",
    )
    body.visual(
        Box((face_depth, 0.105, height - 0.028)),
        origin=Origin(xyz=(face_x, 0.177, bottom_z + height / 2.0)),
        material=dark_panel,
        name="control_panel",
    )

    # Dark interior liner and two glowing heating elements behind the door.
    body.visual(
        Box((0.006, oven_width - 0.020, opening_height - 0.015)),
        origin=Origin(xyz=(rear_x - wall - 0.003, oven_center_y, (opening_bottom + opening_top) / 2.0)),
        material=dark,
        name="cavity_back",
    )
    body.visual(
        Box((0.300, oven_width + 0.026, 0.006)),
        origin=Origin(xyz=(-0.005, oven_center_y, opening_bottom + 0.012)),
        material=dark,
        name="cavity_floor",
    )
    body.visual(
        Box((0.300, oven_width + 0.026, 0.006)),
        origin=Origin(xyz=(-0.005, oven_center_y, opening_top - 0.012)),
        material=dark,
        name="cavity_ceiling",
    )
    for i, x in enumerate((-0.085, 0.055)):
        body.visual(
            Cylinder(radius=0.0045, length=oven_width + 0.026),
            origin=Origin(
                xyz=(x, oven_center_y, opening_top - 0.040),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=warm,
            name=f"upper_element_{i}",
        )
        body.visual(
            Cylinder(radius=0.0045, length=oven_width + 0.026),
            origin=Origin(
                xyz=(x, oven_center_y, opening_bottom + 0.045),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=warm,
            name=f"lower_element_{i}",
        )

    # Simple chrome rack: longitudinal rails with transverse rods, all embedded
    # slightly into the side liner so the rack reads as mounted.
    rack_z = opening_bottom + 0.090
    for j, y in enumerate((oven_y_min + 0.006, oven_y_max + 0.004)):
        body.visual(
            Cylinder(radius=0.0022, length=0.205),
            origin=Origin(xyz=(-0.015, y, rack_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name=f"rack_side_rail_{j}",
        )
    for j, x in enumerate((-0.105, -0.055, -0.005, 0.045, 0.095)):
        body.visual(
            Cylinder(radius=0.0018, length=oven_width + 0.020),
            origin=Origin(xyz=(x, oven_center_y, rack_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"rack_cross_rod_{j}",
        )

    # Low rubber feet under the shell.
    for j, y in enumerate((-0.175, 0.175)):
        for i, x in enumerate((-0.105, 0.105)):
            body.visual(
                Cylinder(radius=0.022, length=0.035),
                origin=Origin(xyz=(x, y, 0.0175)),
                material=rubber,
                name=f"foot_{j}_{i}",
            )

    # Drop-down door.  Its part frame is the lower horizontal hinge line.
    door = model.part("door")
    door_width = 0.350
    door_height = 0.195
    door_thick = 0.014
    hinge_x = front_x - 0.005
    hinge_y = oven_center_y
    hinge_z = opening_bottom - 0.002

    # Door frame rails and smoky inset glass.  The glass is seated behind the
    # rails, so it has a small mechanical capture instead of floating.
    door.visual(
        Box((door_thick, door_width, 0.025)),
        origin=Origin(xyz=(-0.002, 0.0, door_height - 0.0125)),
        material=dark,
        name="door_top_rail",
    )
    door.visual(
        Box((door_thick, door_width, 0.025)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0125)),
        material=dark,
        name="door_bottom_rail",
    )
    door.visual(
        Box((door_thick, 0.026, door_height)),
        origin=Origin(xyz=(-0.002, -door_width / 2.0 + 0.013, door_height / 2.0)),
        material=dark,
        name="door_side_rail_0",
    )
    door.visual(
        Box((door_thick, 0.026, door_height)),
        origin=Origin(xyz=(-0.002, door_width / 2.0 - 0.013, door_height / 2.0)),
        material=dark,
        name="door_side_rail_1",
    )
    door.visual(
        Box((0.004, door_width - 0.040, door_height - 0.040)),
        origin=Origin(xyz=(-0.0075, 0.0, door_height / 2.0)),
        material=glass,
        name="glass_pane",
    )
    door.visual(
        Cylinder(radius=0.006, length=door_width - 0.030),
        origin=Origin(xyz=(-0.001, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    handle_z = door_height - 0.055
    door.visual(
        Cylinder(radius=0.008, length=0.245),
        origin=Origin(xyz=(-0.047, 0.0, handle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_bar",
    )
    for j, y in enumerate((-0.105, 0.105)):
        door.visual(
            Cylinder(radius=0.0048, length=0.038),
            origin=Origin(xyz=(-0.028, y, handle_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name=f"handle_standoff_{j}",
        )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.65),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.050,
            0.022,
            body_style="skirted",
            top_diameter=0.040,
            skirt=KnobSkirt(0.056, 0.0055, flare=0.05, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=20, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0007),
            center=False,
        ),
        "fluted_toaster_knob",
    )
    knob_x = front_x
    knob_y = 0.177
    knob_zs = (bottom_z + 0.218, bottom_z + 0.150, bottom_z + 0.082)
    for idx, z in enumerate(knob_zs):
        knob = model.part(f"knob_{idx}")
        knob.visual(
            Cylinder(radius=0.0065, length=0.012),
            origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
            material=chrome,
            name="shaft",
        )
        knob.visual(
            knob_mesh,
            origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
            material=dark,
            name="dial_cap",
        )
        model.articulation(
            f"control_to_knob_{idx}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, knob_y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=8.0),
        )

        # Fixed printed tick mark beside each knob on the panel.
        body.visual(
            Box((0.0015, 0.020, 0.003)),
            origin=Origin(xyz=(front_x - 0.00075, knob_y - 0.034, z + 0.023)),
            material=white,
            name=f"knob_tick_{idx}",
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_joint = object_model.get_articulation("body_to_door")
    ctx.expect_overlap(
        door,
        body,
        axes="y",
        elem_a="hinge_barrel",
        elem_b="front_bottom_rail",
        min_overlap=0.25,
        name="door hinge spans the front opening",
    )
    ctx.expect_gap(
        body,
        door,
        axis="x",
        positive_elem="front_side_rail_0",
        negative_elem="door_side_rail_0",
        min_gap=-0.001,
        max_gap=0.020,
        name="closed door is seated at the front frame",
    )
    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.45}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door rotates downward and outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][0] < closed_aabb[0][0] - 0.08
        and open_aabb[0][2] < closed_aabb[0][2] + 0.005,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for idx in range(3):
        knob = object_model.get_part(f"knob_{idx}")
        joint = object_model.get_articulation(f"control_to_knob_{idx}")
        ctx.expect_gap(
            body,
            knob,
            axis="x",
            positive_elem="control_panel",
            negative_elem="shaft",
            min_gap=-0.002,
            max_gap=0.006,
            name=f"knob_{idx} shaft meets the control panel",
        )
        ctx.check(
            f"knob_{idx} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=str(joint.articulation_type),
        )

    return ctx.report()


object_model = build_object_model()
