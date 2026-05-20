from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
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
    model = ArticulatedObject(name="tall_countertop_toaster_oven")

    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.61, 0.57, 1.0))
    dark_panel = model.material("black_control_panel", rgba=(0.025, 0.027, 0.028, 1.0))
    dark_cavity = model.material("dark_oven_cavity", rgba=(0.010, 0.009, 0.008, 1.0))
    warm_glass = model.material("smoked_warm_glass", rgba=(0.10, 0.075, 0.045, 0.55))
    black_trim = model.material("matte_black_trim", rgba=(0.02, 0.02, 0.018, 1.0))
    knob_mat = model.material("dark_knob_plastic", rgba=(0.075, 0.073, 0.067, 1.0))
    indicator_mat = model.material("white_indicator_marks", rgba=(0.93, 0.90, 0.82, 1.0))
    shaft_mat = model.material("polished_control_shafts", rgba=(0.80, 0.78, 0.70, 1.0))
    rubber_mat = model.material("dark_rubber_feet", rgba=(0.015, 0.014, 0.013, 1.0))

    depth = 0.43
    width = 0.34
    height = 0.42
    front_x = -depth / 2.0
    back_x = depth / 2.0
    half_w = width / 2.0

    body = model.part("oven_body")

    # Taller, narrower countertop oven shell: continuous side walls, top, bottom,
    # rear wall, and a front frame surrounding the hot cavity.
    body.visual(
        Box((depth, width, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=brushed_steel,
        name="bottom_slab",
    )
    body.visual(
        Box((depth, width, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, height - 0.0175)),
        material=brushed_steel,
        name="top_slab",
    )
    body.visual(
        Box((depth, 0.025, height)),
        origin=Origin(xyz=(0.0, -half_w + 0.0125, height / 2.0)),
        material=brushed_steel,
        name="side_wall_0",
    )
    body.visual(
        Box((depth, 0.025, height)),
        origin=Origin(xyz=(0.0, half_w - 0.0125, height / 2.0)),
        material=brushed_steel,
        name="side_wall_1",
    )
    body.visual(
        Box((0.025, width, height)),
        origin=Origin(xyz=(back_x - 0.0125, 0.0, height / 2.0)),
        material=brushed_steel,
        name="rear_wall",
    )

    # Door aperture and right-side control bank.  The bottom rail is intentionally
    # deep and tall so the front hinge line reads as a substantial appliance frame.
    front_frame_depth = 0.055
    body.visual(
        Box((front_frame_depth, 0.026, 0.320)),
        origin=Origin(xyz=(front_x + front_frame_depth / 2.0, -0.157, 0.220)),
        material=brushed_steel,
        name="front_stile_0",
    )
    body.visual(
        Box((front_frame_depth, 0.022, 0.320)),
        origin=Origin(xyz=(front_x + front_frame_depth / 2.0, 0.087, 0.220)),
        material=brushed_steel,
        name="front_divider",
    )
    body.visual(
        Box((front_frame_depth, 0.270, 0.035)),
        origin=Origin(xyz=(front_x + front_frame_depth / 2.0, -0.035, 0.365)),
        material=brushed_steel,
        name="front_top_rail",
    )
    body.visual(
        Box((0.078, 0.270, 0.078)),
        origin=Origin(xyz=(front_x + 0.018, -0.035, 0.066)),
        material=brushed_steel,
        name="deep_hinge_frame",
    )
    body.visual(
        Box((front_frame_depth, 0.079, 0.330)),
        origin=Origin(xyz=(front_x + front_frame_depth / 2.0, 0.1305, 0.220)),
        material=dark_panel,
        name="control_panel_face",
    )

    # Dark interior surfaces and wire rack glimpsed through the glass door.
    body.visual(
        Box((0.010, 0.220, 0.210)),
        origin=Origin(xyz=(0.055, -0.035, 0.220)),
        material=dark_cavity,
        name="cavity_back",
    )
    body.visual(
        Box((0.240, 0.010, 0.200)),
        origin=Origin(xyz=(-0.035, -0.145, 0.215)),
        material=dark_cavity,
        name="cavity_side_0",
    )
    body.visual(
        Box((0.240, 0.010, 0.200)),
        origin=Origin(xyz=(-0.035, 0.075, 0.215)),
        material=dark_cavity,
        name="cavity_side_1",
    )
    for index, z in enumerate((0.190, 0.214, 0.238)):
        body.visual(
            Box((0.235, 0.006, 0.006)),
            origin=Origin(xyz=(-0.025, -0.035, z)),
            material=shaft_mat,
            name=f"rack_bar_{index}",
        )
    for index, x in enumerate((-0.115, -0.045, 0.025)):
        body.visual(
            Box((0.006, 0.225, 0.005)),
            origin=Origin(xyz=(x, -0.035, 0.214)),
            material=shaft_mat,
            name=f"rack_cross_{index}",
        )

    # Side ventilation is modeled as dark recessed slots set into both side walls.
    slot_xs = (-0.110, -0.045, 0.020, 0.085)
    slot_zs = (0.292, 0.315, 0.338)
    slot_index = 0
    for side_y in (-half_w - 0.0006, half_w + 0.0006):
        for z in slot_zs:
            for x in slot_xs:
                body.visual(
                    Box((0.045, 0.002, 0.006)),
                    origin=Origin(xyz=(x, side_y, z)),
                    material=dark_cavity,
                    name=f"side_vent_{slot_index}",
                )
                slot_index += 1

    # Fixed appliance feet support the body above the countertop.
    for index, (x, y) in enumerate(
        ((-0.145, -0.120), (-0.145, 0.120), (0.145, -0.120), (0.145, 0.120))
    ):
        body.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(xyz=(x, y, -0.009)),
            material=rubber_mat,
            name=f"foot_{index}",
        )

    # Front-hinged glass door.  Its part frame is the hinge axis, so q=0 is
    # closed and positive rotation folds the free edge downward/outward.
    door = model.part("door")
    door_width = 0.270
    door_height = 0.300
    door_thickness = 0.016
    door_x_center = -door_thickness / 2.0
    door.visual(
        Box((door_thickness, door_width, 0.045)),
        origin=Origin(xyz=(door_x_center, 0.0, 0.026)),
        material=brushed_steel,
        name="door_bottom_rail",
    )
    door.visual(
        Box((door_thickness, door_width, 0.040)),
        origin=Origin(xyz=(door_x_center, 0.0, door_height - 0.020)),
        material=brushed_steel,
        name="door_top_rail",
    )
    door.visual(
        Box((door_thickness, 0.025, door_height - 0.040)),
        origin=Origin(xyz=(door_x_center, -door_width / 2.0 + 0.0125, 0.160)),
        material=brushed_steel,
        name="door_stile_0",
    )
    door.visual(
        Box((door_thickness, 0.025, door_height - 0.040)),
        origin=Origin(xyz=(door_x_center, door_width / 2.0 - 0.0125, 0.160)),
        material=brushed_steel,
        name="door_stile_1",
    )
    door.visual(
        Box((0.004, 0.208, 0.190)),
        origin=Origin(xyz=(-0.017, 0.0, 0.165)),
        material=warm_glass,
        name="glass_window",
    )
    door.visual(
        Cylinder(radius=0.010, length=door_width - 0.020),
        origin=Origin(xyz=(-0.006, 0.0, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_sleeve",
    )
    door.visual(
        Cylinder(radius=0.0075, length=0.205),
        origin=Origin(xyz=(-0.046, 0.0, 0.240), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black_trim,
        name="front_handle",
    )
    for index, y in enumerate((-0.078, 0.078)):
        door.visual(
            Box((0.032, 0.012, 0.026)),
            origin=Origin(xyz=(-0.030, y, 0.240)),
            material=black_trim,
            name=f"handle_standoff_{index}",
        )

    hinge_origin = Origin(xyz=(front_x - 0.025, -0.035, 0.078))
    door_hinge = model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=hinge_origin,
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    # Three separate rotating controls sit on visible metal shafts.
    knob_meshes = []
    for index in range(3):
        knob_geom = KnobGeometry(
            0.046,
            0.028,
            body_style="skirted",
            top_diameter=0.036,
            skirt=KnobSkirt(0.052, 0.006, flare=0.06, chamfer=0.0012),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
            bore=KnobBore(style="d_shaft", diameter=0.007, flat_depth=0.001),
            center=False,
        )
        knob_meshes.append(mesh_from_geometry(knob_geom, f"control_knob_{index}"))

    knob_positions = (0.305, 0.220, 0.135)
    for index, z in enumerate(knob_positions):
        y = 0.131
        shaft_center_x = front_x - 0.006
        shaft_end_x = front_x - 0.013
        body.visual(
            Cylinder(radius=0.007, length=0.014),
            origin=Origin(xyz=(shaft_center_x, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=shaft_mat,
            name=f"control_shaft_{index}",
        )
        body.visual(
            Box((0.003, 0.056, 0.003)),
            origin=Origin(xyz=(front_x + 0.001, y, z - 0.035), rpy=(0.0, 0.0, 0.0)),
            material=indicator_mat,
            name=f"control_tick_arc_{index}",
        )

        knob = model.part(f"knob_{index}")
        knob.visual(
            knob_meshes[index],
            origin=Origin(rpy=(0.0, -pi / 2.0, 0.0)),
            material=knob_mat,
            name="knob_cap",
        )
        knob.visual(
            Box((0.0018, 0.006, 0.024)),
            origin=Origin(xyz=(-0.029, 0.0, 0.007)),
            material=indicator_mat,
            name="pointer_mark",
        )
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(shaft_end_x, y, z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=5.0, lower=-2.35, upper=2.35),
        )

    model.meta["primary_mechanisms"] = {
        "door": door_hinge.name,
        "knobs": [f"body_to_knob_{i}" for i in range(3)],
    }
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("oven_body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="x",
            min_gap=0.001,
            max_gap=0.020,
            positive_elem="deep_hinge_frame",
            negative_elem="door_bottom_rail",
            name="closed door sits just proud of the deep hinge frame",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            min_overlap=0.17,
            elem_a="glass_window",
            elem_b="cavity_back",
            name="glass covers the oven opening",
        )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.15}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door folds downward and outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] < closed_aabb[1][2] - 0.12
        and open_aabb[0][0] < closed_aabb[0][0] - 0.08,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for index in range(3):
        knob = object_model.get_part(f"knob_{index}")
        joint = object_model.get_articulation(f"body_to_knob_{index}")
        ctx.expect_contact(
            knob,
            body,
            elem_a="knob_cap",
            elem_b=f"control_shaft_{index}",
            contact_tol=0.004,
            name=f"knob {index} is shaft mounted",
        )
        with ctx.pose({joint: 0.8}):
            ctx.expect_contact(
                knob,
                body,
                elem_a="knob_cap",
                elem_b=f"control_shaft_{index}",
                contact_tol=0.004,
                name=f"knob {index} remains on its shaft while rotating",
            )

    return ctx.report()


object_model = build_object_model()
