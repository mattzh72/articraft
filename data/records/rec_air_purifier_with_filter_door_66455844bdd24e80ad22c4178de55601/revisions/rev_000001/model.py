from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_dual_bay_air_purifier")

    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.87, 0.84, 1.0))
    satin_white = model.material("satin_white_door", rgba=(0.94, 0.95, 0.92, 1.0))
    dark_filter = model.material("dark_filter_media", rgba=(0.035, 0.038, 0.042, 1.0))
    graphite = model.material("graphite_trim", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("black_rubber_gasket", rgba=(0.015, 0.015, 0.014, 1.0))
    hinge_metal = model.material("dull_hinge_metal", rgba=(0.48, 0.50, 0.49, 1.0))

    body = model.part("body")
    width = 1.18
    depth = 0.46
    height = 0.64
    front_y = -depth / 2.0 - 0.010
    rear_shell_depth = 0.38

    # The main appliance case is a wide, low rectangular shell with a shallow
    # stepped front frame.  The front frame surrounds two filter bay openings.
    body.visual(
        Box((width, rear_shell_depth, height)),
        origin=Origin(xyz=(0.0, 0.02, height / 2.0)),
        material=warm_white,
        name="rear_shell",
    )
    body.visual(
        Box((width, 0.070, 0.085)),
        origin=Origin(xyz=(0.0, front_y + 0.035, height - 0.0425)),
        material=warm_white,
        name="top_rail",
    )
    body.visual(
        Box((width, 0.070, 0.085)),
        origin=Origin(xyz=(0.0, front_y + 0.035, 0.0425)),
        material=warm_white,
        name="bottom_rail",
    )
    for name, x in (("side_rail_0", -width / 2.0 + 0.0375), ("side_rail_1", width / 2.0 - 0.0375)):
        body.visual(
            Box((0.075, 0.070, height)),
            origin=Origin(xyz=(x, front_y + 0.035, height / 2.0)),
            material=warm_white,
            name=name,
        )
    body.visual(
        Box((0.065, 0.070, 0.505)),
        origin=Origin(xyz=(0.0, front_y + 0.035, 0.300)),
        material=warm_white,
        name="center_mullion",
    )

    # Pleated dark filter media is visible whenever either service door opens.
    # Each filter face is a slotted mesh set back behind the front frame.
    filter_panel = SlotPatternPanelGeometry(
        (0.415, 0.410),
        0.008,
        slot_size=(0.022, 0.006),
        pitch=(0.032, 0.016),
        frame=0.012,
        corner_radius=0.006,
        slot_angle_deg=0.0,
        stagger=True,
    )
    body.visual(
        mesh_from_geometry(filter_panel, "filter_media_0"),
        origin=Origin(xyz=(-0.275, -0.176, 0.305), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_filter,
        name="filter_media_0",
    )
    body.visual(
        Box((0.445, 0.012, 0.440)),
        origin=Origin(xyz=(-0.275, -0.170, 0.305)),
        material=graphite,
        name="filter_shadow_0",
    )
    body.visual(
        mesh_from_geometry(filter_panel, "filter_media_1"),
        origin=Origin(xyz=(0.275, -0.176, 0.305), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_filter,
        name="filter_media_1",
    )
    body.visual(
        Box((0.445, 0.012, 0.440)),
        origin=Origin(xyz=(0.275, -0.170, 0.305)),
        material=graphite,
        name="filter_shadow_1",
    )

    top_grille = SlotPatternPanelGeometry(
        (0.82, 0.18),
        0.006,
        slot_size=(0.055, 0.010),
        pitch=(0.075, 0.024),
        frame=0.018,
        corner_radius=0.010,
        stagger=False,
    )
    body.visual(
        mesh_from_geometry(top_grille, "top_outlet_grille"),
        origin=Origin(xyz=(0.0, 0.050, height + 0.003)),
        material=graphite,
        name="top_outlet_grille",
    )

    # Small fixed hinge plates on the outer sides of the two bay frames give the
    # revolute doors a visible mount at the front face.
    for name, x in (("hinge_plate_0", -0.546), ("hinge_plate_1", 0.546)):
        body.visual(
            Box((0.020, 0.006, 0.505)),
            origin=Origin(xyz=(x, front_y - 0.003, 0.305)),
            material=hinge_metal,
            name=name,
        )

    # Low feet keep the appliance slightly off the floor while staying connected
    # to the body by contact with the bottom shell.
    for i, (x, y) in enumerate(((-0.46, -0.10), (0.46, -0.10), (-0.46, 0.15), (0.46, 0.15))):
        body.visual(
            Box((0.130, 0.070, 0.026)),
            origin=Origin(xyz=(x, y, -0.013)),
            material=graphite,
            name=f"foot_{i}",
        )

    door_width = 0.490
    door_height = 0.500
    door_thickness = 0.032
    hinge_y = front_y - 0.001
    hinge_z = 0.305

    def add_filter_door(
        name: str,
        *,
        extends_sign: float,
        hinge_x: float,
        axis_z: float,
        handle_x: float,
    ):
        door = model.part(name)
        door.visual(
            Box((door_width, door_thickness, door_height)),
            origin=Origin(xyz=(extends_sign * door_width / 2.0, -0.018, 0.0)),
            material=satin_white,
            name="panel",
        )
        # A proud perimeter lip and a black compression gasket make the door read
        # as a sealed service door rather than a decorative flat plate.
        lip_x = extends_sign * door_width / 2.0
        door.visual(
            Box((door_width - 0.035, 0.010, 0.026)),
            origin=Origin(xyz=(lip_x, -0.0385, door_height / 2.0 - 0.023)),
            material=warm_white,
            name="top_lip",
        )
        door.visual(
            Box((door_width - 0.035, 0.010, 0.026)),
            origin=Origin(xyz=(lip_x, -0.0385, -door_height / 2.0 + 0.023)),
            material=warm_white,
            name="bottom_lip",
        )
        for edge_name, x in (("hinge_lip", extends_sign * 0.018), ("free_lip", extends_sign * (door_width - 0.018))):
            door.visual(
                Box((0.026, 0.010, door_height - 0.035)),
                origin=Origin(xyz=(x, -0.0385, 0.0)),
                material=warm_white,
                name=edge_name,
            )
        door.visual(
            Box((door_width - 0.055, 0.004, door_height - 0.055)),
            origin=Origin(xyz=(lip_x, -0.001, 0.0)),
            material=rubber,
            name="rear_gasket",
        )
        door.visual(
            Box((0.020, 0.010, 0.320)),
            origin=Origin(xyz=(handle_x, -0.0385, 0.0)),
            material=graphite,
            name="vertical_pull",
        )
        door.visual(
            Cylinder(radius=0.012, length=door_height),
            origin=Origin(xyz=(0.0, -0.022, 0.0)),
            material=hinge_metal,
            name="hinge_barrel",
        )
        door.visual(
            Box((0.028, 0.010, door_height)),
            origin=Origin(xyz=(extends_sign * 0.014, -0.021, 0.0)),
            material=hinge_metal,
            name="hinge_leaf",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.75),
        )
        return door

    add_filter_door(
        "filter_door_0",
        extends_sign=1.0,
        hinge_x=-0.530,
        axis_z=-1.0,
        handle_x=door_width - 0.045,
    )
    add_filter_door(
        "filter_door_1",
        extends_sign=-1.0,
        hinge_x=0.530,
        axis_z=1.0,
        handle_x=-(door_width - 0.045),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door_0 = object_model.get_part("filter_door_0")
    door_1 = object_model.get_part("filter_door_1")
    hinge_0 = object_model.get_articulation("body_to_filter_door_0")
    hinge_1 = object_model.get_articulation("body_to_filter_door_1")

    ctx.check(
        "dual hinged filter doors present",
        all(item is not None for item in (body, door_0, door_1, hinge_0, hinge_1)),
        "Expected a purifier body, two separate filter doors, and two revolute hinges.",
    )
    if body is None or door_0 is None or door_1 is None or hinge_0 is None or hinge_1 is None:
        return ctx.report()

    ctx.expect_overlap(
        door_0,
        body,
        axes="xz",
        elem_a="panel",
        elem_b="filter_shadow_0",
        min_overlap=0.35,
        name="door_0 covers left filter bay",
    )
    ctx.expect_overlap(
        door_1,
        body,
        axes="xz",
        elem_a="panel",
        elem_b="filter_shadow_1",
        min_overlap=0.35,
        name="door_1 covers right filter bay",
    )
    ctx.expect_gap(
        body,
        door_0,
        axis="y",
        positive_elem="bottom_rail",
        negative_elem="rear_gasket",
        max_gap=0.001,
        max_penetration=0.001,
        name="door_0 gasket seals against frame",
    )
    ctx.expect_gap(
        body,
        door_1,
        axis="y",
        positive_elem="bottom_rail",
        negative_elem="rear_gasket",
        max_gap=0.001,
        max_penetration=0.001,
        name="door_1 gasket seals against frame",
    )

    closed_0 = ctx.part_element_world_aabb(door_0, elem="panel")
    closed_1 = ctx.part_element_world_aabb(door_1, elem="panel")
    with ctx.pose({hinge_0: 1.20, hinge_1: 1.20}):
        open_0 = ctx.part_element_world_aabb(door_0, elem="panel")
        open_1 = ctx.part_element_world_aabb(door_1, elem="panel")

    opens_outward = (
        closed_0 is not None
        and closed_1 is not None
        and open_0 is not None
        and open_1 is not None
        and float(open_0[0][1]) < float(closed_0[0][1]) - 0.15
        and float(open_1[0][1]) < float(closed_1[0][1]) - 0.15
    )
    ctx.check(
        "both filter doors swing outward",
        opens_outward,
        details=f"closed_0={closed_0}, open_0={open_0}, closed_1={closed_1}, open_1={open_1}",
    )

    return ctx.report()


object_model = build_object_model()
