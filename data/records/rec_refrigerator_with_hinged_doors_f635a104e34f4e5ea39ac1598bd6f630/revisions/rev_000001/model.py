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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _cabinet_shell(width: float, depth: float, height: float, wall: float, z0: float) -> cq.Workplane:
    """Single-piece open-front refrigerator liner/cabinet shell."""
    outer = cq.Workplane("XY").box(width, depth, height).translate((0.0, 0.0, z0 + height / 2.0))

    # The cut starts beyond the front face and stops short of the rear, leaving a
    # back panel and continuous side/top/bottom walls instead of a solid block.
    cavity_depth = depth - wall + 0.045
    cavity_y = -depth / 2.0 - 0.020 + cavity_depth / 2.0
    cavity = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, cavity_depth, height - 2.0 * wall)
        .translate((0.0, cavity_y, z0 + height / 2.0))
    )
    shell = outer.cut(cavity)

    divider_z = z0 + 0.735
    divider = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - wall, wall)
        .translate((0.0, cavity_y, divider_z))
    )
    return shell.union(divider)


def _rounded_door(width: float, thickness: float, height: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(width, thickness, height).edges("|Z").fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_apartment_refrigerator")

    warm_white = model.material("warm_white_enamel", rgba=(0.91, 0.90, 0.84, 1.0))
    liner = model.material("pale_interior_liner", rgba=(0.82, 0.84, 0.82, 1.0))
    dark_rubber = model.material("dark_rubber_gasket", rgba=(0.025, 0.027, 0.030, 1.0))
    brushed_metal = model.material("brushed_hinge_metal", rgba=(0.58, 0.59, 0.56, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.09, 0.10, 0.11, 1.0))
    label_blue = model.material("cool_blue_mark", rgba=(0.16, 0.38, 0.90, 1.0))

    cabinet_w = 0.60
    cabinet_d = 0.55
    cabinet_h = 1.10
    wall = 0.035
    base_z = 0.08
    front_y = -cabinet_d / 2.0
    hinge_x = -cabinet_w / 2.0 - 0.015
    hinge_y = front_y - 0.028

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell(cabinet_w, cabinet_d, cabinet_h, wall, base_z), "cabinet_shell"),
        material=liner,
        name="cabinet_shell",
    )

    # Short feet and a shadow plinth make the refrigerator read as a compact
    # apartment appliance rather than a plain box.
    cabinet.visual(
        Box((0.52, 0.46, 0.045)),
        origin=Origin(xyz=(0.0, 0.02, 0.0575)),
        material=charcoal,
        name="toe_plinth",
    )
    for i, x in enumerate((-0.22, 0.22)):
        for j, y in enumerate((-0.18, 0.19)):
            cabinet.visual(
                Cylinder(radius=0.022, length=0.045),
                origin=Origin(xyz=(x, y, 0.0225)),
                material=charcoal,
                name=f"foot_{i}_{j}",
            )

    # Fixed interior thermostat housing, mounted just under the freezer divider.
    housing_center = (0.125, front_y + 0.070, base_z + 0.683)
    housing_size = (0.185, 0.085, 0.070)
    cabinet.visual(
        Box(housing_size),
        origin=Origin(xyz=housing_center),
        material=charcoal,
        name="control_housing",
    )
    dial_center = (
        housing_center[0],
        housing_center[1] - housing_size[1] / 2.0,
        housing_center[2],
    )
    # Small fixed tick marks around the dial on the housing face.
    cabinet.visual(
        Box((0.006, 0.004, 0.032)),
        origin=Origin(xyz=(dial_center[0] - 0.044, dial_center[1] - 0.001, dial_center[2] + 0.006)),
        material=label_blue,
        name="cold_tick",
    )
    cabinet.visual(
        Box((0.006, 0.004, 0.038)),
        origin=Origin(xyz=(dial_center[0] + 0.044, dial_center[1] - 0.001, dial_center[2] + 0.006)),
        material=warm_white,
        name="warm_tick",
    )

    lower_z = base_z + 0.030
    lower_h = 0.675
    upper_z = base_z + 0.745
    upper_h = 0.335
    door_w = 0.580
    door_t = 0.055
    door_y = -0.039

    # Cabinet-side hinge clips and pins.  The web visibly clips each door barrel
    # back to the cabinet side, so the panels remain retained during swing.
    for prefix, z0, height in (("lower", lower_z, lower_h), ("upper", upper_z, upper_h)):
        cabinet.visual(
            Cylinder(radius=0.0075, length=height + 0.018),
            origin=Origin(xyz=(hinge_x, hinge_y, z0 + height / 2.0 - 0.009)),
            material=brushed_metal,
            name=f"{prefix}_hinge_pin",
        )
        cabinet.visual(
            Box((0.018, 0.034, height)),
            origin=Origin(xyz=(hinge_x + 0.010, hinge_y + 0.017, z0 + height / 2.0)),
            material=brushed_metal,
            name=f"{prefix}_hinge_web",
        )

    main_door = model.part("main_door")
    main_door.visual(
        mesh_from_cadquery(_rounded_door(door_w, door_t, lower_h, 0.018), "main_door_panel"),
        origin=Origin(xyz=(door_w / 2.0 + 0.010, door_y, lower_h / 2.0)),
        material=warm_white,
        name="door_panel",
    )
    main_door.visual(
        Cylinder(radius=0.0175, length=lower_h),
        origin=Origin(xyz=(0.0, 0.0, lower_h / 2.0)),
        material=brushed_metal,
        name="hinge_barrel",
    )
    main_door.visual(
        Box((0.045, 0.016, lower_h)),
        origin=Origin(xyz=(0.023, -0.018, lower_h / 2.0)),
        material=brushed_metal,
        name="hinge_leaf",
    )
    # Magnetic gasket around the inner face of the main door.
    main_door.visual(
        Box((0.500, 0.010, 0.026)),
        origin=Origin(xyz=(0.305, -0.007, lower_h - 0.045)),
        material=dark_rubber,
        name="top_gasket",
    )
    main_door.visual(
        Box((0.500, 0.010, 0.026)),
        origin=Origin(xyz=(0.305, -0.007, 0.045)),
        material=dark_rubber,
        name="bottom_gasket",
    )
    main_door.visual(
        Box((0.024, 0.010, 0.565)),
        origin=Origin(xyz=(0.055, -0.007, lower_h / 2.0)),
        material=dark_rubber,
        name="hinge_gasket",
    )
    main_door.visual(
        Box((0.024, 0.010, 0.565)),
        origin=Origin(xyz=(0.550, -0.007, lower_h / 2.0)),
        material=dark_rubber,
        name="latch_gasket",
    )
    main_door.visual(
        Box((0.030, 0.024, 0.395)),
        origin=Origin(xyz=(0.540, -0.075, lower_h / 2.0 + 0.015)),
        material=charcoal,
        name="vertical_handle",
    )

    freezer_door = model.part("freezer_door")
    freezer_door.visual(
        mesh_from_cadquery(_rounded_door(door_w, door_t, upper_h, 0.016), "freezer_door_panel"),
        origin=Origin(xyz=(door_w / 2.0 + 0.010, door_y, upper_h / 2.0)),
        material=warm_white,
        name="door_panel",
    )
    freezer_door.visual(
        Cylinder(radius=0.0175, length=upper_h),
        origin=Origin(xyz=(0.0, 0.0, upper_h / 2.0)),
        material=brushed_metal,
        name="hinge_barrel",
    )
    freezer_door.visual(
        Box((0.045, 0.016, upper_h)),
        origin=Origin(xyz=(0.023, -0.018, upper_h / 2.0)),
        material=brushed_metal,
        name="hinge_leaf",
    )
    freezer_door.visual(
        Box((0.500, 0.010, 0.024)),
        origin=Origin(xyz=(0.305, -0.007, upper_h - 0.040)),
        material=dark_rubber,
        name="top_gasket",
    )
    freezer_door.visual(
        Box((0.500, 0.010, 0.024)),
        origin=Origin(xyz=(0.305, -0.007, 0.040)),
        material=dark_rubber,
        name="bottom_gasket",
    )
    freezer_door.visual(
        Box((0.024, 0.010, 0.245)),
        origin=Origin(xyz=(0.055, -0.007, upper_h / 2.0)),
        material=dark_rubber,
        name="hinge_gasket",
    )
    freezer_door.visual(
        Box((0.024, 0.010, 0.245)),
        origin=Origin(xyz=(0.550, -0.007, upper_h / 2.0)),
        material=dark_rubber,
        name="latch_gasket",
    )
    freezer_door.visual(
        Box((0.185, 0.023, 0.035)),
        origin=Origin(xyz=(0.475, -0.075, upper_h / 2.0 - 0.035)),
        material=charcoal,
        name="short_handle",
    )

    thermostat_dial = model.part("thermostat_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.046,
            0.024,
            body_style="faceted",
            top_diameter=0.038,
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=16, depth=0.0008),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "thermostat_dial_cap",
    )
    thermostat_dial.visual(dial_mesh, material=liner, name="dial_cap")
    thermostat_dial.visual(
        Box((0.004, 0.030, 0.003)),
        origin=Origin(xyz=(0.0, 0.006, 0.025)),
        material=warm_white,
        name="pointer_marker",
    )

    model.articulation(
        "main_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=main_door,
        origin=Origin(xyz=(hinge_x, hinge_y, lower_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.3, lower=0.0, upper=1.85),
    )
    model.articulation(
        "freezer_door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=freezer_door,
        origin=Origin(xyz=(hinge_x, hinge_y, upper_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.4, lower=0.0, upper=1.75),
    )
    model.articulation(
        "dial_axis",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=thermostat_dial,
        origin=Origin(xyz=dial_center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=2.0, lower=-2.35, upper=2.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    main_door = object_model.get_part("main_door")
    freezer_door = object_model.get_part("freezer_door")
    dial = object_model.get_part("thermostat_dial")
    main_hinge = object_model.get_articulation("main_door_hinge")
    freezer_hinge = object_model.get_articulation("freezer_door_hinge")
    dial_axis = object_model.get_articulation("dial_axis")

    # The cabinet-side pin and clip web are intentionally represented as solid
    # captured hinge hardware inside each door's barrel proxy.
    for door, prefix in ((main_door, "lower"), (freezer_door, "upper")):
        ctx.allow_overlap(
            cabinet,
            door,
            elem_a=f"{prefix}_hinge_pin",
            elem_b="hinge_barrel",
            reason="The compact refrigerator door barrel is clipped around a solid side hinge pin proxy.",
        )
        ctx.allow_overlap(
            cabinet,
            door,
            elem_a=f"{prefix}_hinge_web",
            elem_b="hinge_barrel",
            reason="The hinge web is a local retainer tab modeled inside the simplified hinge barrel envelope.",
        )
        ctx.expect_within(
            cabinet,
            door,
            axes="xy",
            inner_elem=f"{prefix}_hinge_pin",
            outer_elem="hinge_barrel",
            margin=0.002,
            name=f"{prefix} hinge pin stays inside barrel",
        )
        ctx.expect_overlap(
            cabinet,
            door,
            axes="z",
            elem_a=f"{prefix}_hinge_pin",
            elem_b="hinge_barrel",
            min_overlap=0.25,
            name=f"{prefix} hinge retains vertical engagement",
        )

    with ctx.pose({main_hinge: 0.0, freezer_hinge: 0.0}):
        ctx.expect_gap(
            cabinet,
            main_door,
            axis="y",
            positive_elem="cabinet_shell",
            negative_elem="door_panel",
            min_gap=0.015,
            max_gap=0.045,
            name="main door sits just in front of cabinet face",
        )
        ctx.expect_gap(
            cabinet,
            freezer_door,
            axis="y",
            positive_elem="cabinet_shell",
            negative_elem="door_panel",
            min_gap=0.015,
            max_gap=0.045,
            name="freezer door sits just in front of cabinet face",
        )

    main_rest = ctx.part_element_world_aabb(main_door, elem="door_panel")
    freezer_rest = ctx.part_element_world_aabb(freezer_door, elem="door_panel")
    with ctx.pose({main_hinge: 1.10, freezer_hinge: 1.00}):
        main_open = ctx.part_element_world_aabb(main_door, elem="door_panel")
        freezer_open = ctx.part_element_world_aabb(freezer_door, elem="door_panel")
    ctx.check(
        "main door swings outward from hinge side",
        main_rest is not None
        and main_open is not None
        and main_open[0][1] < main_rest[0][1] - 0.20,
        details=f"rest={main_rest}, open={main_open}",
    )
    ctx.check(
        "freezer door swings outward from same hinge side",
        freezer_rest is not None
        and freezer_open is not None
        and freezer_open[0][1] < freezer_rest[0][1] - 0.12,
        details=f"rest={freezer_rest}, open={freezer_open}",
    )

    ctx.expect_contact(
        dial,
        cabinet,
        elem_a="dial_cap",
        elem_b="control_housing",
        contact_tol=0.002,
        name="thermostat dial seats in fixed control housing",
    )
    pointer_rest = ctx.part_element_world_aabb(dial, elem="pointer_marker")
    with ctx.pose({dial_axis: 1.20}):
        pointer_rotated = ctx.part_element_world_aabb(dial, elem="pointer_marker")
    ctx.check(
        "thermostat pointer rotates about local dial axis",
        pointer_rest is not None
        and pointer_rotated is not None
        and (pointer_rotated[1][0] - pointer_rotated[0][0]) > (pointer_rest[1][0] - pointer_rest[0][0]) + 0.010,
        details=f"rest={pointer_rest}, rotated={pointer_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
