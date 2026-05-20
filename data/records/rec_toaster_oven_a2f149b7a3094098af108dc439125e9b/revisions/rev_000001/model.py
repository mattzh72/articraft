from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _translated(profile, dx: float, dy: float):
    return [(x + dx, y + dy) for x, y in profile]


def _front_plane_mesh(geometry, *, x: float, z: float):
    """Map a 2D XY extrusion so width is world Y, height is world Z, depth is world X."""
    return geometry.rotate_z(pi / 2.0).rotate_y(pi / 2.0).translate(x, 0.0, z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_toaster_oven")

    brushed = model.material("brushed_steel", rgba=(0.66, 0.66, 0.62, 1.0))
    black = model.material("black_trim", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("rubber_black", rgba=(0.02, 0.02, 0.018, 1.0))
    glass = model.material("smoky_glass", rgba=(0.08, 0.13, 0.16, 0.45))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.82, 0.78, 1.0))
    dark = model.material("dark_cavity", rgba=(0.006, 0.006, 0.007, 1.0))
    warm = model.material("warm_heater", rgba=(1.0, 0.28, 0.05, 1.0))
    white = model.material("white_mark", rgba=(0.92, 0.90, 0.82, 1.0))

    chassis = model.part("chassis")

    # Countertop toaster-oven body: about 62 cm wide, 38 cm deep, and 28 cm tall.
    chassis.visual(
        Box((0.38, 0.62, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.2675)),
        material=brushed,
        name="top_shell",
    )
    chassis.visual(
        Box((0.38, 0.62, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=brushed,
        name="bottom_shell",
    )
    chassis.visual(
        Box((0.025, 0.62, 0.28)),
        origin=Origin(xyz=(-0.1775, 0.0, 0.14)),
        material=brushed,
        name="rear_wall",
    )
    for y, name in ((-0.2975, "side_wall_0"), (0.2975, "side_wall_1")):
        chassis.visual(
            Box((0.38, 0.025, 0.28)),
            origin=Origin(xyz=(0.0, y, 0.14)),
            material=brushed,
            name=name,
        )

    front_holes = [
        _translated(rounded_rect_profile(0.54, 0.145, 0.012, corner_segments=8), 0.0, -0.0025),
        _translated(superellipse_profile(0.024, 0.024, 2.0, segments=32), 0.0, 0.107),
        _translated(superellipse_profile(0.042, 0.042, 2.0, segments=32), -0.120, 0.107),
        _translated(superellipse_profile(0.042, 0.042, 2.0, segments=32), 0.120, 0.107),
    ]
    front_frame_geom = _front_plane_mesh(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.62, 0.28, 0.026, corner_segments=10),
            front_holes,
            0.018,
            center=True,
        ),
        x=0.185,
        z=0.140,
    )
    chassis.visual(
        mesh_from_geometry(front_frame_geom, "front_frame"),
        material=black,
        name="front_frame",
    )

    # Dark connected oven liner and visible heating elements behind the glass.
    chassis.visual(
        Box((0.010, 0.57, 0.165)),
        origin=Origin(xyz=(-0.145, 0.0, 0.145)),
        material=dark,
        name="cavity_back",
    )
    chassis.visual(
        Box((0.305, 0.57, 0.010)),
        origin=Origin(xyz=(0.005, 0.0, 0.072)),
        material=dark,
        name="cavity_floor",
    )
    chassis.visual(
        Box((0.305, 0.57, 0.010)),
        origin=Origin(xyz=(0.005, 0.0, 0.218)),
        material=dark,
        name="cavity_ceiling",
    )
    for z, name in ((0.090, "lower_heater"), (0.197, "upper_heater")):
        chassis.visual(
            Cylinder(radius=0.0055, length=0.57),
            origin=Origin(xyz=(0.015, 0.0, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=warm,
            name=name,
        )

    # Fixed tray rails inside the lower slot.
    for y, name in ((-0.263, "tray_guide_0"), (0.263, "tray_guide_1")):
        chassis.visual(
            Box((0.305, 0.012, 0.012)),
            origin=Origin(xyz=(0.000, y, 0.047)),
            material=chrome,
            name=name,
        )
    for y, name in ((-0.190, "hinge_saddle_0"), (0.190, "hinge_saddle_1")):
        chassis.visual(
            Box((0.012, 0.050, 0.020)),
            origin=Origin(xyz=(0.191, y, 0.065)),
            material=black,
            name=name,
        )
    for side_y, prefix in ((-0.277, "guide_mount_0"), (0.277, "guide_mount_1")):
        for i, x in enumerate((-0.105, 0.000, 0.105)):
            chassis.visual(
                Box((0.030, 0.016, 0.012)),
                origin=Origin(xyz=(x, side_y, 0.047)),
                material=chrome,
                name=f"{prefix}_{i}",
            )

    # Side ventilation slots as shallow dark insets on the metal side panels.
    for side_y, prefix in ((-0.311, "vent_0"), (0.311, "vent_1")):
        for i in range(5):
            chassis.visual(
                Box((0.070, 0.004, 0.008)),
                origin=Origin(xyz=(-0.040 + 0.034 * i, side_y, 0.226)),
                material=black,
                name=f"{prefix}_{i}",
            )

    for x in (-0.125, 0.125):
        for y in (-0.225, 0.225):
            chassis.visual(
                Cylinder(radius=0.025, length=0.014),
                origin=Origin(xyz=(x, y, -0.006)),
                material=rubber,
                name=f"foot_{x:+.3f}_{y:+.3f}",
            )

    door = model.part("door")
    door_frame_geom = _front_plane_mesh(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.54, 0.140, 0.014, corner_segments=8),
            [rounded_rect_profile(0.455, 0.095, 0.010, corner_segments=8)],
            0.018,
            center=True,
        ),
        x=0.009,
        z=0.070,
    )
    door.visual(
        mesh_from_geometry(door_frame_geom, "door_frame"),
        material=black,
        name="door_frame",
    )
    door.visual(
        Box((0.006, 0.468, 0.105)),
        origin=Origin(xyz=(0.005, 0.0, 0.070)),
        material=glass,
        name="glass_pane",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.430),
        origin=Origin(xyz=(0.009, 0.0, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.014, 0.044, 0.043)),
        origin=Origin(xyz=(0.024, -0.183, 0.123)),
        material=black,
        name="handle_post_0",
    )
    door.visual(
        Box((0.014, 0.044, 0.043)),
        origin=Origin(xyz=(0.024, 0.183, 0.123)),
        material=black,
        name="handle_post_1",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.415),
        origin=Origin(xyz=(0.038, 0.0, 0.130), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_grip",
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=door,
        origin=Origin(xyz=(0.198, 0.0, 0.065)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.72),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=chrome,
        name="dial_shaft",
    )
    knob_geom = KnobGeometry(
        0.057,
        0.026,
        body_style="skirted",
        top_diameter=0.047,
        grip=KnobGrip(style="fluted", count=22, depth=0.0013),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
    )
    dial.visual(
        mesh_from_geometry(knob_geom, "dial_knob"),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="dial_knob",
    )
    dial.visual(
        Box((0.003, 0.005, 0.026)),
        origin=Origin(xyz=(0.038, 0.0, 0.012)),
        material=white,
        name="dial_pointer",
    )
    model.articulation(
        "dial_shaft",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=dial,
        origin=Origin(xyz=(0.194, 0.0, 0.247)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=6.0),
    )

    for idx, y in enumerate((-0.120, 0.120)):
        button = model.part(f"button_{idx}")
        button.visual(
            Cylinder(radius=0.018, length=0.018),
            origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=black,
            name="button_stem",
        )
        model.articulation(
            f"button_{idx}_slide",
            ArticulationType.PRISMATIC,
            parent=chassis,
            child=button,
            origin=Origin(xyz=(0.194, y, 0.247)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=0.12, lower=0.0, upper=0.012),
        )

    tray = model.part("crumb_tray")
    tray.visual(
        Box((0.290, 0.500, 0.010)),
        origin=Origin(xyz=(-0.145, 0.0, -0.006)),
        material=chrome,
        name="tray_pan",
    )
    for y, name in ((-0.251, "tray_flange_0"), (0.251, "tray_flange_1")):
        tray.visual(
            Box((0.290, 0.012, 0.026)),
            origin=Origin(xyz=(-0.145, y, 0.006)),
            material=chrome,
            name=name,
        )
    tray.visual(
        Box((0.020, 0.540, 0.035)),
        origin=Origin(xyz=(0.023, 0.0, 0.009)),
        material=brushed,
        name="tray_front_lip",
    )
    tray.visual(
        Box((0.014, 0.180, 0.010)),
        origin=Origin(xyz=(0.0065, 0.0, 0.004)),
        material=brushed,
        name="tray_lip_bridge",
    )
    tray.visual(
        Box((0.006, 0.210, 0.010)),
        origin=Origin(xyz=(0.036, 0.0, 0.009)),
        material=black,
        name="tray_pull_groove",
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=tray,
        origin=Origin(xyz=(0.145, 0.0, 0.047)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.35, lower=0.0, upper=0.180),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    tray = object_model.get_part("crumb_tray")

    door_hinge = object_model.get_articulation("door_hinge")
    dial_shaft = object_model.get_articulation("dial_shaft")
    button_0_slide = object_model.get_articulation("button_0_slide")
    button_1_slide = object_model.get_articulation("button_1_slide")
    tray_slide = object_model.get_articulation("tray_slide")

    for button in (button_0, button_1):
        ctx.allow_overlap(
            button,
            chassis,
            elem_a="button_stem",
            elem_b="front_frame",
            reason="Each push-button stem is intentionally captured through a front-panel bore.",
        )
        ctx.expect_within(
            button,
            chassis,
            axes="yz",
            inner_elem="button_stem",
            outer_elem="front_frame",
            margin=0.0,
            name=f"{button.name} stem stays centered in the front panel",
        )
        ctx.expect_overlap(
            button,
            chassis,
            axes="x",
            elem_a="button_stem",
            elem_b="front_frame",
            min_overlap=0.006,
            name=f"{button.name} stem remains captured through the panel",
        )

    ctx.check("dial is continuous", dial_shaft.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("door is lower hinged", door_hinge.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("buttons are prismatic", button_0_slide.articulation_type == ArticulationType.PRISMATIC and button_1_slide.articulation_type == ArticulationType.PRISMATIC)
    ctx.check("tray is prismatic", tray_slide.articulation_type == ArticulationType.PRISMATIC)

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            door,
            chassis,
            axis="x",
            min_gap=0.001,
            max_gap=0.008,
            positive_elem="door_frame",
            negative_elem="front_frame",
            name="closed door sits just proud of the front frame",
        )
        ctx.expect_overlap(
            door,
            chassis,
            axes="yz",
            elem_a="door_frame",
            elem_b="front_frame",
            min_overlap=0.12,
            name="closed door covers the oven opening",
        )
        closed_aabb = ctx.part_world_aabb(door)

    with ctx.pose({door_hinge: 1.35}):
        opened_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door rotates downward and forward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][0] > closed_aabb[1][0] + 0.08
        and opened_aabb[1][2] < closed_aabb[1][2] - 0.03,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    ctx.expect_contact(
        dial,
        chassis,
        elem_a="dial_shaft",
        elem_b="front_frame",
        contact_tol=0.002,
        name="dial shaft emerges from the front panel",
    )
    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_0_slide: 0.010}):
        depressed_button_0 = ctx.part_world_position(button_0)
        held_button_1 = ctx.part_world_position(button_1)
    ctx.check(
        "button_0 depresses independently",
        rest_button_0 is not None
        and depressed_button_0 is not None
        and rest_button_1 is not None
        and held_button_1 is not None
        and depressed_button_0[0] < rest_button_0[0] - 0.008
        and abs(held_button_1[0] - rest_button_1[0]) < 1e-6,
        details=f"button_0 rest={rest_button_0}, depressed={depressed_button_0}; button_1 rest={rest_button_1}, held={held_button_1}",
    )

    ctx.expect_overlap(
        tray,
        chassis,
        axes="x",
        elem_a="tray_pan",
        elem_b="tray_guide_0",
        min_overlap=0.25,
        name="closed crumb tray is fully carried by a guide",
    )
    closed_tray = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.180}):
        extended_tray = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            chassis,
            axes="x",
            elem_a="tray_pan",
            elem_b="tray_guide_0",
            min_overlap=0.08,
            name="extended crumb tray remains engaged on its guide",
        )
    ctx.check(
        "crumb tray slides forward",
        closed_tray is not None and extended_tray is not None and extended_tray[0] > closed_tray[0] + 0.16,
        details=f"closed={closed_tray}, extended={extended_tray}",
    )

    return ctx.report()


object_model = build_object_model()
