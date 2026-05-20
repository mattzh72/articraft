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
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_workshop_speaker")

    dark_plastic = model.material("charcoal_plastic", rgba=(0.045, 0.050, 0.055, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.010, 0.010, 0.012, 1.0))
    grille_metal = model.material("dark_perforated_metal", rgba=(0.020, 0.024, 0.028, 1.0))
    handle_mat = model.material("rubberized_handle", rgba=(0.018, 0.020, 0.022, 1.0))
    accent = model.material("orange_workshop_trim", rgba=(0.95, 0.38, 0.08, 1.0))
    button_red = model.material("red_button", rgba=(0.85, 0.05, 0.035, 1.0))

    enclosure = model.part("enclosure")
    enclosure.visual(
        Box((0.210, 0.320, 0.220)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=dark_plastic,
        name="main_housing",
    )

    # A real perforated front grille rather than a flat placeholder.
    grille = PerforatedPanelGeometry(
        (0.135, 0.210),
        0.004,
        hole_diameter=0.006,
        pitch=(0.012, 0.012),
        frame=0.012,
        corner_radius=0.006,
        stagger=True,
    )
    enclosure.visual(
        mesh_from_geometry(grille, "front_grille"),
        origin=Origin(xyz=(-0.1075, 0.020, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_metal,
        name="front_grille",
    )
    enclosure.visual(
        Box((0.006, 0.235, 0.160)),
        origin=Origin(xyz=(-0.111, 0.020, 0.125)),
        material=Material("speaker_shadow", rgba=(0.0, 0.0, 0.0, 1.0)),
        name="dark_driver_cavity",
    )

    # Rugged corner bumpers and feet, all slightly seated into the enclosure.
    for y, y_name in ((-0.145, "side_0"), (0.145, "side_1")):
        for z in (0.035, 0.195):
            enclosure.visual(
                Box((0.030, 0.026, 0.040)),
                origin=Origin(xyz=(-0.112, y, z)),
                material=black_rubber,
                name=f"front_bumper_{y_name}_{'low' if z < 0.1 else 'high'}",
            )
    for y, y_name in ((-0.120, "side_0"), (0.120, "side_1")):
        enclosure.visual(
            Box((0.060, 0.050, 0.018)),
            origin=Origin(xyz=(0.025, y, 0.009)),
            material=black_rubber,
            name=f"rubber_foot_{y_name}",
        )

    # Fixed side pivot bosses for the carrying handle.
    enclosure.visual(
        Cylinder(radius=0.027, length=0.013),
        origin=Origin(xyz=(0.0, -0.1655, 0.225), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="handle_pivot_boss_side_0",
    )
    enclosure.visual(
        Cylinder(radius=0.010, length=0.015),
        origin=Origin(xyz=(0.0, -0.1560, 0.225), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="pivot_center_cap_side_0",
    )
    enclosure.visual(
        Cylinder(radius=0.027, length=0.013),
        origin=Origin(xyz=(0.0, 0.1655, 0.225), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="handle_pivot_boss_side_1",
    )
    enclosure.visual(
        Cylinder(radius=0.010, length=0.015),
        origin=Origin(xyz=(0.0, 0.1560, 0.225), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="pivot_center_cap_side_1",
    )

    # Side control mounting face.
    enclosure.visual(
        Cylinder(radius=0.035, length=0.006),
        origin=Origin(xyz=(0.020, 0.163, 0.120), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="knob_mount_pad",
    )

    # Four bars form a real prismatic guide around the push button stem.
    button_y = -0.120
    button_z = 0.055
    enclosure.visual(
        Box((0.012, 0.006, 0.038)),
        origin=Origin(xyz=(-0.111, button_y - 0.019, button_z)),
        material=black_rubber,
        name="button_side_rail_0",
    )
    enclosure.visual(
        Box((0.012, 0.006, 0.038)),
        origin=Origin(xyz=(-0.111, button_y + 0.019, button_z)),
        material=black_rubber,
        name="button_side_rail_1",
    )
    for side_z, rail_name in ((button_z - 0.017, "button_cross_rail_low"), (button_z + 0.017, "button_cross_rail_high")):
        enclosure.visual(
            Box((0.012, 0.044, 0.006)),
            origin=Origin(xyz=(-0.111, button_y, side_z)),
            material=black_rubber,
            name=rail_name,
        )

    handle = model.part("handle")
    # Child frame is the pivot axis center.  The y-offset side arms and the
    # crossbar form one continuous U-shaped carrying handle.
    handle.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(0.0, -0.177, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="pivot_washer_side_0",
    )
    handle.visual(
        Cylinder(radius=0.011, length=0.120),
        origin=Origin(xyz=(0.0, -0.190, 0.060)),
        material=handle_mat,
        name="upright_arm_side_0",
    )
    handle.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(0.0, 0.177, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="pivot_washer_side_1",
    )
    handle.visual(
        Cylinder(radius=0.011, length=0.120),
        origin=Origin(xyz=(0.0, 0.190, 0.060)),
        material=handle_mat,
        name="upright_arm_side_1",
    )
    handle.visual(
        Cylinder(radius=0.014, length=0.380),
        origin=Origin(xyz=(0.0, 0.0, 0.120), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=handle_mat,
        name="top_grip",
    )
    handle.visual(
        Cylinder(radius=0.018, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.120), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="padded_grip_sleeve",
    )

    volume_knob = model.part("volume_knob")
    volume_knob.visual(
        Cylinder(radius=0.010, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=black_rubber,
        name="short_shaft",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.054,
            0.023,
            body_style="faceted",
            top_diameter=0.045,
            edge_radius=0.0012,
            grip=KnobGrip(style="ribbed", count=18, depth=0.0012, width=0.0020),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "volume_knob_cap",
    )
    volume_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=accent,
        name="knob_cap",
    )

    front_button = model.part("front_button")
    front_button.visual(
        Box((0.018, 0.032, 0.022)),
        origin=Origin(xyz=(-0.007, 0.0, 0.0)),
        material=button_red,
        name="sliding_stem",
    )
    front_button.visual(
        Cylinder(radius=0.018, length=0.011),
        origin=Origin(xyz=(-0.0195, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=button_red,
        name="button_cap",
    )

    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "volume_axis",
        ArticulationType.CONTINUOUS,
        parent=enclosure,
        child=volume_knob,
        origin=Origin(xyz=(0.020, 0.166, 0.120), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    model.articulation(
        "button_slide",
        ArticulationType.PRISMATIC,
        parent=enclosure,
        child=front_button,
        origin=Origin(xyz=(-0.113, button_y, button_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.4, lower=0.0, upper=0.006),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    enclosure = object_model.get_part("enclosure")
    handle = object_model.get_part("handle")
    volume_knob = object_model.get_part("volume_knob")
    front_button = object_model.get_part("front_button")
    handle_joint = object_model.get_articulation("handle_pivot")
    knob_joint = object_model.get_articulation("volume_axis")
    button_joint = object_model.get_articulation("button_slide")

    ctx.expect_contact(
        handle,
        enclosure,
        elem_a="pivot_washer_side_1",
        elem_b="handle_pivot_boss_side_1",
        contact_tol=0.001,
        name="handle pivot washer bears on boss side 1",
    )
    ctx.expect_contact(
        handle,
        enclosure,
        elem_a="pivot_washer_side_0",
        elem_b="handle_pivot_boss_side_0",
        contact_tol=0.001,
        name="handle pivot washer bears on boss side 0",
    )
    ctx.expect_gap(
        handle,
        enclosure,
        axis="z",
        min_gap=0.085,
        positive_elem="top_grip",
        negative_elem="main_housing",
        name="handle clears top of housing",
    )
    ctx.expect_contact(
        volume_knob,
        enclosure,
        elem_a="short_shaft",
        elem_b="knob_mount_pad",
        contact_tol=0.001,
        name="volume knob shaft is mounted on side pad",
    )
    ctx.expect_contact(
        front_button,
        enclosure,
        elem_a="sliding_stem",
        elem_b="button_side_rail_0",
        contact_tol=0.001,
        name="push button stem rides in side rail",
    )

    rest_button_pos = ctx.part_world_position(front_button)
    with ctx.pose({button_joint: 0.006}):
        pressed_button_pos = ctx.part_world_position(front_button)
        ctx.expect_gap(
            enclosure,
            front_button,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="main_housing",
            negative_elem="sliding_stem",
            name="pressed button remains just outside front face",
        )

    ctx.check(
        "button moves inward on prismatic guide",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[0] > rest_button_pos[0] + 0.005,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    rest_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_joint: 0.9}):
        swung_handle_aabb = ctx.part_element_world_aabb(handle, elem="top_grip")
    ctx.check(
        "handle rotates above enclosure",
        rest_handle_pos is not None
        and swung_handle_aabb is not None
        and swung_handle_aabb[0][2] > 0.265,
        details=f"rest={rest_handle_pos}, swung_aabb={swung_handle_aabb}",
    )

    with ctx.pose({knob_joint: math.pi / 2.0}):
        ctx.expect_contact(
            volume_knob,
            enclosure,
            elem_a="short_shaft",
            elem_b="knob_mount_pad",
            contact_tol=0.001,
            name="rotated knob remains seated on shaft",
        )

    return ctx.report()


object_model = build_object_model()
