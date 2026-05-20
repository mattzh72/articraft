from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_microwave")

    satin_steel = Material("satin_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_plastic = Material("dark_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    charcoal = Material("charcoal_panel", rgba=(0.06, 0.065, 0.07, 1.0))
    smoked_glass = Material("smoked_glass", rgba=(0.03, 0.05, 0.065, 0.55))
    cavity_enamel = Material("dark_cavity_enamel", rgba=(0.12, 0.13, 0.13, 1.0))
    glass_green = Material("green_tinted_glass", rgba=(0.55, 0.82, 0.78, 0.38))
    rubber = Material("matte_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    # Overall full-size family microwave: about 60 cm wide, 43 cm deep,
    # and 34 cm high including feet.  The front is the -Y face.
    housing = model.part("housing")
    housing.visual(Box((0.600, 0.430, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.3225)), material=satin_steel, name="top_shell")
    housing.visual(Box((0.600, 0.430, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0375)), material=satin_steel, name="bottom_shell")
    housing.visual(Box((0.035, 0.430, 0.320)), origin=Origin(xyz=(-0.2825, 0.0, 0.180)), material=satin_steel, name="side_wall_0")
    housing.visual(Box((0.035, 0.430, 0.320)), origin=Origin(xyz=(0.2825, 0.0, 0.180)), material=satin_steel, name="side_wall_1")
    housing.visual(Box((0.600, 0.035, 0.320)), origin=Origin(xyz=(0.0, 0.1975, 0.180)), material=satin_steel, name="rear_wall")
    housing.visual(Box((0.030, 0.395, 0.285)), origin=Origin(xyz=(0.150, 0.000, 0.180)), material=satin_steel, name="center_divider")
    housing.visual(Box((0.150, 0.040, 0.285)), origin=Origin(xyz=(0.225, -0.195, 0.180)), material=charcoal, name="control_face")
    housing.visual(Box((0.016, 0.045, 0.270)), origin=Origin(xyz=(-0.298, -0.2375, 0.180)), material=satin_steel, name="hinge_pocket")

    # A shallow open cooking cavity is visible whenever the hinged door swings out.
    housing.visual(Box((0.395, 0.012, 0.220)), origin=Origin(xyz=(-0.070, 0.125, 0.180)), material=cavity_enamel, name="cavity_back")
    housing.visual(Box((0.395, 0.300, 0.010)), origin=Origin(xyz=(-0.070, -0.025, 0.065)), material=cavity_enamel, name="cavity_floor")
    housing.visual(Box((0.395, 0.300, 0.010)), origin=Origin(xyz=(-0.070, -0.025, 0.295)), material=cavity_enamel, name="cavity_ceiling")
    housing.visual(Box((0.010, 0.300, 0.220)), origin=Origin(xyz=(-0.265, -0.025, 0.180)), material=cavity_enamel, name="cavity_side_0")
    housing.visual(Box((0.010, 0.300, 0.220)), origin=Origin(xyz=(0.125, -0.025, 0.180)), material=cavity_enamel, name="cavity_side_1")

    cavity_lip = BezelGeometry(
        (0.365, 0.215),
        (0.430, 0.285),
        0.014,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.010,
        outer_corner_radius=0.014,
        face=BezelFace(style="radiused_step", front_lip=0.002, fillet=0.001),
    )
    housing.visual(
        mesh_from_geometry(cavity_lip, "cavity_lip"),
        origin=Origin(xyz=(-0.070, -0.218, 0.180), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="cavity_lip",
    )

    # Turntable disk seated on the cavity floor.
    housing.visual(Cylinder(radius=0.132, length=0.007), origin=Origin(xyz=(-0.070, -0.025, 0.071)), material=glass_green, name="turntable")
    housing.visual(Cylinder(radius=0.018, length=0.009), origin=Origin(xyz=(-0.070, -0.025, 0.066)), material=dark_plastic, name="turntable_hub")

    # Side cooling slots and simple rubber feet.
    side_slots = SlotPatternPanelGeometry(
        (0.210, 0.105),
        0.004,
        slot_size=(0.034, 0.006),
        pitch=(0.046, 0.017),
        frame=0.012,
        corner_radius=0.006,
    )
    housing.visual(
        mesh_from_geometry(side_slots, "side_vent"),
        origin=Origin(xyz=(0.3015, 0.040, 0.230), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="side_vent",
    )
    for idx, (x, y) in enumerate(((-0.225, -0.145), (0.225, -0.145), (-0.225, 0.145), (0.225, 0.145))):
        housing.visual(Box((0.070, 0.050, 0.020)), origin=Origin(xyz=(x, y, 0.010)), material=rubber, name=f"foot_{idx}")

    # Static control-panel details around the two moving controls.
    housing.visual(Box((0.105, 0.004, 0.040)), origin=Origin(xyz=(0.225, -0.216, 0.272)), material=dark_plastic, name="display_window")
    dial_bezel = BezelGeometry((0.054, 0.054), (0.076, 0.076), 0.006, opening_shape="circle", outer_shape="circle")
    housing.visual(
        mesh_from_geometry(dial_bezel, "dial_bezel"),
        origin=Origin(xyz=(0.225, -0.218, 0.198), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="dial_bezel",
    )
    housing.visual(Box((0.092, 0.004, 0.034)), origin=Origin(xyz=(0.225, -0.216, 0.118)), material=dark_plastic, name="button_recess")

    # The left-side door is modeled in a hinge-line frame and extends along +X.
    door = model.part("door")
    door_frame = BezelGeometry(
        (0.292, 0.160),
        (0.420, 0.255),
        0.030,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.016,
        outer_corner_radius=0.018,
        wall=(0.055, 0.073, 0.048, 0.047),
        face=BezelFace(style="radiused_step", front_lip=0.003, fillet=0.0015),
    )
    door.visual(
        mesh_from_geometry(door_frame, "door_frame"),
        origin=Origin(xyz=(0.210, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="door_frame",
    )
    door.visual(Box((0.304, 0.006, 0.172)), origin=Origin(xyz=(0.198, -0.018, 0.0)), material=smoked_glass, name="window_glass")
    door.visual(Box((0.032, 0.038, 0.255)), origin=Origin(xyz=(0.016, 0.0, 0.0)), material=dark_plastic, name="hinge_stile")
    door.visual(Cylinder(radius=0.008, length=0.260), origin=Origin(), material=dark_plastic, name="hinge_pin")
    door.visual(Box((0.018, 0.040, 0.118)), origin=Origin(xyz=(0.414, -0.002, 0.0)), material=charcoal, name="latch_stile")

    # A rotating jog dial with a gripped side and small pointer dot.
    jog_dial = model.part("jog_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.064,
            0.027,
            body_style="domed",
            edge_radius=0.0012,
            grip=KnobGrip(style="ribbed", count=24, depth=0.0010),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=90.0),
            center=False,
        ),
        "jog_dial",
    )
    jog_dial.visual(dial_mesh, material=satin_steel, name="dial_cap")

    push_button = model.part("push_button")
    push_button.visual(Box((0.078, 0.024, 0.026)), origin=Origin(), material=satin_steel, name="button_cap")
    push_button.visual(Box((0.058, 0.002, 0.004)), origin=Origin(xyz=(0.0, -0.013, 0.006)), material=dark_plastic, name="button_mark")

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(-0.290, -0.250, 0.180)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "dial_shaft",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=jog_dial,
        origin=Origin(xyz=(0.225, -0.2205, 0.198), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )
    model.articulation(
        "button_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=push_button,
        origin=Origin(xyz=(0.225, -0.2295, 0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.20, lower=0.0, upper=0.012),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    jog_dial = object_model.get_part("jog_dial")
    push_button = object_model.get_part("push_button")
    door_hinge = object_model.get_articulation("door_hinge")
    dial_shaft = object_model.get_articulation("dial_shaft")
    button_slide = object_model.get_articulation("button_slide")

    ctx.allow_overlap(
        housing,
        push_button,
        elem_a="button_recess",
        elem_b="button_cap",
        reason="At full press the push-open button intentionally slides into the panel recess.",
    )
    ctx.allow_overlap(
        housing,
        jog_dial,
        elem_a="dial_bezel",
        elem_b="dial_cap",
        reason="The jog dial cap is intentionally seated by a tiny overlap against its bezel-mounted shaft.",
    )
    ctx.allow_overlap(
        housing,
        door,
        elem_a="hinge_pocket",
        elem_b="hinge_pin",
        reason="The door hinge pin is intentionally captured inside the housing-side hinge pocket.",
    )

    with ctx.pose({door_hinge: 0.0, button_slide: 0.0}):
        ctx.expect_gap(
            housing,
            door,
            axis="y",
            positive_elem="cavity_lip",
            negative_elem="door_frame",
            min_gap=0.001,
            max_gap=0.025,
            name="closed door sits proud of the front",
        )
        ctx.expect_overlap(door, housing, axes="xz", min_overlap=0.20, name="door covers cooking cavity")
        ctx.expect_overlap(door, housing, axes="z", elem_a="hinge_pin", elem_b="hinge_pocket", min_overlap=0.20, name="hinge pin is retained vertically")
        ctx.expect_contact(push_button, housing, elem_a="button_cap", elem_b="button_recess", contact_tol=0.002, name="button starts seated in recess")

    rest_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.15}):
        open_door_aabb = ctx.part_world_aabb(door)
        ctx.expect_gap(housing, door, axis="y", positive_elem="cavity_lip", negative_elem="latch_stile", min_gap=0.060, name="free edge swings outward on vertical hinge")
    ctx.check(
        "door opens toward the user",
        rest_door_aabb is not None and open_door_aabb is not None and open_door_aabb[0][1] < rest_door_aabb[0][1] - 0.020,
        details=f"rest={rest_door_aabb}, open={open_door_aabb}",
    )

    rest_button_position = ctx.part_world_position(push_button)
    with ctx.pose({button_slide: 0.012}):
        pressed_button_position = ctx.part_world_position(push_button)
        ctx.expect_gap(
            housing,
            push_button,
            axis="y",
            min_gap=-0.014,
            max_gap=-0.004,
            positive_elem="button_recess",
            negative_elem="button_cap",
            name="pressed button enters panel recess",
        )
    ctx.check(
        "button travels into the control panel",
        rest_button_position is not None and pressed_button_position is not None and pressed_button_position[1] > rest_button_position[1] + 0.010,
        details=f"rest={rest_button_position}, pressed={pressed_button_position}",
    )

    with ctx.pose({dial_shaft: math.pi / 2.0}):
        ctx.expect_contact(jog_dial, housing, elem_a="dial_cap", elem_b="dial_bezel", contact_tol=0.003, name="dial remains on its shaft bezel")

    return ctx.report()


object_model = build_object_model()
