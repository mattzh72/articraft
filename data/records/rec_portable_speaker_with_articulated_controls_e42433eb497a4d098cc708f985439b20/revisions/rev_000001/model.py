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
    model = ArticulatedObject(name="rugged_jobsite_speaker")

    frame_yellow = Material("molded_yellow_frame", rgba=(0.95, 0.66, 0.08, 1.0))
    rubber = Material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    charcoal = Material("charcoal_body", rgba=(0.08, 0.085, 0.09, 1.0))
    grille_mat = Material("perforated_black_steel", rgba=(0.005, 0.006, 0.007, 1.0))
    dark_grey = Material("dark_grey_plastic", rgba=(0.12, 0.12, 0.13, 1.0))
    white = Material("white_indicator", rgba=(0.92, 0.92, 0.86, 1.0))
    metal = Material("dark_burnished_metal", rgba=(0.22, 0.21, 0.19, 1.0))

    body = model.part("body")

    # Main rectangular speaker enclosure nested inside the protective cage.
    body.visual(
        Box((0.405, 0.205, 0.275)),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=charcoal,
        name="main_shell",
    )

    # Boxy protective cage.  The bars overlap the enclosure by a few millimetres
    # because this is one molded/static assembly with the shell seated inside it.
    body.visual(
        Box((0.555, 0.255, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
        material=frame_yellow,
        name="top_rail",
    )
    body.visual(
        Box((0.555, 0.255, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=frame_yellow,
        name="bottom_rail",
    )
    body.visual(
        Box((0.052, 0.255, 0.380)),
        origin=Origin(xyz=(-0.252, 0.0, 0.190)),
        material=frame_yellow,
        name="side_upright_0",
    )
    body.visual(
        Box((0.052, 0.255, 0.380)),
        origin=Origin(xyz=(0.252, 0.0, 0.190)),
        material=frame_yellow,
        name="side_upright_1",
    )

    # Extra dark elastomer pads and feet make the frame read as a jobsite
    # bumper rather than a plain rectangle.
    for index, (x, y, z) in enumerate(
        (
            (-0.252, -0.115, 0.375),
            (-0.252, 0.115, 0.375),
            (0.252, -0.115, 0.375),
            (0.252, 0.115, 0.375),
            (-0.252, -0.115, 0.005),
            (-0.252, 0.115, 0.005),
            (0.252, -0.115, 0.005),
            (0.252, 0.115, 0.005),
        )
    ):
        body.visual(
            Box((0.054, 0.060, 0.075)),
            origin=Origin(xyz=(x, y, z)),
            material=rubber,
            name=f"corner_bumper_{index}",
        )

    for index, x in enumerate((-0.216, 0.216)):
        body.visual(
            Box((0.060, 0.052, 0.105)),
            origin=Origin(xyz=(x, 0.0, 0.190)),
            material=dark_grey,
            name=f"shell_side_mount_{index}",
        )

    for index, x in enumerate((-0.170, 0.170)):
        body.visual(
            Box((0.115, 0.200, 0.020)),
            origin=Origin(xyz=(x, 0.0, -0.030)),
            material=rubber,
            name=f"foot_{index}",
        )

    # Front grille with real perforations over a dark acoustic cavity.
    body.visual(
        Box((0.325, 0.020, 0.215)),
        origin=Origin(xyz=(-0.020, -0.107, 0.185)),
        material=dark_grey,
        name="front_recess",
    )
    grille_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (0.315, 0.205),
            0.006,
            hole_diameter=0.0075,
            pitch=(0.014, 0.014),
            frame=0.016,
            corner_radius=0.010,
            stagger=True,
        ),
        "speaker_perforated_grille",
    )
    body.visual(
        grille_mesh,
        origin=Origin(xyz=(-0.020, -0.122, 0.185), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grille_mat,
        name="front_grille",
    )

    # Speaker driver shadows visible through the grille.
    for index, x in enumerate((-0.080, 0.085)):
        body.visual(
            Cylinder(radius=0.060, length=0.010),
            origin=Origin(xyz=(x, -0.116, 0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"driver_shadow_{index}",
        )

    # Raised upper-corner control island for the volume knob.
    body.visual(
        Box((0.108, 0.022, 0.078)),
        origin=Origin(xyz=(0.153, -0.124, 0.298)),
        material=dark_grey,
        name="control_panel",
    )

    for index, (x, z) in enumerate(((0.107, 0.330), (0.199, 0.330), (0.107, 0.266), (0.199, 0.266))):
        body.visual(
            Cylinder(radius=0.005, length=0.006),
            origin=Origin(xyz=(x, -0.136, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"panel_screw_{index}",
        )

    # Pins fixed to the side uprights for the folding handle.  The handle's
    # proxy sleeves intentionally sit around these pins.
    body.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(-0.292, -0.010, 0.390), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="pivot_pin_0",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(0.292, -0.010, 0.390), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="pivot_pin_1",
    )

    # Folding bail handle.  Its part frame lies on the common pivot axis.
    handle = model.part("handle")
    handle.visual(
        Box((0.660, 0.050, 0.036)),
        origin=Origin(xyz=(0.0, -0.112, 0.060)),
        material=rubber,
        name="top_grip",
    )
    handle.visual(
        Box((0.042, 0.145, 0.036)),
        origin=Origin(xyz=(-0.300, -0.060, 0.030)),
        material=frame_yellow,
        name="side_arm_0",
    )
    handle.visual(
        Box((0.042, 0.145, 0.036)),
        origin=Origin(xyz=(0.300, -0.060, 0.030)),
        material=frame_yellow,
        name="side_arm_1",
    )
    handle.visual(
        Cylinder(radius=0.024, length=0.036),
        origin=Origin(xyz=(-0.300, -0.010, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_yellow,
        name="pivot_sleeve_0",
    )
    handle.visual(
        Cylinder(radius=0.024, length=0.036),
        origin=Origin(xyz=(0.300, -0.010, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_yellow,
        name="pivot_sleeve_1",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    # Rotary volume knob and its short shaft, mounted through the control island.
    volume_knob = model.part("volume_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.058,
            0.033,
            body_style="faceted",
            top_diameter=0.050,
            base_diameter=0.060,
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=20, depth=0.0012, width=0.0020),
            indicator=KnobIndicator(style="line", mode="engraved", angle_deg=90.0),
            center=False,
        ),
        "faceted_volume_knob",
    )
    volume_knob.visual(
        knob_mesh,
        origin=Origin(),
        material=dark_grey,
        name="knob_cap",
    )
    volume_knob.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=metal,
        name="knob_shaft",
    )
    volume_knob.visual(
        Box((0.006, 0.030, 0.0015)),
        origin=Origin(xyz=(0.0, 0.012, 0.0328)),
        material=white,
        name="pointer_line",
    )
    model.articulation(
        "body_to_volume_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=volume_knob,
        origin=Origin(xyz=(0.153, -0.136, 0.298), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0, lower=-2.35, upper=2.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    volume_knob = object_model.get_part("volume_knob")
    handle_joint = object_model.get_articulation("body_to_handle")
    knob_joint = object_model.get_articulation("body_to_volume_knob")

    ctx.allow_overlap(
        body,
        handle,
        elem_a="pivot_pin_0",
        elem_b="pivot_sleeve_0",
        reason="The modeled handle sleeve is a solid proxy intentionally captured around the fixed pivot pin.",
    )
    ctx.allow_overlap(
        body,
        handle,
        elem_a="pivot_pin_1",
        elem_b="pivot_sleeve_1",
        reason="The modeled handle sleeve is a solid proxy intentionally captured around the fixed pivot pin.",
    )
    ctx.expect_within(
        body,
        handle,
        axes="yz",
        inner_elem="pivot_pin_0",
        outer_elem="pivot_sleeve_0",
        margin=0.001,
        name="handle pin 0 is captured inside sleeve",
    )
    ctx.expect_within(
        body,
        handle,
        axes="yz",
        inner_elem="pivot_pin_1",
        outer_elem="pivot_sleeve_1",
        margin=0.001,
        name="handle pin 1 is captured inside sleeve",
    )
    ctx.expect_overlap(
        body,
        handle,
        axes="x",
        elem_a="pivot_pin_0",
        elem_b="pivot_sleeve_0",
        min_overlap=0.020,
        name="handle pin 0 has retained axial engagement",
    )
    ctx.expect_overlap(
        body,
        handle,
        axes="x",
        elem_a="pivot_pin_1",
        elem_b="pivot_sleeve_1",
        min_overlap=0.020,
        name="handle pin 1 has retained axial engagement",
    )

    ctx.allow_overlap(
        body,
        volume_knob,
        elem_a="control_panel",
        elem_b="knob_shaft",
        reason="The short knob shaft is intentionally seated inside the raised control-panel boss.",
    )
    ctx.expect_within(
        volume_knob,
        body,
        axes="xz",
        inner_elem="knob_shaft",
        outer_elem="control_panel",
        margin=0.001,
        name="volume knob shaft is centered in the control boss",
    )
    ctx.expect_overlap(
        volume_knob,
        body,
        axes="y",
        elem_a="knob_shaft",
        elem_b="control_panel",
        min_overlap=0.006,
        name="volume knob shaft is inserted into the control boss",
    )

    stowed_grip = ctx.part_element_world_aabb(handle, elem="top_grip")
    with ctx.pose({handle_joint: 1.35}):
        raised_grip = ctx.part_element_world_aabb(handle, elem="top_grip")
    ctx.check(
        "folding handle rotates upward",
        stowed_grip is not None
        and raised_grip is not None
        and float(raised_grip[1][2]) > float(stowed_grip[1][2]) + 0.060,
        details=f"stowed={stowed_grip}, raised={raised_grip}",
    )

    rest_pointer = ctx.part_element_world_aabb(volume_knob, elem="pointer_line")
    with ctx.pose({knob_joint: 1.2}):
        turned_pointer = ctx.part_element_world_aabb(volume_knob, elem="pointer_line")
    if rest_pointer is not None and turned_pointer is not None:
        rest_center_x = 0.5 * (float(rest_pointer[0][0]) + float(rest_pointer[1][0]))
        turned_center_x = 0.5 * (float(turned_pointer[0][0]) + float(turned_pointer[1][0]))
        moved = abs(turned_center_x - rest_center_x) > 0.006
    else:
        moved = False
    ctx.check(
        "volume knob rotates its pointer",
        moved,
        details=f"rest={rest_pointer}, turned={turned_pointer}",
    )

    return ctx.report()


object_model = build_object_model()
