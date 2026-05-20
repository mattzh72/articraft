from __future__ import annotations

import math

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
    MeshGeometry,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _trapezoid_plate_mesh(x_center: float, thickness: float) -> MeshGeometry:
    """Thin side closing plate for the hood canopy."""
    # Side profile in the hood's Y/Z plane: front lower, front fascia top,
    # rear high back, rear lower return.
    profile_yz = [
        (-0.50, 0.08),
        (-0.50, 0.32),
        (0.22, 0.55),
        (0.22, 0.02),
    ]
    xs = [x_center - thickness / 2.0, x_center + thickness / 2.0]
    geom = MeshGeometry()
    for x in xs:
        for y, z in profile_yz:
            geom.add_vertex(x, y, z)
    # Faces for the two trapezoid sides and the four narrow edge strips.
    geom.add_face(0, 1, 2)
    geom.add_face(0, 2, 3)
    geom.add_face(4, 6, 5)
    geom.add_face(4, 7, 6)
    for i in range(4):
        j = (i + 1) % 4
        geom.add_face(i, j, 4 + j)
        geom.add_face(i, 4 + j, 4 + i)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_wall_hood")

    stainless = model.material("brushed_stainless", rgba=(0.66, 0.68, 0.66, 1.0))
    filter_metal = model.material("baffle_stainless", rgba=(0.52, 0.54, 0.52, 1.0))
    dark = model.material("black_phenolic", rgba=(0.015, 0.014, 0.013, 1.0))
    shadow = model.material("dark_recess", rgba=(0.05, 0.055, 0.055, 1.0))
    label = model.material("engraved_marks", rgba=(0.01, 0.01, 0.009, 1.0))

    width = 1.20
    half_width = width / 2.0
    front_y = -0.50
    fascia_front_y = front_y - 0.0175

    canopy = model.part("canopy")
    # Fixed stainless hood body: a sloped upper shell, a deep vertical front
    # fascia, a wall mounting plate, side closing cheeks, and a chimney duct.
    canopy.visual(
        Box((1.22, 0.035, 0.22)),
        origin=Origin(xyz=(0.0, front_y, 0.20)),
        material=stainless,
        name="front_fascia",
    )
    canopy.visual(
        Box((1.22, 0.035, 0.54)),
        origin=Origin(xyz=(0.0, 0.225, 0.28)),
        material=stainless,
        name="wall_back",
    )

    top_angle = math.atan2(0.25, 0.72)
    canopy.visual(
        Box((1.22, 0.765, 0.032)),
        origin=Origin(xyz=(0.0, -0.14, 0.425), rpy=(top_angle, 0.0, 0.0)),
        material=stainless,
        name="sloped_canopy",
    )
    canopy.visual(
        Box((1.22, 0.070, 0.055)),
        origin=Origin(xyz=(0.0, -0.487, 0.075)),
        material=stainless,
        name="grease_trough",
    )
    canopy.visual(
        mesh_from_geometry(_trapezoid_plate_mesh(-half_width - 0.006, 0.024), "side_cheek_0"),
        material=stainless,
        name="side_cheek_0",
    )
    canopy.visual(
        mesh_from_geometry(_trapezoid_plate_mesh(half_width + 0.006, 0.024), "side_cheek_1"),
        material=stainless,
        name="side_cheek_1",
    )
    canopy.visual(
        Box((0.42, 0.24, 0.68)),
        origin=Origin(xyz=(0.0, 0.035, 0.855)),
        material=stainless,
        name="chimney_duct",
    )
    canopy.visual(
        Box((0.48, 0.28, 0.035)),
        origin=Origin(xyz=(0.0, 0.035, 0.535)),
        material=stainless,
        name="chimney_flange",
    )
    canopy.visual(
        Box((1.02, 0.058, 0.016)),
        origin=Origin(xyz=(0.0, 0.196, 0.052)),
        material=stainless,
        name="fixed_hinge_leaf",
    )
    # Small fixed tick marks above each control position make the front fascia
    # read as a commercial control strip while the actual knobs remain separate
    # articulated parts.
    knob_xs = (-0.40, -0.20, 0.0, 0.20, 0.40)
    for index, x in enumerate(knob_xs):
        canopy.visual(
            Box((0.010, 0.004, 0.030)),
            origin=Origin(xyz=(x, fascia_front_y - 0.001, 0.262)),
            material=label,
            name=f"knob_mark_{index}",
        )

    access_panel = model.part("access_panel")
    # The access panel is a hinged rectangular stainless carrier frame. Its
    # local frame sits on the hinge axis; the panel extends toward local -Y.
    access_panel.visual(
        Cylinder(radius=0.014, length=1.02),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )
    access_panel.visual(
        Box((1.08, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, -0.020, -0.015)),
        material=stainless,
        name="rear_frame",
    )
    access_panel.visual(
        Box((1.08, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, -0.520, -0.015)),
        material=stainless,
        name="front_frame",
    )
    access_panel.visual(
        Box((0.040, 0.520, 0.030)),
        origin=Origin(xyz=(-0.540, -0.270, -0.015)),
        material=stainless,
        name="side_frame_0",
    )
    access_panel.visual(
        Box((0.040, 0.520, 0.030)),
        origin=Origin(xyz=(0.540, -0.270, -0.015)),
        material=stainless,
        name="side_frame_1",
    )
    access_panel.visual(
        Box((0.036, 0.490, 0.030)),
        origin=Origin(xyz=(0.0, -0.270, -0.015)),
        material=stainless,
        name="center_divider",
    )
    access_panel.visual(
        Box((0.15, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.545, -0.018)),
        material=shadow,
        name="pull_lip",
    )
    # Four spring-like retainer tabs reach in from the fixed frame and touch the
    # top edge of each removable filter cassette so the filters visibly sit in
    # a carried frame rather than floating below it.
    for x in (-0.500, -0.039, 0.039, 0.500):
        access_panel.visual(
            Box((0.042, 0.105, 0.010)),
            origin=Origin(xyz=(x, -0.270, -0.035)),
            material=stainless,
            name=f"filter_clip_{len(access_panel.visuals)}",
        )

    panel_hinge = model.articulation(
        "canopy_to_access_panel",
        ArticulationType.REVOLUTE,
        parent=canopy,
        child=access_panel,
        origin=Origin(xyz=(0.0, 0.170, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.18),
    )

    # Removable slotted baffle filter cassettes ride in the hinged frame and
    # drop out along the panel normal for cleaning.
    filter_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.455, 0.370),
            0.012,
            slot_size=(0.060, 0.010),
            pitch=(0.075, 0.030),
            frame=0.020,
            corner_radius=0.004,
            slot_angle_deg=18.0,
            stagger=True,
        ),
        "baffle_filter_face",
    )
    filter_origins = (-0.270, 0.270)
    for index, x in enumerate(filter_origins):
        filt = model.part(f"filter_{index}")
        filt.visual(
            filter_mesh,
            origin=Origin(),
            material=filter_metal,
            name="filter_face",
        )
        filt.visual(
            Box((0.126, 0.018, 0.014)),
            origin=Origin(xyz=(0.0, -0.125, -0.026)),
            material=filter_metal,
            name="finger_bar",
        )
        filt.visual(
            Box((0.018, 0.018, 0.024)),
            origin=Origin(xyz=(-0.052, -0.125, -0.016)),
            material=filter_metal,
            name="handle_post_0",
        )
        filt.visual(
            Box((0.018, 0.018, 0.024)),
            origin=Origin(xyz=(0.052, -0.125, -0.016)),
            material=filter_metal,
            name="handle_post_1",
        )
        model.articulation(
            f"panel_to_filter_{index}",
            ArticulationType.PRISMATIC,
            parent=access_panel,
            child=filt,
            origin=Origin(xyz=(x, -0.270, -0.046)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=12.0, velocity=0.25, lower=0.0, upper=0.090),
        )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.030,
            body_style="skirted",
            top_diameter=0.040,
            skirt=KnobSkirt(0.064, 0.006, flare=0.08, chamfer=0.0012),
            grip=KnobGrip(style="fluted", count=20, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
        ),
        "control_knob",
    )
    for index, x in enumerate(knob_xs):
        knob = model.part(f"knob_{index}")
        knob.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0, -0.015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark,
            name="knob_cap",
        )
        knob.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(0.0, 0.0015, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=shadow,
            name="shaft_stub",
        )
        model.articulation(
            f"fascia_to_knob_{index}",
            ArticulationType.REVOLUTE,
            parent=canopy,
            child=knob,
            origin=Origin(xyz=(x, fascia_front_y - 0.0065, 0.205)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.9, velocity=5.0, lower=0.0, upper=4.71),
        )

    # Keep references alive for tests by name, without returning them.
    _ = panel_hinge
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    panel = object_model.get_part("access_panel")
    hinge = object_model.get_articulation("canopy_to_access_panel")

    # The panel should sit under the hood body and open by dropping its front
    # edge downward around a real hinge line.
    rest_front = ctx.part_element_world_aabb(panel, elem="front_frame")
    with ctx.pose({hinge: 1.0}):
        open_front = ctx.part_element_world_aabb(panel, elem="front_frame")
    ctx.check(
        "access panel swings downward",
        rest_front is not None
        and open_front is not None
        and open_front[0][2] < rest_front[0][2] - 0.20,
        details=f"rest={rest_front}, open={open_front}",
    )

    for index in range(2):
        filt = object_model.get_part(f"filter_{index}")
        slider = object_model.get_articulation(f"panel_to_filter_{index}")
        ctx.expect_within(
            filt,
            panel,
            axes="xy",
            margin=0.002,
            inner_elem="filter_face",
            name=f"filter {index} is seated inside the access frame",
        )
        rest_pos = ctx.part_world_position(filt)
        with ctx.pose({slider: 0.090}):
            dropped_pos = ctx.part_world_position(filt)
        ctx.check(
            f"filter {index} drops out for removal",
            rest_pos is not None and dropped_pos is not None and dropped_pos[2] < rest_pos[2] - 0.080,
            details=f"rest={rest_pos}, dropped={dropped_pos}",
        )

    knob_spans_ok = True
    for index in range(5):
        knob = object_model.get_part(f"knob_{index}")
        joint = object_model.get_articulation(f"fascia_to_knob_{index}")
        limits = joint.motion_limits
        knob_spans_ok = knob_spans_ok and limits is not None and limits.upper - limits.lower > 4.0
        ctx.expect_gap(
            canopy,
            knob,
            axis="y",
            min_gap=0.0,
            max_gap=0.004,
            positive_elem="front_fascia",
            negative_elem="knob_cap",
            name=f"knob {index} sits proud of the front fascia",
        )
    ctx.check("all five knobs have rotary travel", knob_spans_ok)

    return ctx.report()


object_model = build_object_model()
