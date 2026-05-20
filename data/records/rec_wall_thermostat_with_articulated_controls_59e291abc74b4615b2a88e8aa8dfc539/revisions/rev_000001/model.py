from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    CylinderGeometry,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, *, segments: int = 72):
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=segments)
    inner = CylinderGeometry(radius=inner_radius, height=height + 0.006, radial_segments=segments)
    return boolean_difference(outer, inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_wall_thermostat")

    warm_gray = Material("powder_coated_warm_gray", rgba=(0.72, 0.70, 0.64, 1.0))
    dark_gray = Material("dark_service_plastic", rgba=(0.10, 0.11, 0.11, 1.0))
    black_rubber = Material("replaceable_black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    zinc = Material("zinc_plated_steel", rgba=(0.62, 0.61, 0.56, 1.0))
    dial_gray = Material("grippy_dial_gray", rgba=(0.27, 0.28, 0.27, 1.0))
    white_mark = Material("painted_white_markings", rgba=(0.92, 0.90, 0.82, 1.0))
    amber = Material("amber_service_window", rgba=(0.95, 0.48, 0.10, 1.0))
    brass = Material("brass_terminal_block", rgba=(0.77, 0.55, 0.22, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.196, 0.258, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=zinc,
        name="wall_backplate",
    )
    body.visual(
        Box((0.168, 0.226, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=warm_gray,
        name="sealed_housing",
    )
    body.visual(
        Box((0.154, 0.212, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=warm_gray,
        name="front_service_face",
    )

    # Replaceable wear strips and proud corner guards make the field-service
    # housing read as a tool-room workhorse rather than a fragile consumer shell.
    for x, name in ((-0.088, "side_wear_strip_0"), (0.088, "side_wear_strip_1")):
        body.visual(
            Box((0.010, 0.222, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.053)),
            material=black_rubber,
            name=name,
        )
    body.visual(
        Box((0.166, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.112, 0.053)),
        material=black_rubber,
        name="end_wear_strip_1",
    )
    for x, name in ((-0.083, "end_wear_strip_0"), (0.083, "end_wear_strip_2")):
        body.visual(
            Box((0.030, 0.010, 0.014)),
            origin=Origin(xyz=(x, -0.112, 0.053)),
            material=black_rubber,
            name=name,
        )

    dial_bezel = BezelGeometry(
        (0.086, 0.086),
        (0.120, 0.120),
        0.007,
        opening_shape="circle",
        outer_shape="circle",
        face=BezelFace(style="radiused_step", front_lip=0.0025, fillet=0.0015),
        center=False,
    )
    body.visual(
        mesh_from_geometry(dial_bezel, "dial_bezel"),
        origin=Origin(xyz=(0.0, 0.028, 0.046)),
        material=dark_gray,
        name="dial_bezel",
    )
    body.visual(
        Cylinder(radius=0.0095, length=0.0265),
        origin=Origin(xyz=(0.0, 0.028, 0.05925)),
        material=zinc,
        name="center_shaft",
    )

    # Temperature index ticks are seated into the front face outside the bezel.
    for i, angle_deg in enumerate((-145, -115, -85, -55, -30, 30, 55, 85, 115, 145)):
        angle = angle_deg * pi / 180.0
        radius = 0.066
        x = radius * sin(angle)
        y = 0.028 + radius * cos(angle)
        tick_len = 0.014 if abs(angle_deg) in (85, 115) else 0.010
        body.visual(
            Box((0.0030, tick_len, 0.0030)),
            origin=Origin(xyz=(x, y, 0.0495), rpy=(0.0, 0.0, -angle)),
            material=white_mark,
            name=f"temperature_tick_{i}",
        )

    # A rugged mode slider track has real rails, end stops, and a service label.
    body.visual(
        Box((0.124, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, 0.098, 0.048)),
        material=dark_gray,
        name="slider_rail_panel",
    )
    for y, name in ((0.087, "slider_rail_0"), (0.109, "slider_rail_1")):
        body.visual(
            Box((0.112, 0.004, 0.009)),
            origin=Origin(xyz=(0.0, y, 0.0525)),
            material=black_rubber,
            name=name,
        )
    for x, name in ((-0.061, "slider_end_stop_0"), (0.061, "slider_end_stop_1")):
        body.visual(
            Box((0.006, 0.026, 0.009)),
            origin=Origin(xyz=(x, 0.098, 0.0525)),
            material=black_rubber,
            name=name,
        )

    # Maintenance bay details are visible when the lower cover is hinged down.
    body.visual(
        Box((0.118, 0.044, 0.0025)),
        origin=Origin(xyz=(0.0, -0.083, 0.04725)),
        material=dark_gray,
        name="service_bay_shadow",
    )
    body.visual(
        Box((0.104, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, -0.083, 0.044)),
        material=brass,
        name="terminal_block",
    )
    for x, name in ((-0.041, "terminal_screw_0"), (-0.014, "terminal_screw_1"), (0.014, "terminal_screw_2"), (0.041, "terminal_screw_3")):
        body.visual(
            Cylinder(radius=0.004, length=0.003),
            origin=Origin(xyz=(x, -0.083, 0.0475)),
            material=zinc,
            name=name,
        )

    # Exposed service-cover hinge knuckles.  Their segmented layout leaves
    # clearance for the moving center knuckle on the cover.
    for x, name in ((-0.050, "body_hinge_barrel_0"), (0.050, "body_hinge_barrel_1")):
        body.visual(
            Cylinder(radius=0.005, length=0.028),
            origin=Origin(xyz=(x, -0.121, 0.054), rpy=(0.0, pi / 2.0, 0.0)),
            material=zinc,
            name=name,
        )
        body.visual(
            Box((0.026, 0.012, 0.004)),
            origin=Origin(xyz=(x, -0.114, 0.048)),
            material=zinc,
            name=f"{name}_leaf",
        )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(_annular_cylinder(0.038, 0.0145, 0.0175), "main_dial"),
        origin=Origin(xyz=(0.0, 0.0, 0.00875)),
        material=dial_gray,
        name="dial_cap",
    )
    for index in range(32):
        angle = index * 2.0 * pi / 32.0
        x = 0.0385 * sin(angle)
        y = 0.0385 * cos(angle)
        dial.visual(
            Box((0.0024, 0.0060, 0.0120)),
            origin=Origin(xyz=(x, y, 0.009), rpy=(0.0, 0.0, -angle)),
            material=dial_gray,
            name=f"dial_grip_rib_{index}",
        )
    dial.visual(
        Box((0.004, 0.032, 0.0025)),
        origin=Origin(xyz=(0.0, 0.027, 0.0185)),
        material=white_mark,
        name="dial_pointer_bar",
    )

    retainer = model.part("retainer")
    retainer.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=zinc,
        name="retainer_washer",
    )
    retainer.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
        material=dark_gray,
        name="retainer_bolt_head",
    )

    mode_slider = model.part("mode_slider")
    mode_slider.visual(
        Box((0.028, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=black_rubber,
        name="slider_cap",
    )
    mode_slider.visual(
        Box((0.018, 0.003, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
        material=white_mark,
        name="slider_index_line",
    )

    access_panel = model.part("access_panel")
    access_panel.visual(
        Box((0.132, 0.067, 0.008)),
        origin=Origin(xyz=(0.0, 0.0395, 0.004)),
        material=warm_gray,
        name="access_panel_face",
    )
    access_panel.visual(
        Box((0.112, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.058, 0.010)),
        material=black_rubber,
        name="replaceable_door_gasket",
    )
    access_panel.visual(
        Box((0.050, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.035, 0.011)),
        material=dark_gray,
        name="pull_rib",
    )
    access_panel.visual(
        Cylinder(radius=0.005, length=0.073),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=zinc,
        name="panel_hinge_barrel",
    )
    access_panel.visual(
        Box((0.050, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.007, 0.001)),
        material=zinc,
        name="panel_hinge_leaf",
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=zinc,
        name="latch_quarter_turn_head",
    )
    latch.visual(
        Box((0.014, 0.003, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=dark_gray,
        name="latch_tool_slot",
    )
    latch.visual(
        Box((0.011, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, -0.007, 0.0045)),
        material=amber,
        name="latch_cam_flag",
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.028, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.8, velocity=2.5, lower=-2.35, upper=2.35),
        motion_properties=MotionProperties(damping=0.04, friction=0.09),
    )
    model.articulation(
        "body_to_retainer",
        ArticulationType.FIXED,
        parent=body,
        child=retainer,
        origin=Origin(xyz=(0.0, 0.028, 0.0725)),
    )
    model.articulation(
        "body_to_mode_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_slider,
        origin=Origin(xyz=(0.0, 0.098, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.18, lower=-0.030, upper=0.030),
        motion_properties=MotionProperties(damping=0.08, friction=0.4),
    )
    model.articulation(
        "body_to_access_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=access_panel,
        origin=Origin(xyz=(0.0, -0.121, 0.054)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.3, lower=0.0, upper=1.35),
        motion_properties=MotionProperties(damping=0.12, friction=0.18),
    )
    model.articulation(
        "access_panel_to_latch",
        ArticulationType.REVOLUTE,
        parent=access_panel,
        child=latch,
        origin=Origin(xyz=(0.046, 0.052, 0.008)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.9, velocity=2.0, lower=-1.57, upper=1.57),
        motion_properties=MotionProperties(damping=0.02, friction=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    retainer = object_model.get_part("retainer")
    mode_slider = object_model.get_part("mode_slider")
    access_panel = object_model.get_part("access_panel")
    latch = object_model.get_part("latch")
    dial_joint = object_model.get_articulation("body_to_dial")
    slider_joint = object_model.get_articulation("body_to_mode_slider")
    panel_joint = object_model.get_articulation("body_to_access_panel")

    ctx.expect_within(
        dial,
        body,
        axes="xy",
        inner_elem="dial_cap",
        outer_elem="dial_bezel",
        margin=0.002,
        name="dial is captured inside circular bezel clearance",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="z",
        min_gap=0.0003,
        max_gap=0.003,
        positive_elem="dial_cap",
        negative_elem="dial_bezel",
        name="dial rear face has robust bezel running clearance",
    )
    ctx.expect_contact(
        retainer,
        dial,
        elem_a="retainer_washer",
        elem_b="dial_cap",
        contact_tol=0.0005,
        name="retainer washer lightly captures rotating dial face",
    )
    ctx.expect_contact(
        retainer,
        body,
        elem_a="retainer_washer",
        elem_b="center_shaft",
        contact_tol=0.0005,
        name="retainer washer seats on fixed center shaft",
    )
    ctx.expect_gap(
        mode_slider,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="slider_cap",
        negative_elem="slider_rail_panel",
        name="mode slider cap rides on the service rail panel",
    )
    ctx.expect_gap(
        access_panel,
        body,
        axis="z",
        min_gap=0.003,
        max_gap=0.008,
        positive_elem="access_panel_face",
        negative_elem="front_service_face",
        name="access cover stands proud of service bay at rest",
    )
    ctx.expect_contact(
        latch,
        access_panel,
        elem_a="latch_quarter_turn_head",
        elem_b="access_panel_face",
        contact_tol=0.0005,
        name="quarter turn latch head is seated on access cover",
    )

    slider_rest = ctx.part_world_position(mode_slider)
    with ctx.pose({slider_joint: 0.030}):
        slider_extended = ctx.part_world_position(mode_slider)
    ctx.check(
        "mode slider travels horizontally in rails",
        slider_rest is not None
        and slider_extended is not None
        and slider_extended[0] > slider_rest[0] + 0.025,
        details=f"rest={slider_rest}, extended={slider_extended}",
    )

    closed_aabb = ctx.part_element_world_aabb(access_panel, elem="access_panel_face")
    with ctx.pose({panel_joint: 1.20}):
        open_aabb = ctx.part_element_world_aabb(access_panel, elem="access_panel_face")
    ctx.check(
        "service cover hinges outward and downward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.040
        and open_aabb[1][1] < closed_aabb[1][1] - 0.010,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    with ctx.pose({dial_joint: 1.0}):
        ctx.expect_within(
            dial,
            body,
            axes="xy",
            inner_elem="dial_cap",
            outer_elem="dial_bezel",
            margin=0.002,
            name="rotated dial remains centered in bezel",
        )

    return ctx.report()


object_model = build_object_model()
