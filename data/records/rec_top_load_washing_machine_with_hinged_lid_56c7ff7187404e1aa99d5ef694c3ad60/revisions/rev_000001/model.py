from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.72
BODY_D = 0.74
BODY_H = 0.88
OPENING_Y = -0.02
HINGE_Y = 0.268
HINGE_Z = 0.925


def _cabinet_body_shape() -> cq.Workplane:
    """Rounded washer cabinet with a real through-cut top loading chamber."""
    outer = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H)
        .translate((0.0, 0.0, BODY_H / 2.0))
        .edges("|Z")
        .fillet(0.035)
    )
    cavity_cutter = (
        cq.Workplane("XY")
        .ellipse(0.292, 0.282)
        .extrude(BODY_H + 0.22)
        .translate((0.0, OPENING_Y, 0.055))
    )
    return outer.cut(cavity_cutter)


def _rounded_box_shape(size: tuple[float, float, float], radius: float, offset=(0.0, 0.0, 0.0)) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size).translate(offset)
    return shape.edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_top_load_washer")

    enamel = model.material("warm_white_enamel", rgba=(0.88, 0.90, 0.88, 1.0))
    white_plastic = model.material("white_molded_plastic", rgba=(0.96, 0.97, 0.95, 1.0))
    dark = model.material("soft_black", rgba=(0.015, 0.017, 0.020, 1.0))
    panel_black = model.material("gloss_black_panel", rgba=(0.02, 0.025, 0.032, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.66, 0.68, 0.66, 1.0))
    glass = model.material("smoked_glass", rgba=(0.08, 0.14, 0.18, 0.38))
    blue = model.material("blue_status_marker", rgba=(0.10, 0.32, 0.82, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_body_shape(), "cabinet_hollow_body", tolerance=0.002),
        material=enamel,
        name="body_shell",
    )
    cabinet.visual(
        Box((0.56, 0.018, 0.070)),
        origin=Origin(xyz=(0.0, -0.372, 0.070)),
        material=dark,
        name="recessed_toe_kick",
    )
    cabinet.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.535, 0.515),
                (0.650, 0.625),
                0.012,
                opening_shape="ellipse",
                outer_shape="rounded_rect",
                outer_corner_radius=0.055,
            ),
            "opening_gasket",
        ),
        origin=Origin(xyz=(0.0, OPENING_Y, BODY_H + 0.006)),
        material=dark,
        name="opening_gasket",
    )
    cabinet.visual(
        mesh_from_cadquery(
            _rounded_box_shape((0.740, 0.120, 0.270), 0.018, (0.0, 0.410, 1.015)),
            "rear_console",
            tolerance=0.0015,
        ),
        material=enamel,
        name="rear_console",
    )
    cabinet.visual(
        Box((0.635, 0.008, 0.165)),
        origin=Origin(xyz=(0.0, 0.346, 1.040)),
        material=panel_black,
        name="console_panel",
    )
    for x, seat_name in ((-0.235, "hinge_seat_0"), (0.235, "hinge_seat_1")):
        cabinet.visual(
            Box((0.105, 0.024, 0.070)),
            origin=Origin(xyz=(x, HINGE_Y + 0.028, 0.915)),
            material=white_plastic,
            name=seat_name,
        )
    cabinet.visual(
        Cylinder(radius=0.045, length=0.070),
        origin=Origin(xyz=(0.0, OPENING_Y, 0.090)),
        material=dark,
        name="drive_bearing",
    )

    tub = model.part("wash_tub")
    tub_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.235, 0.135),
            (0.248, 0.180),
            (0.255, 0.790),
            (0.245, 0.825),
        ],
        inner_profile=[
            (0.195, 0.150),
            (0.214, 0.190),
            (0.224, 0.780),
            (0.217, 0.810),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )
    tub.visual(mesh_from_geometry(tub_shell, "stainless_wash_basket"), material=stainless, name="basket_shell")
    tub.visual(Cylinder(radius=0.205, length=0.022), origin=Origin(xyz=(0.0, 0.0, 0.150)), material=stainless, name="basket_floor")
    tub.visual(Cylinder(radius=0.024, length=0.050), origin=Origin(xyz=(0.0, 0.0, 0.150)), material=stainless, name="drive_shaft")
    tub.visual(mesh_from_geometry(TorusGeometry(0.238, 0.0075), "basket_rolled_rim"), origin=Origin(xyz=(0.0, 0.0, 0.824)), material=stainless, name="rolled_rim")
    tub.visual(Cylinder(radius=0.055, length=0.045), origin=Origin(xyz=(0.0, 0.0, 0.183)), material=stainless, name="impeller_hub")
    for i in range(3):
        tub.visual(
            Box((0.170, 0.030, 0.020)),
            origin=Origin(xyz=(0.060, 0.0, 0.178), rpy=(0.0, 0.0, i * 2.0 * math.pi / 3.0)),
            material=stainless,
            name=f"impeller_fin_{i}",
        )
    tub.visual(
        Box((0.036, 0.012, 0.026)),
        origin=Origin(xyz=(0.246, 0.0, 0.826)),
        material=blue,
        name="rim_marker",
    )
    model.articulation(
        "cabinet_to_tub",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, OPENING_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=8.0),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.405, 0.340),
                (0.625, 0.580),
                0.034,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.060,
                outer_corner_radius=0.070,
            ),
            "lid_window_frame",
        ),
        origin=Origin(xyz=(0.0, -0.290, -0.009)),
        material=white_plastic,
        name="lid_frame",
    )
    lid.visual(
        Box((0.425, 0.360, 0.006)),
        origin=Origin(xyz=(0.0, -0.290, 0.006)),
        material=glass,
        name="glass_window",
    )
    lid.visual(
        Box((0.245, 0.048, 0.030)),
        origin=Origin(xyz=(0.0, -0.596, 0.004)),
        material=white_plastic,
        name="front_handle",
    )
    lid.visual(
        Box((0.160, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, -0.610, -0.006)),
        material=dark,
        name="handle_shadow",
    )
    for x, leaf_name, barrel_name in (
        (-0.235, "hinge_leaf_0", "hinge_barrel_0"),
        (0.235, "hinge_leaf_1", "hinge_barrel_1"),
    ):
        lid.visual(
            Box((0.135, 0.040, 0.008)),
            origin=Origin(xyz=(x, -0.018, -0.013)),
            material=white_plastic,
            name=leaf_name,
        )
        lid.visual(
            Cylinder(radius=0.012, length=0.105),
            origin=Origin(xyz=(x, 0.004, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name=barrel_name,
        )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.82, effort=22.0, velocity=1.4),
    )

    dial = model.part("selector_dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.112,
                0.040,
                body_style="skirted",
                top_diameter=0.086,
                skirt=KnobSkirt(0.132, 0.008, flare=0.06, chamfer=0.0015),
                grip=KnobGrip(style="fluted", count=28, depth=0.0018),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "selector_dial_knob",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="dial_knob",
    )
    dial.visual(
        Box((0.010, 0.003, 0.036)),
        origin=Origin(xyz=(0.0, -0.0415, 0.023)),
        material=dark,
        name="dial_pointer",
    )
    model.articulation(
        "console_to_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(0.0, 0.342, 1.045)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
    )

    button_mesh = mesh_from_cadquery(
        _rounded_box_shape((0.064, 0.016, 0.032), 0.004, (0.0, -0.008, 0.0)),
        "program_button_cap",
        tolerance=0.0008,
    )
    button_positions = [
        (-0.295, 1.070),
        (-0.215, 1.070),
        (-0.135, 1.070),
        (0.135, 1.070),
        (0.215, 1.070),
        (0.295, 1.070),
    ]
    for idx, (x, z) in enumerate(button_positions):
        button = model.part(f"program_button_{idx}")
        button.visual(button_mesh, material=white_plastic, name="button_cap")
        model.articulation(
            f"console_to_program_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, 0.342, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.007, effort=4.0, velocity=0.08),
        )

    lid_button = model.part("lid_button")
    lid_button.visual(
        mesh_from_cadquery(
            _rounded_box_shape((0.150, 0.034, 0.014), 0.005, (0.0, 0.0, 0.007)),
            "lid_release_button",
            tolerance=0.0008,
        ),
        material=white_plastic,
        name="button_cap",
    )
    model.articulation(
        "deck_to_lid_button",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lid_button,
        origin=Origin(xyz=(0.0, -0.352, BODY_H)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.010, effort=5.0, velocity=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("wash_tub")
    dial = object_model.get_part("selector_dial")
    lid_button = object_model.get_part("lid_button")

    lid_joint = object_model.get_articulation("cabinet_to_lid")
    tub_joint = object_model.get_articulation("cabinet_to_tub")
    dial_joint = object_model.get_articulation("console_to_dial")
    lid_button_joint = object_model.get_articulation("deck_to_lid_button")
    program_joints = [
        object_model.get_articulation(f"console_to_program_button_{i}") for i in range(6)
    ]
    program_parts = [object_model.get_part(f"program_button_{i}") for i in range(6)]

    ctx.check(
        "six separate program push buttons",
        len(program_parts) == 6 and len(program_joints) == 6,
        details=f"parts={program_parts}, joints={program_joints}",
    )
    ctx.check(
        "primary motions have correct joint types",
        lid_joint.articulation_type == ArticulationType.REVOLUTE
        and tub_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and lid_button_joint.articulation_type == ArticulationType.PRISMATIC
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in program_joints),
        details=(
            f"lid={lid_joint.articulation_type}, tub={tub_joint.articulation_type}, "
            f"dial={dial_joint.articulation_type}, lid_button={lid_button_joint.articulation_type}"
        ),
    )
    ctx.check(
        "push controls have short appliance-button travel",
        all(0.005 <= j.motion_limits.upper <= 0.012 for j in program_joints)
        and 0.008 <= lid_button_joint.motion_limits.upper <= 0.014,
        details=f"program={[j.motion_limits.upper for j in program_joints]}, lid={lid_button_joint.motion_limits.upper}",
    )

    ctx.expect_within(
        tub,
        cabinet,
        axes="xy",
        inner_elem="basket_shell",
        outer_elem="opening_gasket",
        margin=0.010,
        name="rotating basket sits inside the top loading opening",
    )
    ctx.expect_gap(
        cabinet,
        tub,
        axis="z",
        positive_elem="opening_gasket",
        negative_elem="rolled_rim",
        min_gap=0.040,
        max_gap=0.075,
        name="basket rim is visibly below the deck for chamber depth",
    )
    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        positive_elem="lid_frame",
        negative_elem="opening_gasket",
        min_gap=0.002,
        max_gap=0.015,
        name="closed lid clears the raised opening gasket",
    )
    ctx.expect_overlap(
        lid,
        cabinet,
        axes="xy",
        elem_a="glass_window",
        elem_b="opening_gasket",
        min_overlap=0.30,
        name="glass window spans the laundry chamber opening",
    )
    ctx.expect_contact(
        lid,
        cabinet,
        elem_a="hinge_barrel_0",
        elem_b="hinge_seat_0",
        contact_tol=0.002,
        name="rear hinge barrel is carried by the cabinet hinge seat",
    )
    ctx.expect_contact(
        tub,
        cabinet,
        elem_a="drive_shaft",
        elem_b="drive_bearing",
        contact_tol=0.002,
        name="rotating tub is supported on the drive bearing",
    )
    ctx.expect_contact(
        program_parts[0],
        cabinet,
        elem_a="button_cap",
        elem_b="console_panel",
        contact_tol=0.002,
        name="program buttons mount flush to the console panel",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    closed_handle = _center_from_aabb(ctx.part_element_world_aabb(lid, elem="front_handle"))
    with ctx.pose({lid_joint: 1.20}):
        raised_handle = _center_from_aabb(ctx.part_element_world_aabb(lid, elem="front_handle"))
    ctx.check(
        "lid hinge lifts the front handle upward",
        closed_handle is not None
        and raised_handle is not None
        and raised_handle[2] > closed_handle[2] + 0.30,
        details=f"closed={closed_handle}, raised={raised_handle}",
    )

    tub_marker_rest = _center_from_aabb(ctx.part_element_world_aabb(tub, elem="rim_marker"))
    with ctx.pose({tub_joint: math.pi / 2.0}):
        tub_marker_rotated = _center_from_aabb(ctx.part_element_world_aabb(tub, elem="rim_marker"))
    ctx.check(
        "wash tub continuous joint rotates the visible rim marker",
        tub_marker_rest is not None
        and tub_marker_rotated is not None
        and abs(tub_marker_rotated[0] - tub_marker_rest[0]) > 0.15
        and tub_marker_rotated[1] > tub_marker_rest[1] + 0.15,
        details=f"rest={tub_marker_rest}, rotated={tub_marker_rotated}",
    )

    dial_pointer_rest = _center_from_aabb(ctx.part_element_world_aabb(dial, elem="dial_pointer"))
    with ctx.pose({dial_joint: math.pi / 2.0}):
        dial_pointer_rotated = _center_from_aabb(ctx.part_element_world_aabb(dial, elem="dial_pointer"))
    ctx.check(
        "selector dial continuous joint turns the pointer",
        dial_pointer_rest is not None
        and dial_pointer_rotated is not None
        and dial_pointer_rotated[0] > dial_pointer_rest[0] + 0.015,
        details=f"rest={dial_pointer_rest}, rotated={dial_pointer_rotated}",
    )

    button_rest = ctx.part_world_position(program_parts[0])
    with ctx.pose({program_joints[0]: program_joints[0].motion_limits.upper}):
        button_pressed = ctx.part_world_position(program_parts[0])
    ctx.check(
        "program button slides inward into console",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[1] > button_rest[1] + 0.005,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    lid_button_rest = ctx.part_world_position(lid_button)
    with ctx.pose({lid_button_joint: lid_button_joint.motion_limits.upper}):
        lid_button_pressed = ctx.part_world_position(lid_button)
    ctx.check(
        "front lid-release button depresses into the top deck",
        lid_button_rest is not None
        and lid_button_pressed is not None
        and lid_button_pressed[2] < lid_button_rest[2] - 0.007,
        details=f"rest={lid_button_rest}, pressed={lid_button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
