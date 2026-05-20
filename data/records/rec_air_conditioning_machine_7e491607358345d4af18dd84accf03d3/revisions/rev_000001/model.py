from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

PANEL_SIZE = 0.95
PANEL_THICKNESS = 0.028
CHASSIS_SIZE = 0.68
CHASSIS_HEIGHT = 0.27

INTAKE_OPENING = 0.488
INTAKE_GRILLE_OUTER = 0.492
INTAKE_GRILLE_THICKNESS = 0.006
INTAKE_GRILLE_FRAME = 0.024
INTAKE_BAR_WIDTH = 0.012
INTAKE_BAR_COUNT = 5

OUTLET_LENGTH = 0.50
OUTLET_DEPTH = 0.105
OUTLET_GAP = 0.015
OUTLET_CENTER_OFFSET = INTAKE_GRILLE_OUTER / 2.0 + OUTLET_GAP + OUTLET_DEPTH / 2.0

FLAP_THICKNESS = 0.01
FLAP_LENGTH = OUTLET_LENGTH - 0.012
FLAP_DEPTH = OUTLET_DEPTH - 0.012
FLAP_SWING = 1.1

BUTTON_CAP_SIZE = (0.018, 0.012, 0.003)
BUTTON_STEM_HEIGHT = 0.009
BUTTON_SLOT = (0.021, 0.015)
BUTTON_TRAVEL = 0.0035
BUTTON_Y = -0.432
BUTTON_XS = (0.334, 0.366)


def _cut_box(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(xyz)


def _build_panel_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(PANEL_SIZE, PANEL_SIZE, PANEL_THICKNESS).translate(
        (0.0, 0.0, PANEL_THICKNESS / 2.0)
    )

    full_cut_height = PANEL_THICKNESS + 0.02
    shell = shell.cut(_cut_box((INTAKE_OPENING, INTAKE_OPENING, full_cut_height), (0.0, 0.0, PANEL_THICKNESS / 2.0)))
    shell = shell.cut(
        _cut_box(
            (OUTLET_LENGTH, OUTLET_DEPTH, full_cut_height),
            (0.0, -OUTLET_CENTER_OFFSET, PANEL_THICKNESS / 2.0),
        )
    )
    shell = shell.cut(
        _cut_box(
            (OUTLET_LENGTH, OUTLET_DEPTH, full_cut_height),
            (0.0, OUTLET_CENTER_OFFSET, PANEL_THICKNESS / 2.0),
        )
    )
    shell = shell.cut(
        _cut_box(
            (OUTLET_DEPTH, OUTLET_LENGTH, full_cut_height),
            (-OUTLET_CENTER_OFFSET, 0.0, PANEL_THICKNESS / 2.0),
        )
    )
    shell = shell.cut(
        _cut_box(
            (OUTLET_DEPTH, OUTLET_LENGTH, full_cut_height),
            (OUTLET_CENTER_OFFSET, 0.0, PANEL_THICKNESS / 2.0),
        )
    )

    for button_x in BUTTON_XS:
        shell = shell.cut(
            _cut_box(
                (BUTTON_SLOT[0], BUTTON_SLOT[1], full_cut_height),
                (button_x, BUTTON_Y, PANEL_THICKNESS / 2.0),
            )
        )

    return shell


def _build_intake_grille() -> cq.Workplane:
    grille = cq.Workplane("XY").box(
        INTAKE_GRILLE_OUTER,
        INTAKE_GRILLE_OUTER,
        INTAKE_GRILLE_THICKNESS,
    ).translate((0.0, 0.0, INTAKE_GRILLE_THICKNESS / 2.0))

    clear = INTAKE_GRILLE_OUTER - 2.0 * INTAKE_GRILLE_FRAME
    grille = grille.cut(
        _cut_box(
            (clear, clear, INTAKE_GRILLE_THICKNESS + 0.004),
            (0.0, 0.0, INTAKE_GRILLE_THICKNESS / 2.0),
        )
    )

    pitch = clear / (INTAKE_BAR_COUNT + 1)
    bar_span = clear + 0.002
    for i in range(INTAKE_BAR_COUNT):
        pos = -clear / 2.0 + (i + 1) * pitch
        grille = grille.union(
            cq.Workplane("XY")
            .box(INTAKE_BAR_WIDTH, bar_span, INTAKE_GRILLE_THICKNESS)
            .translate((pos, 0.0, INTAKE_GRILLE_THICKNESS / 2.0))
        )
        grille = grille.union(
            cq.Workplane("XY")
            .box(bar_span, INTAKE_BAR_WIDTH, INTAKE_GRILLE_THICKNESS)
            .translate((0.0, pos, INTAKE_GRILLE_THICKNESS / 2.0))
        )

    return grille


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_cassette_air_conditioner")

    panel_white = model.material("panel_white", color=(0.94, 0.95, 0.93))
    chassis_gray = model.material("chassis_gray", color=(0.80, 0.82, 0.82))
    grille_gray = model.material("grille_gray", color=(0.72, 0.75, 0.76))
    flap_white = model.material("flap_white", color=(0.92, 0.93, 0.91))
    button_gray = model.material("button_gray", color=(0.58, 0.60, 0.62))

    cassette = model.part("cassette")
    cassette.visual(
        mesh_from_cadquery(_build_panel_shell(), "cassette_panel_shell"),
        material=panel_white,
        name="panel_shell",
    )
    cassette.visual(
        mesh_from_cadquery(_build_intake_grille(), "cassette_intake_grille"),
        material=grille_gray,
        name="intake_grille",
    )
    cassette.visual(
        Box((CHASSIS_SIZE, CHASSIS_SIZE, CHASSIS_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, PANEL_THICKNESS + CHASSIS_HEIGHT / 2.0)),
        material=chassis_gray,
        name="upper_body",
    )

    front_flap = model.part("front_flap")
    front_flap.visual(
        Box((FLAP_LENGTH, FLAP_DEPTH, FLAP_THICKNESS)),
        origin=Origin(xyz=(0.0, FLAP_DEPTH / 2.0, -FLAP_THICKNESS / 2.0)),
        material=flap_white,
        name="flap_panel",
    )

    rear_flap = model.part("rear_flap")
    rear_flap.visual(
        Box((FLAP_LENGTH, FLAP_DEPTH, FLAP_THICKNESS)),
        origin=Origin(xyz=(0.0, -FLAP_DEPTH / 2.0, -FLAP_THICKNESS / 2.0)),
        material=flap_white,
        name="flap_panel",
    )

    left_flap = model.part("left_flap")
    left_flap.visual(
        Box((FLAP_DEPTH, FLAP_LENGTH, FLAP_THICKNESS)),
        origin=Origin(xyz=(FLAP_DEPTH / 2.0, 0.0, -FLAP_THICKNESS / 2.0)),
        material=flap_white,
        name="flap_panel",
    )

    right_flap = model.part("right_flap")
    right_flap.visual(
        Box((FLAP_DEPTH, FLAP_LENGTH, FLAP_THICKNESS)),
        origin=Origin(xyz=(-FLAP_DEPTH / 2.0, 0.0, -FLAP_THICKNESS / 2.0)),
        material=flap_white,
        name="flap_panel",
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"front_button_{index}")
        button.visual(
            Box(BUTTON_CAP_SIZE),
            material=button_gray,
            name="button_cap",
        )
        button.visual(
            Box((BUTTON_SLOT[0], BUTTON_SLOT[1], BUTTON_STEM_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, BUTTON_CAP_SIZE[2] / 2.0 + BUTTON_STEM_HEIGHT / 2.0)),
            material=button_gray,
            name="button_stem",
        )
        model.articulation(
            f"front_button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=cassette,
            child=button,
            origin=Origin(xyz=(button_x, BUTTON_Y, BUTTON_CAP_SIZE[2] / 2.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    model.articulation(
        "front_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=cassette,
        child=front_flap,
        origin=Origin(xyz=(0.0, -OUTLET_CENTER_OFFSET - OUTLET_DEPTH / 2.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=FLAP_SWING),
    )
    model.articulation(
        "rear_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=cassette,
        child=rear_flap,
        origin=Origin(xyz=(0.0, OUTLET_CENTER_OFFSET + OUTLET_DEPTH / 2.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=FLAP_SWING),
    )
    model.articulation(
        "left_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=cassette,
        child=left_flap,
        origin=Origin(xyz=(-OUTLET_CENTER_OFFSET - OUTLET_DEPTH / 2.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=FLAP_SWING),
    )
    model.articulation(
        "right_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=cassette,
        child=right_flap,
        origin=Origin(xyz=(OUTLET_CENTER_OFFSET + OUTLET_DEPTH / 2.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=FLAP_SWING),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cassette = object_model.get_part("cassette")
    flap_specs = (
        ("front_flap", "front_flap_hinge"),
        ("rear_flap", "rear_flap_hinge"),
        ("left_flap", "left_flap_hinge"),
        ("right_flap", "right_flap_hinge"),
    )

    for index in range(len(BUTTON_XS)):
        ctx.allow_overlap(
            cassette,
            f"front_button_{index}",
            elem_a="panel_shell",
            elem_b="button_stem",
            reason="The button stem is intentionally represented as a captured slider running inside the service-button pocket proxy in the panel shell.",
        )

    cassette_aabb = ctx.part_world_aabb(cassette)
    ctx.check("cassette_aabb_present", cassette_aabb is not None, "Expected the cassette body to compile into geometry.")
    if cassette_aabb is not None:
        min_pt, max_pt = cassette_aabb
        width = max_pt[0] - min_pt[0]
        depth = max_pt[1] - min_pt[1]
        height = max_pt[2] - min_pt[2]
        ctx.check("cassette_width_scale", abs(width - PANEL_SIZE) < 0.02, details=f"width={width}")
        ctx.check("cassette_depth_scale", abs(depth - PANEL_SIZE) < 0.02, details=f"depth={depth}")
        ctx.check(
            "cassette_height_scale",
            0.27 < height < 0.32,
            details=f"height={height}",
        )

    for flap_name, hinge_name in flap_specs:
        flap = object_model.get_part(flap_name)
        hinge = object_model.get_articulation(hinge_name)

        ctx.expect_gap(
            cassette,
            flap,
            axis="z",
            positive_elem="panel_shell",
            negative_elem="flap_panel",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{flap_name}_closed_flush",
        )

        rest_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")
        upper = hinge.motion_limits.upper if hinge.motion_limits is not None and hinge.motion_limits.upper is not None else 0.0
        with ctx.pose({hinge: upper}):
            open_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

        ctx.check(
            f"{flap_name}_opens_downward",
            rest_aabb is not None
            and open_aabb is not None
            and open_aabb[0][2] < rest_aabb[0][2] - 0.05,
            details=f"rest={rest_aabb}, open={open_aabb}",
        )

    for index in range(len(BUTTON_XS)):
        button = object_model.get_part(f"front_button_{index}")
        joint = object_model.get_articulation(f"front_button_{index}_press")

        rest_pos = ctx.part_world_position(button)
        upper = joint.motion_limits.upper if joint.motion_limits is not None and joint.motion_limits.upper is not None else 0.0
        with ctx.pose({joint: upper}):
            pressed_pos = ctx.part_world_position(button)

        ctx.check(
            f"front_button_{index}_presses_inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] > rest_pos[2] + 0.002,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
