from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.150
BASE_WIDTH = 0.032
BASE_HEIGHT = 0.008

TOP_LENGTH = 0.080
TOP_CAP_WIDTH = 0.038
TOP_CAP_HEIGHT = 0.006
TOP_SKIRT_HEIGHT = 0.005
TOP_FRAME_X = 0.030
TOP_FRAME_Z = 0.0105
TOP_SLIDE_TRAVEL = 0.045

GUARD_HINGE_X = -0.018
GUARD_HINGE_Z = 0.009


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _make_top_shell() -> cq.Workplane:
    """A single connected U-channel top cover with switch openings and hinge lugs."""
    shell = _cq_box((TOP_LENGTH, TOP_CAP_WIDTH, TOP_CAP_HEIGHT), (0.0, 0.0, TOP_CAP_HEIGHT / 2.0))

    # Underside side skirts make the detachable top read as a sliding cap over the base rails.
    skirt_y = TOP_CAP_WIDTH / 2.0
    shell = shell.union(_cq_box((TOP_LENGTH, 0.003, TOP_SKIRT_HEIGHT), (0.0, skirt_y, -TOP_SKIRT_HEIGHT / 2.0)))
    shell = shell.union(_cq_box((TOP_LENGTH, 0.003, TOP_SKIRT_HEIGHT), (0.0, -skirt_y, -TOP_SKIRT_HEIGHT / 2.0)))

    # Two fixed hinge cheeks on the top shell support the pivoting button guard.
    lug_size = (0.008, 0.0028, 0.006)
    shell = shell.union(_cq_box(lug_size, (GUARD_HINGE_X, 0.0126, TOP_CAP_HEIGHT + lug_size[2] / 2.0)))
    shell = shell.union(_cq_box(lug_size, (GUARD_HINGE_X, -0.0126, TOP_CAP_HEIGHT + lug_size[2] / 2.0)))

    # Through-openings let the separate buttons depress without colliding with the cap.
    shell = shell.cut(_cq_box((0.011, 0.007, 0.012), (-0.001, 0.0, TOP_CAP_HEIGHT / 2.0)))
    shell = shell.cut(_cq_box((0.009, 0.006, 0.012), (0.022, 0.0, TOP_CAP_HEIGHT / 2.0)))
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_presenter_clicker")

    matte_black = Material("matte_black_plastic", rgba=(0.015, 0.016, 0.018, 1.0))
    charcoal = Material("charcoal_top_plastic", rgba=(0.055, 0.058, 0.064, 1.0))
    rail_gray = Material("dark_rail_plastic", rgba=(0.020, 0.022, 0.026, 1.0))
    rubber = Material("soft_rubber_buttons", rgba=(0.12, 0.13, 0.14, 1.0))
    translucent_smoke = Material("smoked_translucent_guard", rgba=(0.20, 0.24, 0.28, 0.62))
    red_lens = Material("red_laser_window", rgba=(0.85, 0.05, 0.03, 0.78))
    contact_metal = Material("dull_gold_contacts", rgba=(0.78, 0.56, 0.20, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=matte_black,
        name="lower_base",
    )
    # Twin raised guide rails show how the detachable top is retained while sliding.
    for idx, y in enumerate((-0.0105, 0.0105)):
        base.visual(
            Box((0.095, 0.003, 0.0025)),
            origin=Origin(xyz=(0.020, y, BASE_HEIGHT + 0.00125)),
            material=rail_gray,
            name=f"guide_rail_{idx}",
        )
    base.visual(
        Box((0.003, 0.024, 0.004)),
        origin=Origin(xyz=(-0.028, 0.0, BASE_HEIGHT + 0.002)),
        material=rail_gray,
        name="slide_stop",
    )
    for idx, y in enumerate((-0.004, 0.004)):
        base.visual(
            Box((0.014, 0.0022, 0.0008)),
            origin=Origin(xyz=(0.061, y, BASE_HEIGHT + 0.0004)),
            material=contact_metal,
            name=f"charge_contact_{idx}",
        )

    clicker_top = model.part("clicker_top")
    clicker_top.visual(
        mesh_from_cadquery(_make_top_shell(), "clicker_top_shell", tolerance=0.00045),
        material=charcoal,
        name="top_shell",
    )
    clicker_top.visual(
        Box((0.0015, 0.010, 0.0035)),
        origin=Origin(xyz=(TOP_LENGTH / 2.0 + 0.00075, 0.0, 0.0032)),
        material=red_lens,
        name="laser_window",
    )
    clicker_top.visual(
        Box((0.030, 0.002, 0.0007)),
        origin=Origin(xyz=(0.007, -0.015, TOP_CAP_HEIGHT + 0.00035)),
        material=rail_gray,
        name="side_seam",
    )

    model.articulation(
        "top_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=clicker_top,
        origin=Origin(xyz=(TOP_FRAME_X, 0.0, TOP_FRAME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=0.20, lower=0.0, upper=TOP_SLIDE_TRAVEL),
    )

    main_button = model.part("main_button")
    main_button.visual(
        Box((0.011, 0.007, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.00075)),
        material=rubber,
        name="button_cap",
    )
    main_button.visual(
        Box((0.005, 0.0035, 0.0050)),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=rubber,
        name="button_stem",
    )
    model.articulation(
        "main_button_travel",
        ArticulationType.PRISMATIC,
        parent=clicker_top,
        child=main_button,
        origin=Origin(xyz=(-0.001, 0.0, TOP_CAP_HEIGHT)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.04, lower=0.0, upper=0.0015),
    )

    forward_button = model.part("forward_button")
    forward_button.visual(
        Box((0.009, 0.006, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, 0.00075)),
        material=rubber,
        name="button_cap",
    )
    forward_button.visual(
        Box((0.004, 0.003, 0.0050)),
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
        material=rubber,
        name="button_stem",
    )
    model.articulation(
        "forward_button_travel",
        ArticulationType.PRISMATIC,
        parent=clicker_top,
        child=forward_button,
        origin=Origin(xyz=(0.022, 0.0, TOP_CAP_HEIGHT)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.04, lower=0.0, upper=0.0013),
    )

    button_guard = model.part("button_guard")
    button_guard.visual(
        Cylinder(radius=0.002, length=0.0224),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=translucent_smoke,
        name="hinge_barrel",
    )
    button_guard.visual(
        Box((0.030, 0.018, 0.0015)),
        origin=Origin(xyz=(0.017, 0.0, 0.00080)),
        material=translucent_smoke,
        name="guard_flap",
    )
    button_guard.visual(
        Box((0.002, 0.018, 0.0030)),
        origin=Origin(xyz=(0.031, 0.0, 0.0012)),
        material=translucent_smoke,
        name="lift_tab",
    )
    model.articulation(
        "guard_hinge",
        ArticulationType.REVOLUTE,
        parent=clicker_top,
        child=button_guard,
        origin=Origin(xyz=(GUARD_HINGE_X, 0.0, GUARD_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=2.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    clicker_top = object_model.get_part("clicker_top")
    button_guard = object_model.get_part("button_guard")
    main_button = object_model.get_part("main_button")
    forward_button = object_model.get_part("forward_button")
    top_slide = object_model.get_articulation("top_slide")
    guard_hinge = object_model.get_articulation("guard_hinge")
    main_travel = object_model.get_articulation("main_button_travel")
    forward_travel = object_model.get_articulation("forward_button_travel")

    ctx.expect_overlap(
        clicker_top,
        base,
        axes="xy",
        min_overlap=0.020,
        name="detachable top is seated on the rectangular base footprint",
    )
    ctx.expect_overlap(
        button_guard,
        main_button,
        axes="xy",
        min_overlap=0.006,
        name="button guard covers the main click button",
    )

    rest_top_pos = ctx.part_world_position(clicker_top)
    with ctx.pose({top_slide: TOP_SLIDE_TRAVEL}):
        ctx.expect_overlap(
            clicker_top,
            base,
            axes="x",
            min_overlap=0.030,
            name="slid top keeps short retained engagement on the long-axis rails",
        )
        extended_top_pos = ctx.part_world_position(clicker_top)
    ctx.check(
        "top slide moves along the body long axis",
        rest_top_pos is not None
        and extended_top_pos is not None
        and extended_top_pos[0] > rest_top_pos[0] + 0.040,
        details=f"rest={rest_top_pos}, extended={extended_top_pos}",
    )

    guard_closed_aabb = ctx.part_world_aabb(button_guard)
    with ctx.pose({guard_hinge: 1.10}):
        guard_open_aabb = ctx.part_world_aabb(button_guard)
    ctx.check(
        "button guard pivots upward from its hinge",
        guard_closed_aabb is not None
        and guard_open_aabb is not None
        and guard_open_aabb[1][2] > guard_closed_aabb[1][2] + 0.015,
        details=f"closed={guard_closed_aabb}, open={guard_open_aabb}",
    )

    main_rest = ctx.part_world_position(main_button)
    forward_rest = ctx.part_world_position(forward_button)
    with ctx.pose({main_travel: 0.0012, forward_travel: 0.0010}):
        main_pressed = ctx.part_world_position(main_button)
        forward_pressed = ctx.part_world_position(forward_button)
    ctx.check(
        "buttons depress into the clicker top openings",
        main_rest is not None
        and main_pressed is not None
        and forward_rest is not None
        and forward_pressed is not None
        and main_pressed[2] < main_rest[2] - 0.0010
        and forward_pressed[2] < forward_rest[2] - 0.0008,
        details=f"main={main_rest}->{main_pressed}, forward={forward_rest}->{forward_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
