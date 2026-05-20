from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq
import math


BODY_WIDTH = 0.074
BODY_LENGTH = 0.145
BODY_THICKNESS = 0.010
HINGE_LENGTH = 0.022
HINGE_THICKNESS = 0.003
SCREEN_WIDTH = 0.062
SCREEN_LENGTH = 0.119
SCREEN_THICKNESS = 0.0010


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Rounded-corner rectangular slab in local XYZ, sized in meters."""
    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parallel_screen_flip_phone")

    shell_mat = model.material("anodized_graphite", rgba=(0.035, 0.040, 0.048, 1.0))
    glass_mat = model.material("black_glass", rgba=(0.005, 0.007, 0.010, 1.0))
    hinge_mat = model.material("brushed_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    blue_mat = model.material("lit_blue_panel", rgba=(0.08, 0.32, 0.95, 1.0))
    violet_mat = model.material("lit_violet_panel", rgba=(0.42, 0.12, 0.75, 1.0))
    green_mat = model.material("lit_green_panel", rgba=(0.08, 0.68, 0.42, 1.0))
    speaker_mat = model.material("speaker_black", rgba=(0.0, 0.0, 0.0, 1.0))

    shell_shape = _rounded_box((BODY_WIDTH, BODY_LENGTH, BODY_THICKNESS), 0.010)
    screen_shape = _rounded_box((SCREEN_WIDTH, SCREEN_LENGTH, SCREEN_THICKNESS), 0.006)

    lower_body = model.part("lower_body")
    lower_body.visual(
        mesh_from_cadquery(shell_shape, "lower_shell"),
        origin=Origin(xyz=(0.0, -BODY_LENGTH / 2.0, BODY_THICKNESS / 2.0)),
        material=shell_mat,
        name="lower_shell",
    )
    lower_body.visual(
        mesh_from_cadquery(screen_shape, "lower_display"),
        # A very slight embed makes the glass read as a seated flush display.
        origin=Origin(xyz=(0.0, -BODY_LENGTH / 2.0, BODY_THICKNESS + 0.00035)),
        material=glass_mat,
        name="lower_display",
    )
    lower_body.visual(
        Box((0.054, 0.026, 0.00010)),
        origin=Origin(xyz=(0.0, -0.092, BODY_THICKNESS + 0.00090)),
        material=blue_mat,
        name="lower_app_tile",
    )
    lower_body.visual(
        Box((0.054, 0.018, 0.00010)),
        origin=Origin(xyz=(0.0, -0.052, BODY_THICKNESS + 0.00090)),
        material=green_mat,
        name="lower_status_tile",
    )

    hinge_link = model.part("hinge_link")
    hinge_link.visual(
        Box((BODY_WIDTH + 0.004, HINGE_LENGTH, HINGE_THICKNESS)),
        origin=Origin(xyz=(0.0, HINGE_LENGTH / 2.0, 0.0)),
        material=hinge_mat,
        name="hinge_leaf",
    )
    hinge_link.visual(
        Box((BODY_WIDTH + 0.008, 0.0022, HINGE_THICKNESS * 1.2)),
        origin=Origin(xyz=(0.0, 0.0011, 0.0)),
        material=hinge_mat,
        name="lower_knuckle",
    )
    hinge_link.visual(
        Box((BODY_WIDTH + 0.008, 0.0022, HINGE_THICKNESS * 1.2)),
        origin=Origin(xyz=(0.0, HINGE_LENGTH - 0.0011, 0.0)),
        material=hinge_mat,
        name="upper_knuckle",
    )

    upper_body = model.part("upper_body")
    upper_body.visual(
        mesh_from_cadquery(shell_shape, "upper_shell"),
        origin=Origin(xyz=(0.0, BODY_LENGTH / 2.0, 0.0)),
        material=shell_mat,
        name="upper_shell",
    )
    upper_body.visual(
        mesh_from_cadquery(screen_shape, "upper_display"),
        origin=Origin(xyz=(0.0, BODY_LENGTH / 2.0, BODY_THICKNESS / 2.0 + 0.00035)),
        material=glass_mat,
        name="upper_display",
    )
    upper_body.visual(
        Box((0.054, 0.030, 0.00010)),
        origin=Origin(xyz=(0.0, 0.092, BODY_THICKNESS / 2.0 + 0.00090)),
        material=violet_mat,
        name="upper_wallpaper_tile",
    )
    upper_body.visual(
        Box((0.054, 0.017, 0.00010)),
        origin=Origin(xyz=(0.0, 0.052, BODY_THICKNESS / 2.0 + 0.00090)),
        material=blue_mat,
        name="upper_notification_tile",
    )
    upper_body.visual(
        Box((0.018, 0.0020, 0.00010)),
        origin=Origin(xyz=(0.0, 0.012, BODY_THICKNESS / 2.0 + 0.00005)),
        material=speaker_mat,
        name="speaker_slot",
    )

    model.articulation(
        "lower_to_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=hinge_link,
        # The first fold axis is the widthwise line along the lower slab's top edge.
        origin=Origin(xyz=(0.0, 0.0, BODY_THICKNESS / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=1.8, velocity=1.5),
    )
    model.articulation(
        "hinge_to_upper",
        ArticulationType.REVOLUTE,
        parent=hinge_link,
        child=upper_body,
        # A counter-rotating second leaf gives one controlled folding DOF while
        # the two display faces stay parallel, like a compact parallel hinge.
        origin=Origin(xyz=(0.0, HINGE_LENGTH, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-1.25, upper=0.0, effort=1.8, velocity=1.5),
        mimic=Mimic(joint="lower_to_hinge", multiplier=-1.0, offset=0.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    hinge_link = object_model.get_part("hinge_link")
    upper_body = object_model.get_part("upper_body")
    fold = object_model.get_articulation("lower_to_hinge")

    ctx.expect_gap(
        upper_body,
        lower_body,
        axis="y",
        min_gap=HINGE_LENGTH - 0.001,
        max_gap=HINGE_LENGTH + 0.001,
        name="flat-open halves are separated by the hinge strip",
    )
    ctx.expect_contact(
        hinge_link,
        lower_body,
        elem_a="hinge_leaf",
        elem_b="lower_shell",
        contact_tol=0.0001,
        name="hinge leaf meets the lower body edge",
    )
    ctx.expect_contact(
        hinge_link,
        upper_body,
        elem_a="hinge_leaf",
        elem_b="upper_shell",
        contact_tol=0.0001,
        name="hinge leaf meets the upper body edge",
    )
    ctx.expect_within(
        lower_body,
        lower_body,
        axes="xy",
        inner_elem="lower_display",
        outer_elem="lower_shell",
        margin=0.0,
        name="lower display sits within its body bezel",
    )
    ctx.expect_within(
        upper_body,
        upper_body,
        axes="xy",
        inner_elem="upper_display",
        outer_elem="upper_shell",
        margin=0.0,
        name="upper display sits within its body bezel",
    )

    rest_upper = ctx.part_world_position(upper_body)
    with ctx.pose({fold: 1.0}):
        ctx.expect_gap(
            upper_body,
            lower_body,
            axis="y",
            min_gap=0.006,
            name="opened parallel hinge keeps slab edges clear",
        )
        opened_upper = ctx.part_world_position(upper_body)
        lower_aabb = ctx.part_element_world_aabb(lower_body, elem="lower_display")
        upper_aabb = ctx.part_element_world_aabb(upper_body, elem="upper_display")
        if lower_aabb is None or upper_aabb is None:
            ctx.fail("display AABBs are available", "could not resolve display elements")
        else:
            lower_dz = float(lower_aabb[1][2] - lower_aabb[0][2])
            upper_dz = float(upper_aabb[1][2] - upper_aabb[0][2])
            ctx.check(
                "opened upper display remains parallel to lower display",
                lower_dz < 0.0025 and upper_dz < 0.0025,
                details=f"lower display dz={lower_dz:.6f}, upper display dz={upper_dz:.6f}",
            )

    ctx.check(
        "folding hinge lifts the upper body while preserving display attitude",
        rest_upper is not None
        and opened_upper is not None
        and opened_upper[2] > rest_upper[2] + HINGE_LENGTH * math.sin(1.0) * 0.8,
        details=f"rest={rest_upper}, opened={opened_upper}",
    )

    return ctx.report()


object_model = build_object_model()
