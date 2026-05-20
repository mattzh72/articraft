from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


BODY_X = 0.640
BODY_Y = 0.460
BODY_Z = 0.075
HINGE_X = -0.335
HINGE_Z = 0.090
LID_X = 0.595
LID_Y = 0.445
LID_Z = 0.030
LID_REAR_CLEARANCE = 0.020
GUIDE_TRAVEL = 0.200


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded rectangular solid centered on the local origin."""
    x, y, z = size
    return cq.Workplane("XY").box(x, y, z).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="a3_flatbed_scanner")

    model.material("warm_gray_plastic", rgba=(0.64, 0.66, 0.66, 1.0))
    model.material("dark_bezel", rgba=(0.045, 0.050, 0.055, 1.0))
    model.material("smoked_glass", rgba=(0.45, 0.70, 0.85, 0.42))
    model.material("lid_plastic", rgba=(0.82, 0.84, 0.82, 1.0))
    model.material("white_backing", rgba=(0.95, 0.96, 0.94, 1.0))
    model.material("brushed_metal", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("guide_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("button_black", rgba=(0.03, 0.035, 0.04, 1.0))
    model.material("button_green", rgba=(0.10, 0.42, 0.18, 1.0))

    body = model.part("scanner_body")
    body.visual(
        mesh_from_cadquery(_rounded_box((BODY_X, BODY_Y, BODY_Z), 0.012), "body_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z / 2.0)),
        material="warm_gray_plastic",
        name="body_shell",
    )

    glass_x = 0.455
    glass_y = 0.322
    glass_cx = 0.030
    border = 0.025
    glass_top_z = BODY_Z + 0.004
    # Separate bezel strips keep the glass visibly inset in a dark scan bed.
    for name, xyz, size in (
        (
            "rear_bezel",
            (glass_cx - glass_x / 2.0 - border / 2.0, 0.0, BODY_Z + 0.0012),
            (border, glass_y + 2 * border, 0.0030),
        ),
        (
            "front_bezel",
            (glass_cx + glass_x / 2.0 + border / 2.0, 0.0, BODY_Z + 0.0012),
            (border, glass_y + 2 * border, 0.0030),
        ),
        (
            "side_bezel_0",
            (glass_cx, -(glass_y / 2.0 + border / 2.0), BODY_Z + 0.0012),
            (glass_x, border, 0.0030),
        ),
        (
            "side_bezel_1",
            (glass_cx, glass_y / 2.0 + border / 2.0, BODY_Z + 0.0012),
            (glass_x, border, 0.0030),
        ),
    ):
        body.visual(Box(size), origin=Origin(xyz=xyz), material="dark_bezel", name=name)

    body.visual(
        Box((glass_x, glass_y, 0.004)),
        origin=Origin(xyz=(glass_cx, 0.0, BODY_Z + 0.002)),
        material="smoked_glass",
        name="platen_glass",
    )

    # A small exposed front control island reads as a real scanner appliance.
    body.visual(
        Box((0.050, 0.095, 0.004)),
        origin=Origin(xyz=(0.296, 0.165, BODY_Z + 0.0018)),
        material="dark_bezel",
        name="control_panel",
    )
    body.visual(
        Box((0.028, 0.006, 0.002)),
        origin=Origin(xyz=(-0.215, -0.190, BODY_Z + 0.0010)),
        material="white_backing",
        name="a3_mark",
    )
    body.visual(
        Box((0.006, 0.028, 0.002)),
        origin=Origin(xyz=(-0.232, -0.174, BODY_Z + 0.0010)),
        material="white_backing",
        name="corner_mark",
    )

    for index, y_pos in enumerate((-0.115, 0.115)):
        body.visual(
            Box((0.036, 0.198, 0.006)),
            origin=Origin(xyz=(HINGE_X + 0.008, y_pos, BODY_Z + 0.0020)),
            material="brushed_metal",
            name=f"rear_hinge_leaf_{index}",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_rounded_box((LID_X, LID_Y, LID_Z), 0.008), "lid_panel"),
        origin=Origin(xyz=(LID_REAR_CLEARANCE + LID_X / 2.0, 0.0, 0.018)),
        material="lid_plastic",
        name="lid_panel",
    )
    lid.visual(
        Box((0.520, 0.370, 0.004)),
        origin=Origin(xyz=(0.340, 0.0, 0.001)),
        material="white_backing",
        name="inner_pad",
    )
    # Two full-width barrel-hinge segments span nearly the whole rear edge.
    for index, y_pos in enumerate((-0.115, 0.115)):
        lid.visual(
            Cylinder(radius=0.010, length=0.198),
            origin=Origin(xyz=(0.0, y_pos, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="brushed_metal",
            name=f"hinge_barrel_{index}",
        )
        lid.visual(
            Box((0.036, 0.198, 0.004)),
            origin=Origin(xyz=(0.018, y_pos, 0.004)),
            material="brushed_metal",
            name=f"lid_hinge_leaf_{index}",
        )

    guide = model.part("guide_strip")
    guide.visual(
        Box((0.460, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material="guide_gray",
        name="guide_bar",
    )
    guide.visual(
        Box((0.045, 0.030, 0.006)),
        origin=Origin(xyz=(0.205, 0.0, -0.003)),
        material="button_black",
        name="thumb_tab",
    )

    power_button = model.part("power_button")
    power_button.visual(
        Box((0.020, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="button_black",
        name="button_cap",
    )

    scan_button = model.part("scan_button")
    scan_button.visual(
        Box((0.020, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="button_green",
        name="button_cap",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.25),
    )
    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=guide,
        origin=Origin(xyz=(0.340, -0.170, -0.001)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.15, lower=0.0, upper=GUIDE_TRAVEL),
    )
    model.articulation(
        "power_button_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_button,
        origin=Origin(xyz=(0.296, 0.145, BODY_Z + 0.0038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=-0.003, upper=0.0),
    )
    model.articulation(
        "scan_button_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=scan_button,
        origin=Origin(xyz=(0.296, 0.183, BODY_Z + 0.0038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=-0.003, upper=0.0),
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

    body = object_model.get_part("scanner_body")
    lid = object_model.get_part("lid")
    guide = object_model.get_part("guide_strip")
    hinge = object_model.get_articulation("lid_hinge")
    slide = object_model.get_articulation("guide_slide")

    with ctx.pose({hinge: 0.0, slide: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="body_shell",
            min_overlap=0.40,
            name="closed lid covers the A3 scan bed",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="inner_pad",
            negative_elem="platen_glass",
            min_gap=0.004,
            max_gap=0.020,
            name="lid backing clears the glass platen",
        )
        ctx.expect_gap(
            lid,
            guide,
            axis="z",
            positive_elem="inner_pad",
            negative_elem="guide_bar",
            max_gap=0.001,
            max_penetration=0.0002,
            name="guide rides on the lid inner face",
        )
        ctx.expect_within(
            guide,
            lid,
            axes="x",
            inner_elem="guide_bar",
            outer_elem="inner_pad",
            margin=0.005,
            name="guide strip stays on the lid backing",
        )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: 1.10}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid hinge opens upward",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.25,
        details=f"rest={rest_lid_aabb}, open={open_lid_aabb}",
    )

    rest_guide_pos = ctx.part_world_position(guide)
    with ctx.pose({slide: GUIDE_TRAVEL}):
        extended_guide_pos = ctx.part_world_position(guide)
        ctx.expect_within(
            guide,
            lid,
            axes="x",
            inner_elem="guide_bar",
            outer_elem="inner_pad",
            margin=0.005,
            name="extended guide remains captured by lid track",
        )
    ctx.check(
        "document guide slides laterally",
        rest_guide_pos is not None
        and extended_guide_pos is not None
        and extended_guide_pos[1] > rest_guide_pos[1] + 0.15,
        details=f"rest={rest_guide_pos}, extended={extended_guide_pos}",
    )

    return ctx.report()


object_model = build_object_model()
