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
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_LENGTH = 0.180
BODY_DEPTH = 0.056
BODY_HEIGHT = 0.060
END_CAP_THICKNESS = 0.010


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Return a centered, softly radiused rectangular solid."""
    shape = cq.Workplane("XY").box(*size)
    if radius > 0:
        shape = shape.edges().fillet(radius)
    return shape


def _carry_loop_shape() -> cq.Workplane:
    """Flattened rubber loop modeled around a vertical hinge axis at local x=0."""
    arm_len = 0.050
    strap_w = 0.008
    strap_t = 0.007
    pivot_z = 0.021

    top_arm = _rounded_box((arm_len, strap_w, strap_t), 0.002).translate(
        (-arm_len / 2.0, 0.0, pivot_z)
    )
    bottom_arm = _rounded_box((arm_len, strap_w, strap_t), 0.002).translate(
        (-arm_len / 2.0, 0.0, -pivot_z)
    )
    outer_grip = _rounded_box((strap_w, strap_w, 2.0 * pivot_z + strap_t), 0.002).translate(
        (-arm_len, 0.0, 0.0)
    )
    upper_pivot_lug = _rounded_box((0.013, 0.012, 0.011), 0.003).translate(
        (0.0, 0.0, pivot_z)
    )
    lower_pivot_lug = _rounded_box((0.013, 0.012, 0.011), 0.003).translate(
        (0.0, 0.0, -pivot_z)
    )

    return (
        top_arm.union(bottom_arm)
        .union(outer_grip)
        .union(upper_pivot_lug)
        .union(lower_pivot_lug)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_speaker")

    shell_mat = Material("warm_gray_anodized_body", rgba=(0.42, 0.44, 0.45, 1.0))
    rubber_mat = Material("matte_black_rubber", rgba=(0.015, 0.016, 0.017, 1.0))
    grille_mat = Material("dark_perforated_grille", rgba=(0.02, 0.023, 0.025, 1.0))
    strip_mat = Material("charcoal_control_strip", rgba=(0.05, 0.055, 0.06, 1.0))
    slider_mat = Material("blue_power_slider", rgba=(0.02, 0.35, 0.85, 1.0))
    metal_mat = Material("satin_pivot_metal", rgba=(0.70, 0.72, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(
            _rounded_box((BODY_LENGTH, BODY_DEPTH, BODY_HEIGHT), 0.008),
            "rounded_speaker_body",
            tolerance=0.0008,
            angular_tolerance=0.15,
        ),
        material=shell_mat,
        name="body_shell",
    )

    # Rubber end caps wrap the short sides and give the pivots a credible mount.
    left_cap_x = -BODY_LENGTH / 2.0 - END_CAP_THICKNESS / 2.0 + 0.002
    right_cap_x = BODY_LENGTH / 2.0 + END_CAP_THICKNESS / 2.0 - 0.002
    for idx, x in enumerate((left_cap_x, right_cap_x)):
        body.visual(
            mesh_from_cadquery(
                _rounded_box((END_CAP_THICKNESS, BODY_DEPTH + 0.004, BODY_HEIGHT + 0.004), 0.006),
                f"rubber_end_cap_{idx}",
                tolerance=0.0008,
                angular_tolerance=0.15,
            ),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=rubber_mat,
            name=f"end_cap_{idx}",
        )

    grille = PerforatedPanelGeometry(
        (0.132, 0.040),
        0.003,
        hole_diameter=0.004,
        pitch=(0.008, 0.008),
        frame=0.006,
        corner_radius=0.004,
        stagger=True,
    )
    body.visual(
        mesh_from_geometry(grille, "front_perforated_grille"),
        origin=Origin(
            xyz=(0.012, BODY_DEPTH / 2.0 + 0.0010, -0.002),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=grille_mat,
        name="front_grille",
    )

    # Upper strip, recessed guide slot, and two tiny printed control marks.
    body.visual(
        Box((0.130, 0.020, 0.003)),
        origin=Origin(xyz=(0.014, -0.006, BODY_HEIGHT / 2.0 + 0.0015)),
        material=strip_mat,
        name="control_strip",
    )
    body.visual(
        Box((0.046, 0.011, 0.0012)),
        origin=Origin(xyz=(0.034, -0.006, BODY_HEIGHT / 2.0 + 0.0036)),
        material=grille_mat,
        name="switch_slot",
    )
    body.visual(
        Box((0.004, 0.0015, 0.001)),
        origin=Origin(xyz=(0.009, -0.014, BODY_HEIGHT / 2.0 + 0.0033)),
        material=slider_mat,
        name="power_mark_0",
    )
    body.visual(
        Box((0.007, 0.0015, 0.001)),
        origin=Origin(xyz=(0.058, -0.014, BODY_HEIGHT / 2.0 + 0.0033)),
        material=slider_mat,
        name="power_mark_1",
    )

    hinge_x = left_cap_x - END_CAP_THICKNESS / 2.0 - 0.004
    for idx, z in enumerate((0.021, -0.021)):
        body.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(xyz=(hinge_x + 0.004, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"pivot_boss_{idx}",
        )

    carry_loop = model.part("carry_loop")
    carry_loop.visual(
        mesh_from_cadquery(
            _carry_loop_shape(),
            "carry_loop_frame",
            tolerance=0.0006,
            angular_tolerance=0.12,
        ),
        material=rubber_mat,
        name="loop_frame",
    )

    power_slider = model.part("power_slider")
    power_slider.visual(
        Box((0.016, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=slider_mat,
        name="slider_cap",
    )
    power_slider.visual(
        Box((0.010, 0.003, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=strip_mat,
        name="thumb_ridge",
    )

    model.articulation(
        "body_to_carry_loop",
        ArticulationType.REVOLUTE,
        parent=body,
        child=carry_loop,
        origin=Origin(xyz=(hinge_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=0.0, upper=1.35),
    )

    model.articulation(
        "body_to_power_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_slider,
        origin=Origin(xyz=(0.034, -0.006, BODY_HEIGHT / 2.0 + 0.0042)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=0.15, lower=-0.010, upper=0.010),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    carry_loop = object_model.get_part("carry_loop")
    power_slider = object_model.get_part("power_slider")
    loop_joint = object_model.get_articulation("body_to_carry_loop")
    slider_joint = object_model.get_articulation("body_to_power_slider")

    for boss in ("pivot_boss_0", "pivot_boss_1"):
        ctx.allow_overlap(
            body,
            carry_loop,
            elem_a=boss,
            elem_b="loop_frame",
            reason="The loop lug is intentionally captured around the end-cap pivot boss.",
        )
        ctx.expect_overlap(
            body,
            carry_loop,
            axes="x",
            elem_a=boss,
            elem_b="loop_frame",
            min_overlap=0.001,
            name=f"{boss} captures the carry loop lug",
        )

    ctx.expect_within(
        power_slider,
        body,
        axes="xy",
        inner_elem="slider_cap",
        outer_elem="switch_slot",
        margin=0.001,
        name="power slider sits inside the guide slot footprint",
    )
    ctx.expect_gap(
        power_slider,
        body,
        axis="z",
        positive_elem="slider_cap",
        negative_elem="switch_slot",
        max_gap=0.001,
        max_penetration=0.0,
        name="power slider cap rides on the slot surface",
    )

    rest_slider_pos = ctx.part_world_position(power_slider)
    with ctx.pose({slider_joint: 0.010}):
        ctx.expect_within(
            power_slider,
            body,
            axes="xy",
            inner_elem="slider_cap",
            outer_elem="switch_slot",
            margin=0.001,
            name="power slider remains retained at the on position",
        )
        on_slider_pos = ctx.part_world_position(power_slider)
    ctx.check(
        "power slider translates along the control strip",
        rest_slider_pos is not None
        and on_slider_pos is not None
        and on_slider_pos[0] > rest_slider_pos[0] + 0.008,
        details=f"rest={rest_slider_pos}, on={on_slider_pos}",
    )

    def element_center_y(aabb):
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) / 2.0

    rest_loop_aabb = ctx.part_element_world_aabb(carry_loop, elem="loop_frame")
    rest_loop_y = element_center_y(rest_loop_aabb)
    with ctx.pose({loop_joint: 1.0}):
        swung_loop_aabb = ctx.part_element_world_aabb(carry_loop, elem="loop_frame")
        swung_loop_y = element_center_y(swung_loop_aabb)
    ctx.check(
        "carry loop rotates outward from the end cap",
        rest_loop_y is not None and swung_loop_y is not None and swung_loop_y < rest_loop_y - 0.005,
        details=f"rest_y={rest_loop_y}, swung_y={swung_loop_y}",
    )

    return ctx.report()


object_model = build_object_model()
