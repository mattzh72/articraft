from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.58
BODY_D = 0.22
BODY_H = 0.34


def _rounded_body_shell():
    """Broad portable-speaker housing with softened molded-plastic corners."""
    return cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H).edges("|Z").fillet(0.028)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_speaker_fold_handles")

    shell_mat = model.material("charcoal_molded_shell", rgba=(0.06, 0.07, 0.08, 1.0))
    grille_mat = model.material("black_perforated_metal", rgba=(0.005, 0.005, 0.006, 1.0))
    rubber_mat = model.material("matte_black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    hinge_mat = model.material("dark_hinge_hardware", rgba=(0.03, 0.032, 0.035, 1.0))
    handle_mat = model.material("graphite_handle_plastic", rgba=(0.025, 0.026, 0.028, 1.0))
    wheel_mat = model.material("satin_selector_wheel", rgba=(0.12, 0.12, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_body_shell(), "rounded_speaker_body", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
        material=shell_mat,
        name="shell",
    )

    grille = PerforatedPanelGeometry(
        (0.47, 0.235),
        0.006,
        hole_diameter=0.0065,
        pitch=(0.014, 0.012),
        frame=0.014,
        corner_radius=0.010,
        stagger=True,
    )
    body.visual(
        mesh_from_geometry(grille, "front_perforated_grille"),
        origin=Origin(
            xyz=(0.0, -BODY_D / 2.0 - 0.002, BODY_H / 2.0 - 0.018),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=grille_mat,
        name="front_grille",
    )

    # A shallow rubber control shelf and a round escutcheon make the selector
    # read as recessed into the upper front edge rather than pasted on.
    body.visual(
        Box((0.26, 0.010, 0.038)),
        origin=Origin(xyz=(0.13, -BODY_D / 2.0 - 0.004, BODY_H - 0.055)),
        material=rubber_mat,
        name="front_control_shelf",
    )
    body.visual(
        Cylinder(radius=0.047, length=0.006),
        origin=Origin(
            xyz=(0.18, -BODY_D / 2.0 - 0.003, BODY_H - 0.055),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber_mat,
        name="selector_bezel",
    )

    # Fixed half of the side hinges: central knuckles and wall plates.  The
    # moving handles carry top/bottom knuckles, leaving visible gaps rather than
    # broad interpenetrating hinge barrels.
    for side_name, sx in (("left", -1.0), ("right", 1.0)):
        x = sx * (BODY_W / 2.0 + 0.013)
        body.visual(
            Box((0.006, 0.038, 0.120)),
            origin=Origin(xyz=(sx * (BODY_W / 2.0 + 0.003), -0.055, BODY_H / 2.0)),
            material=hinge_mat,
            name=f"{side_name}_hinge_plate",
        )
        body.visual(
            Cylinder(radius=0.012, length=0.100),
            origin=Origin(xyz=(x, -0.055, BODY_H / 2.0)),
            material=hinge_mat,
            name=f"{side_name}_fixed_knuckle",
        )

    def add_handle(name: str, sx: float, axis_sign: float) -> None:
        handle = model.part(name)
        # Two small hinge barrels sit above and below the fixed body knuckle.
        for zc, label in ((0.081, "upper"), (-0.081, "lower")):
            handle.visual(
                Cylinder(radius=0.011, length=0.038),
                origin=Origin(xyz=(0.0, 0.0, zc)),
                material=handle_mat,
                name=f"{label}_knuckle",
            )
            handle.visual(
                Box((0.018, 0.132, 0.020)),
                origin=Origin(xyz=(0.0, 0.066, zc)),
                material=handle_mat,
                name=f"{label}_arm",
            )
        handle.visual(
            Cylinder(radius=0.013, length=0.170),
            origin=Origin(xyz=(0.0, 0.132, 0.0)),
            material=handle_mat,
            name="grip",
        )

        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=handle,
            origin=Origin(xyz=(sx * (BODY_W / 2.0 + 0.013), -0.055, BODY_H / 2.0)),
            axis=(0.0, 0.0, axis_sign),
            motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=1.55),
        )

    add_handle("left_handle", -1.0, 1.0)
    add_handle("right_handle", 1.0, -1.0)

    selector_wheel = model.part("selector_wheel")
    selector_wheel.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.072,
                0.025,
                body_style="faceted",
                top_diameter=0.064,
                edge_radius=0.0012,
                grip=KnobGrip(style="ribbed", count=22, depth=0.0010, width=0.0015),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "rotary_selector_wheel",
        ),
        material=wheel_mat,
        name="selector_cap",
    )
    model.articulation(
        "body_to_selector_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_wheel,
        origin=Origin(
            xyz=(0.18, -BODY_D / 2.0 - 0.006, BODY_H - 0.055),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    selector_wheel = object_model.get_part("selector_wheel")
    left_joint = object_model.get_articulation("body_to_left_handle")
    right_joint = object_model.get_articulation("body_to_right_handle")
    wheel_joint = object_model.get_articulation("body_to_selector_wheel")

    ctx.expect_contact(
        selector_wheel,
        body,
        elem_a="selector_cap",
        elem_b="selector_bezel",
        contact_tol=0.0015,
        name="selector wheel seats against front bezel",
    )
    ctx.expect_overlap(
        selector_wheel,
        body,
        axes="xz",
        elem_a="selector_cap",
        elem_b="selector_bezel",
        min_overlap=0.050,
        name="selector wheel is centered in the round bezel",
    )

    rest_left = ctx.part_world_aabb(left_handle)
    rest_right = ctx.part_world_aabb(right_handle)
    with ctx.pose({left_joint: 1.25, right_joint: 1.25, wheel_joint: 1.0}):
        open_left = ctx.part_world_aabb(left_handle)
        open_right = ctx.part_world_aabb(right_handle)
    ctx.check(
        "left handle swings outward from side wall",
        rest_left is not None
        and open_left is not None
        and open_left[0][0] < rest_left[0][0] - 0.050,
        details=f"rest={rest_left}, open={open_left}",
    )
    ctx.check(
        "right handle swings outward from side wall",
        rest_right is not None
        and open_right is not None
        and open_right[1][0] > rest_right[1][0] + 0.050,
        details=f"rest={rest_right}, open={open_right}",
    )
    ctx.check(
        "selector wheel rotates about short front axis",
        tuple(round(v, 3) for v in wheel_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={wheel_joint.axis}",
    )

    return ctx.report()


object_model = build_object_model()
