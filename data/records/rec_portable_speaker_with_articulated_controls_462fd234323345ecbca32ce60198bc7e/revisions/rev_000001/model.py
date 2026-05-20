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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_speaker_with_control_cover")

    case_mat = Material("warm_gray_molded_case", rgba=(0.25, 0.26, 0.25, 1.0))
    grille_mat = Material("black_perforated_metal", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber_mat = Material("matte_black_rubber", rgba=(0.02, 0.02, 0.02, 1.0))
    cover_mat = Material("smoked_translucent_cover", rgba=(0.05, 0.06, 0.07, 0.45))
    hinge_mat = Material("dark_hinge_plastic", rgba=(0.03, 0.035, 0.04, 1.0))
    accent_mat = Material("soft_blue_button", rgba=(0.10, 0.30, 0.55, 1.0))

    body = model.part("body")

    case_shape = (
        cq.Workplane("XY")
        .box(0.300, 0.160, 0.180)
        .edges()
        .fillet(0.014)
    )
    body.visual(
        mesh_from_cadquery(case_shape, "rounded_speaker_case", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=case_mat,
        name="rounded_case",
    )

    speaker_face = PerforatedPanelGeometry(
        (0.220, 0.116),
        0.004,
        hole_diameter=0.0055,
        pitch=(0.011, 0.010),
        frame=0.010,
        corner_radius=0.007,
        stagger=True,
    )
    body.visual(
        mesh_from_geometry(speaker_face, "front_speaker_grille"),
        origin=Origin(xyz=(0.0, -0.081, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grille_mat,
        name="front_grille",
    )

    body.visual(
        Box((0.250, 0.078, 0.006)),
        origin=Origin(xyz=(0.0, -0.025, 0.183)),
        material=rubber_mat,
        name="control_strip",
    )

    # Fixed hinge knuckles and leaf pads for the narrow protective cover.
    for index, x in enumerate((-0.075, 0.075)):
        body.visual(
            Box((0.058, 0.012, 0.012)),
            origin=Origin(xyz=(x, 0.018, 0.190)),
            material=hinge_mat,
            name=f"cover_hinge_leaf_{index}",
        )
        body.visual(
            Cylinder(radius=0.006, length=0.052),
            origin=Origin(xyz=(x, 0.018, 0.198), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_mat,
            name=f"cover_hinge_knuckle_{index}",
        )

    # Side bosses receive the rotating carry handle at the upper case corner.
    for index, y in enumerate((-0.055, 0.055)):
        body.visual(
            Cylinder(radius=0.014, length=0.018),
            origin=Origin(xyz=(0.154, y, 0.135), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_mat,
            name=f"handle_boss_{index}",
        )

    cover = model.part("control_cover")
    cover.visual(
        Box((0.250, 0.084, 0.005)),
        origin=Origin(xyz=(0.0, -0.042, 0.0065)),
        material=cover_mat,
        name="clear_panel",
    )
    cover.visual(
        Box((0.250, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, -0.084, 0.006)),
        material=cover_mat,
        name="front_lip",
    )
    for index, x in enumerate((-0.125, 0.125)):
        cover.visual(
            Box((0.006, 0.084, 0.012)),
            origin=Origin(xyz=(x, -0.042, 0.006)),
            material=cover_mat,
            name=f"side_lip_{index}",
        )
    cover.visual(
        Cylinder(radius=0.006, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_mat,
        name="center_knuckle",
    )

    handle = model.part("carry_handle")
    for index, y in enumerate((-0.055, 0.055)):
        handle.visual(
            Cylinder(radius=0.008, length=0.024),
            origin=Origin(xyz=(0.004, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_mat,
            name=f"pivot_tube_{index}",
        )
        handle.visual(
            Cylinder(radius=0.007, length=0.074),
            origin=Origin(xyz=(0.018, y, -0.038)),
            material=hinge_mat,
            name=f"side_arm_{index}",
        )
    handle.visual(
        Cylinder(radius=0.010, length=0.126),
        origin=Origin(xyz=(0.018, 0.0, -0.076), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_mat,
        name="rounded_grip",
    )

    volume_knob = model.part("volume_knob")
    volume_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.026,
                0.012,
                body_style="faceted",
                grip=KnobGrip(style="ribbed", count=14, depth=0.0007),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "volume_knob_cap",
        ),
        material=hinge_mat,
        name="knob_cap",
    )

    for name, x, material in (
        ("play_button", 0.000, accent_mat),
        ("mode_button", 0.043, rubber_mat),
    ):
        button = model.part(name)
        button.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=material,
            name="button_cap",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, -0.028, 0.186)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.04, lower=0.0, upper=0.003),
        )

    model.articulation(
        "body_to_control_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, 0.018, 0.198)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.35),
    )

    model.articulation(
        "body_to_carry_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.158, 0.0, 0.135)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    model.articulation(
        "body_to_volume_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=volume_knob,
        origin=Origin(xyz=(-0.052, -0.028, 0.186)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=5.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("control_cover")
    handle = object_model.get_part("carry_handle")
    knob = object_model.get_part("volume_knob")
    play = object_model.get_part("play_button")
    mode = object_model.get_part("mode_button")
    cover_joint = object_model.get_articulation("body_to_control_cover")
    handle_joint = object_model.get_articulation("body_to_carry_handle")
    play_joint = object_model.get_articulation("body_to_play_button")

    for index in (0, 1):
        ctx.allow_overlap(
            body,
            handle,
            elem_a=f"handle_boss_{index}",
            elem_b=f"pivot_tube_{index}",
            reason="The side handle pivot tube is captured around the molded case boss.",
        )
        ctx.expect_overlap(
            body,
            handle,
            axes="xz",
            elem_a=f"handle_boss_{index}",
            elem_b=f"pivot_tube_{index}",
            min_overlap=0.006,
            name=f"handle pivot {index} is seated on side boss",
        )

    with ctx.pose({cover_joint: 0.0, handle_joint: 0.0}):
        ctx.expect_overlap(
            cover,
            body,
            axes="xy",
            elem_a="clear_panel",
            elem_b="control_strip",
            min_overlap=0.050,
            name="closed cover spans the top control strip",
        )
        ctx.expect_gap(
            cover,
            knob,
            axis="z",
            min_gap=0.001,
            max_gap=0.020,
            positive_elem="clear_panel",
            negative_elem="knob_cap",
            name="closed cover clears the volume knob",
        )
        ctx.expect_gap(
            cover,
            play,
            axis="z",
            min_gap=0.001,
            max_gap=0.020,
            positive_elem="clear_panel",
            negative_elem="button_cap",
            name="closed cover clears the play button",
        )

    closed_cover_aabb = ctx.part_world_aabb(cover)
    closed_handle_aabb = ctx.part_world_aabb(handle)
    with ctx.pose({cover_joint: 1.15, handle_joint: 1.10}):
        open_cover_aabb = ctx.part_world_aabb(cover)
        open_handle_aabb = ctx.part_world_aabb(handle)
        ctx.expect_gap(
            handle,
            body,
            axis="x",
            min_gap=0.002,
            positive_elem="rounded_grip",
            negative_elem="rounded_case",
            name="opened handle grip clears the side case",
        )

    ctx.check(
        "control cover opens upward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.030,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )
    ctx.check(
        "carry handle swings outward",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][0] > closed_handle_aabb[1][0] + 0.030,
        details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
    )

    rest_button_pos = ctx.part_world_position(play)
    with ctx.pose({play_joint: 0.003}):
        pressed_button_pos = ctx.part_world_position(play)
    ctx.check(
        "push button travels downward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.002,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
