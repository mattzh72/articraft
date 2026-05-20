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
    Material,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jobsite_portable_speaker")

    shell = model.material("charcoal_shell", rgba=(0.13, 0.14, 0.15, 1.0))
    rubber = model.material("black_rubber", rgba=(0.025, 0.027, 0.030, 1.0))
    yellow = model.material("jobsite_yellow", rgba=(0.94, 0.65, 0.08, 1.0))
    grille_mat = model.material("black_grille", rgba=(0.015, 0.017, 0.020, 1.0))
    control_mat = model.material("control_face", rgba=(0.06, 0.065, 0.070, 1.0))
    button_mat = model.material("button_grey", rgba=(0.40, 0.43, 0.46, 1.0))
    metal = model.material("dark_metal", rgba=(0.19, 0.20, 0.21, 1.0))

    body = model.part("body")

    # Broad portable worksite-radio body: about 56 cm wide and 36 cm tall.
    body.visual(
        Box((0.560, 0.260, 0.340)),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=shell,
        name="main_shell",
    )
    body.visual(
        Box((0.520, 0.018, 0.088)),
        origin=Origin(xyz=(0.0, -0.142, 0.298)),
        material=control_mat,
        name="control_face",
    )

    # Deep framed grille recess with a black back plane and a real perforated face.
    body.visual(
        Box((0.450, 0.008, 0.184)),
        origin=Origin(xyz=(0.0, -0.134, 0.148)),
        material=grille_mat,
        name="grille_shadow",
    )
    body.visual(
        Box((0.480, 0.036, 0.030)),
        origin=Origin(xyz=(0.0, -0.146, 0.239)),
        material=rubber,
        name="grille_top_lip",
    )
    body.visual(
        Box((0.480, 0.036, 0.030)),
        origin=Origin(xyz=(0.0, -0.146, 0.057)),
        material=rubber,
        name="grille_bottom_lip",
    )
    body.visual(
        Box((0.034, 0.036, 0.210)),
        origin=Origin(xyz=(-0.221, -0.146, 0.148)),
        material=rubber,
        name="grille_side_lip_0",
    )
    body.visual(
        Box((0.034, 0.036, 0.210)),
        origin=Origin(xyz=(0.221, -0.146, 0.148)),
        material=rubber,
        name="grille_side_lip_1",
    )
    grille_mesh = mesh_from_geometry(
        PerforatedPanelGeometry(
            (0.416, 0.168),
            0.006,
            hole_diameter=0.010,
            pitch=(0.020, 0.018),
            frame=0.012,
            corner_radius=0.010,
            stagger=True,
        ),
        "front_speaker_grille",
    )
    body.visual(
        grille_mesh,
        origin=Origin(xyz=(0.0, -0.164, 0.148), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grille_mat,
        name="front_grille",
    )

    # Reinforced rubber/yellow corner armor, protruding past the main shell.
    for index, (x, y) in enumerate(
        ((-0.292, -0.137), (0.292, -0.137), (-0.292, 0.137), (0.292, 0.137))
    ):
        body.visual(
            Box((0.056, 0.042, 0.342)),
            origin=Origin(xyz=(x, y, 0.171)),
            material=yellow,
            name=f"corner_bumper_{index}",
        )
        body.visual(
            Box((0.044, 0.048, 0.054)),
            origin=Origin(xyz=(x, y, 0.027)),
            material=rubber,
            name=f"lower_corner_pad_{index}",
        )
        body.visual(
            Box((0.044, 0.048, 0.054)),
            origin=Origin(xyz=(x, y, 0.315)),
            material=rubber,
            name=f"upper_corner_pad_{index}",
        )

    # Side hinge landing pads for the two fold-flat handles.
    for index, x in enumerate((-0.290, 0.290)):
        body.visual(
            Box((0.028, 0.052, 0.244)),
            origin=Origin(xyz=(x, -0.060, 0.180)),
            material=rubber,
            name=f"side_hinge_pad_{index}",
        )
        body.visual(
            Box((0.040, 0.070, 0.028)),
            origin=Origin(xyz=(x, -0.060, 0.040)),
            material=metal,
            name=f"lower_hinge_socket_{index}",
        )
        body.visual(
            Box((0.040, 0.070, 0.028)),
            origin=Origin(xyz=(x, -0.060, 0.320)),
            material=metal,
            name=f"upper_hinge_socket_{index}",
        )

    # A visible front axle/boss for the continuously rotating selector wheel.
    body.visual(
        Cylinder(radius=0.013, length=0.030),
        origin=Origin(xyz=(-0.180, -0.148, 0.297), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="selector_axle",
    )
    body.visual(
        Box((0.104, 0.010, 0.104)),
        origin=Origin(xyz=(-0.180, -0.153, 0.297)),
        material=control_mat,
        name="wheel_recess",
    )

    # Three distinct button pockets in the upper control face.
    for index, x in enumerate((0.050, 0.118, 0.186)):
        body.visual(
            Box((0.060, 0.006, 0.034)),
            origin=Origin(xyz=(x, -0.148, 0.292)),
            material=grille_mat,
            name=f"button_pocket_{index}",
        )

    # Fold-flat U handles on the two side faces.  Positive motion swings them outward.
    for index, (x, axis_z) in enumerate(((-0.316, 1.0), (0.316, -1.0))):
        handle = model.part(f"side_handle_{index}")
        handle.visual(
            Cylinder(radius=0.012, length=0.240),
            origin=Origin(xyz=(0.0, 0.0, 0.120)),
            material=metal,
            name="hinge_bar",
        )
        handle.visual(
            Box((0.015, 0.164, 0.018)),
            origin=Origin(xyz=(0.0, 0.082, 0.035)),
            material=rubber,
            name="lower_arm",
        )
        handle.visual(
            Box((0.015, 0.164, 0.018)),
            origin=Origin(xyz=(0.0, 0.082, 0.205)),
            material=rubber,
            name="upper_arm",
        )
        handle.visual(
            Box((0.024, 0.026, 0.190)),
            origin=Origin(xyz=(0.0, 0.166, 0.120)),
            material=rubber,
            name="grip",
        )
        model.articulation(
            f"body_to_side_handle_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=handle,
            origin=Origin(xyz=(x, -0.060, 0.060)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.35),
        )

    # Large selector wheel near the upper front edge.
    selector = model.part("selector_wheel")
    selector_mesh = mesh_from_geometry(
        KnobGeometry(
            0.086,
            0.034,
            body_style="skirted",
            top_diameter=0.070,
            edge_radius=0.002,
            grip=KnobGrip(style="ribbed", count=32, depth=0.0022, width=0.0030),
            indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
            bore=KnobBore(style="round", diameter=0.030),
        ),
        "selector_wheel",
    )
    selector.visual(
        selector_mesh,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=yellow,
        name="selector_cap",
    )
    model.articulation(
        "body_to_selector_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(-0.180, -0.178, 0.297)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    # Three independent front push buttons on the control face.
    for index, x in enumerate((0.050, 0.118, 0.186)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.048, 0.014, 0.026)),
            origin=Origin(),
            material=button_mat,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, -0.158, 0.292)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5.0, velocity=0.05, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    selector = object_model.get_part("selector_wheel")
    selector_joint = object_model.get_articulation("body_to_selector_wheel")

    ctx.check(
        "selector wheel is continuous",
        selector_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"selector_joint={selector_joint.articulation_type}",
    )
    ctx.expect_overlap(
        selector,
        body,
        axes="xz",
        elem_a="selector_cap",
        elem_b="wheel_recess",
        min_overlap=0.050,
        name="selector wheel sits in upper front recess",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        ctx.check(
            f"button {index} is prismatic",
            button_joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"button_joint={button_joint.articulation_type}",
        )
        ctx.expect_contact(
            button,
            body,
            elem_a="button_cap",
            elem_b="button_pocket_" + str(index),
            contact_tol=0.003,
            name=f"button {index} seated in control face",
        )
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.006}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button {index} depresses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.004,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    for index in range(2):
        handle = object_model.get_part(f"side_handle_{index}")
        handle_joint = object_model.get_articulation(f"body_to_side_handle_{index}")
        ctx.expect_contact(
            handle,
            body,
            elem_a="hinge_bar",
            elem_b=f"side_hinge_pad_{index}",
            contact_tol=0.004,
            name=f"side handle {index} hinge is mounted",
        )
        rest_aabb = ctx.part_element_world_aabb(handle, elem="grip")
        with ctx.pose({handle_joint: 1.0}):
            open_aabb = ctx.part_element_world_aabb(handle, elem="grip")
        if index == 0:
            moved_outward = (
                rest_aabb is not None
                and open_aabb is not None
                and open_aabb[0][0] < rest_aabb[0][0] - 0.050
            )
        else:
            moved_outward = (
                rest_aabb is not None
                and open_aabb is not None
                and open_aabb[1][0] > rest_aabb[1][0] + 0.050
            )
        ctx.check(
            f"side handle {index} swings outward",
            moved_outward,
            details=f"rest={rest_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
