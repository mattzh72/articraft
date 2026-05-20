from __future__ import annotations

from math import atan2, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


PLATE_TO_X = (pi / 2.0, 0.0, pi / 2.0)


def _rounded_plate(width: float, height: float, thickness: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, height, radius, corner_segments=8),
            thickness,
            center=True,
        ),
        name,
    )


def _rounded_frame(
    outer_w: float,
    outer_h: float,
    inner_w: float,
    inner_h: float,
    thickness: float,
    outer_r: float,
    inner_r: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_w, outer_h, outer_r, corner_segments=10),
            [rounded_rect_profile(inner_w, inner_h, inner_r, corner_segments=10)],
            thickness,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_glove_compartment")

    off_white = model.material("gelcoat_off_white", rgba=(0.86, 0.88, 0.83, 1.0))
    inner_gray = model.material("shadowed_inner_gray", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.012, 0.012, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_plastic = model.material("black_plastic", rgba=(0.025, 0.028, 0.03, 1.0))
    latch_plastic = model.material("latch_black", rgba=(0.004, 0.004, 0.005, 1.0))

    dash = model.part("dash_panel")

    # A molded dash panel with a rounded, rubber-gasketed opening.
    dash.visual(
        _rounded_frame(0.58, 0.32, 0.44, 0.205, 0.018, 0.040, 0.025, "dash_face_frame"),
        origin=Origin(xyz=(-0.009, 0.0, 0.0), rpy=PLATE_TO_X),
        material=off_white,
        name="dash_face_frame",
    )
    dash.visual(
        _rounded_frame(0.475, 0.240, 0.425, 0.190, 0.006, 0.030, 0.022, "rubber_gasket"),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=PLATE_TO_X),
        material=rubber,
        name="rubber_gasket",
    )

    # Recessed rigid storage box behind the dash opening.  The five walls touch
    # each other and the front frame, leaving the user-facing front open.
    dash.visual(
        Box((0.014, 0.438, 0.204)),
        origin=Origin(xyz=(-0.242, 0.0, 0.0)),
        material=inner_gray,
        name="box_back_wall",
    )
    dash.visual(
        Box((0.225, 0.014, 0.204)),
        origin=Origin(xyz=(-0.1225, 0.219, 0.0)),
        material=off_white,
        name="box_side_wall_0",
    )
    dash.visual(
        Box((0.225, 0.014, 0.204)),
        origin=Origin(xyz=(-0.1225, -0.219, 0.0)),
        material=off_white,
        name="box_side_wall_1",
    )
    dash.visual(
        Box((0.225, 0.438, 0.014)),
        origin=Origin(xyz=(-0.1225, 0.0, 0.102)),
        material=off_white,
        name="box_top_wall",
    )
    dash.visual(
        Box((0.225, 0.438, 0.014)),
        origin=Origin(xyz=(-0.1225, 0.0, -0.102)),
        material=off_white,
        name="box_bottom_wall",
    )

    hinge_x = 0.006
    hinge_z = 0.132
    dash.visual(
        Cylinder(radius=0.0055, length=0.505),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="upper_hinge_pin",
    )
    dash.visual(
        Box((0.010, 0.480, 0.010)),
        origin=Origin(xyz=(-0.001, 0.0, hinge_z - 0.016)),
        material=stainless,
        name="upper_hinge_leaf",
    )
    for i, y in enumerate((-0.235, 0.235)):
        dash.visual(
            Box((0.012, 0.020, 0.034)),
            origin=Origin(xyz=(0.000, y, hinge_z - 0.013)),
            material=stainless,
            name=f"hinge_support_{i}",
        )

    # Fixed side pivot pins and small molded bosses for the stay arms.
    box_pivot_x = -0.035
    box_pivot_z = -0.060
    arm_y = 0.315
    for i, side in enumerate((1.0, -1.0)):
        dash.visual(
            Cylinder(radius=0.006, length=0.044),
            origin=Origin(
                xyz=(box_pivot_x, side * 0.315, box_pivot_z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=stainless,
            name=f"box_pivot_{i}",
        )
        dash.visual(
            Cylinder(radius=0.017, length=0.010),
            origin=Origin(
                xyz=(box_pivot_x, side * 0.292, box_pivot_z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=off_white,
            name=f"pivot_boss_{i}",
        )

    dash.visual(
        Box((0.012, 0.095, 0.018)),
        origin=Origin(xyz=(-0.010, 0.0, -0.103)),
        material=stainless,
        name="latch_keeper",
    )

    lid = model.part("lid")
    lid_w = 0.470
    lid_h = 0.226
    lid_t = 0.018
    lid_center_x = 0.012
    lid_center_z = -lid_h / 2.0
    lid.visual(
        _rounded_plate(lid_w, lid_h, lid_t, 0.026, "lid_shell"),
        origin=Origin(xyz=(lid_center_x, 0.0, lid_center_z), rpy=PLATE_TO_X),
        material=off_white,
        name="lid_shell",
    )
    lid.visual(
        _rounded_plate(0.385, 0.130, 0.004, 0.020, "raised_lid_panel"),
        origin=Origin(xyz=(0.0225, 0.0, -0.114), rpy=PLATE_TO_X),
        material=Material("slightly_warmer_lid", rgba=(0.91, 0.92, 0.87, 1.0)),
        name="raised_lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.034, length=0.005),
        origin=Origin(xyz=(0.026, 0.0, -0.114), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_plastic,
        name="latch_bezel",
    )
    lid.visual(
        Cylinder(radius=0.009, length=0.108),
        origin=Origin(xyz=(0.000, -0.168, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_barrel_0",
    )
    lid.visual(
        Cylinder(radius=0.009, length=0.108),
        origin=Origin(xyz=(0.000, 0.168, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_barrel_1",
    )
    lid.visual(
        Box((0.006, 0.440, 0.016)),
        origin=Origin(xyz=(0.010, 0.0, -0.006)),
        material=stainless,
        name="lid_hinge_leaf",
    )

    lid_pin_local_x = 0.014
    lid_pin_local_z = -0.190
    for i, side in enumerate((1.0, -1.0)):
        lid.visual(
            Box((0.010, 0.080, 0.022)),
            origin=Origin(xyz=(lid_pin_local_x, side * 0.270, lid_pin_local_z)),
            material=stainless,
            name=f"side_tab_{i}",
        )
        lid.visual(
            Cylinder(radius=0.006, length=0.034),
            origin=Origin(
                xyz=(lid_pin_local_x, side * 0.305, lid_pin_local_z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=stainless,
            name=f"side_pin_{i}",
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=dash,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.8, lower=0.0, upper=1.15),
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.025, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=latch_plastic,
        name="button_cap",
    )
    latch.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="button_stem",
    )
    model.articulation(
        "latch_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=latch,
        origin=Origin(xyz=(0.034, 0.0, -0.114)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.010),
    )

    # Stay arms rotate around box-side pivots and include a visible slotted bar
    # that captures the lid-side pin through the opening sweep.
    closed_pin_x = hinge_x + lid_pin_local_x
    closed_pin_z = hinge_z + lid_pin_local_z
    arm_rest_angle = -atan2(closed_pin_z - box_pivot_z, closed_pin_x - box_pivot_x)
    for i, side in enumerate((1.0, -1.0)):
        arm = model.part(f"support_arm_{i}")
        arm.visual(
            Box((0.250, 0.006, 0.014)),
            origin=Origin(xyz=(0.125, 0.0, 0.0)),
            material=stainless,
            name="slotted_bar",
        )
        arm.visual(
            Box((0.175, 0.007, 0.003)),
            origin=Origin(xyz=(0.138, side * 0.0038, 0.0)),
            material=rubber,
            name="slot_shadow",
        )
        arm.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name="proximal_eye",
        )
        arm.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(xyz=(0.235, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name="distal_eye",
        )
        model.articulation(
            f"arm_pivot_{i}",
            ArticulationType.REVOLUTE,
            parent=dash,
            child=arm,
            origin=Origin(
                xyz=(box_pivot_x, side * arm_y, box_pivot_z),
                rpy=(0.0, arm_rest_angle, 0.0),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=1.0, lower=0.0, upper=0.60),
            mimic=Mimic(joint="lid_hinge", multiplier=0.43, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    dash = object_model.get_part("dash_panel")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    arm_0 = object_model.get_part("support_arm_0")
    arm_1 = object_model.get_part("support_arm_1")
    lid_hinge = object_model.get_articulation("lid_hinge")
    latch_slide = object_model.get_articulation("latch_slide")

    ctx.allow_overlap(
        dash,
        lid,
        elem_a="upper_hinge_pin",
        elem_b="hinge_barrel_0",
        reason="The stainless hinge pin is intentionally captured inside the lid hinge barrel.",
    )
    ctx.allow_overlap(
        dash,
        lid,
        elem_a="upper_hinge_pin",
        elem_b="hinge_barrel_1",
        reason="The stainless hinge pin is intentionally captured inside the second lid hinge barrel.",
    )
    for i, arm in enumerate((arm_0, arm_1)):
        ctx.allow_overlap(
            dash,
            arm,
            elem_a=f"box_pivot_{i}",
            elem_b="proximal_eye",
            reason="The support-arm eye rotates around a captured box-side pivot pin.",
        )
        ctx.allow_overlap(
            lid,
            arm,
            elem_a=f"side_pin_{i}",
            elem_b="slotted_bar",
            reason="The lid-side pin is captured in the simplified slotted stay-arm bar.",
        )
        ctx.allow_overlap(
            lid,
            arm,
            elem_a=f"side_pin_{i}",
            elem_b="slot_shadow",
            reason="The dark slot insert shares the same stay-arm slot that the lid-side pin passes through.",
        )
    ctx.allow_overlap(
        lid,
        latch,
        elem_a="lid_shell",
        elem_b="button_stem",
        reason="The push-latch stem intentionally passes through the simplified lid bore.",
    )
    ctx.allow_overlap(
        lid,
        latch,
        elem_a="latch_bezel",
        elem_b="button_stem",
        reason="The push-latch stem is nested through the bezel opening represented as a solid ring proxy.",
    )

    with ctx.pose({lid_hinge: 0.0, latch_slide: 0.0}):
        ctx.expect_gap(
            lid,
            dash,
            axis="x",
            positive_elem="lid_shell",
            negative_elem="rubber_gasket",
            min_gap=0.0005,
            max_gap=0.006,
            name="closed lid sits just proud of gasket",
        )
        ctx.expect_overlap(
            lid,
            dash,
            axes="yz",
            elem_a="lid_shell",
            elem_b="rubber_gasket",
            min_overlap=0.18,
            name="lid covers gasketed dash opening",
        )
        ctx.expect_overlap(
            lid,
            dash,
            axes="y",
            elem_a="hinge_barrel_0",
            elem_b="upper_hinge_pin",
            min_overlap=0.075,
            name="first hinge barrel is retained on upper pin",
        )
        ctx.expect_overlap(
            lid,
            dash,
            axes="y",
            elem_a="hinge_barrel_1",
            elem_b="upper_hinge_pin",
            min_overlap=0.075,
            name="second hinge barrel is retained on upper pin",
        )
        for i, arm in enumerate((arm_0, arm_1)):
            ctx.expect_overlap(
                arm,
                dash,
                axes="xz",
                elem_a="proximal_eye",
                elem_b=f"box_pivot_{i}",
                min_overlap=0.010,
                name=f"support arm {i} pivots on box pin",
            )
            ctx.expect_overlap(
                lid,
                arm,
                axes="xz",
                elem_a=f"side_pin_{i}",
                elem_b="slotted_bar",
                min_overlap=0.006,
                name=f"support arm {i} captures lid pin closed",
            )
            ctx.expect_overlap(
                lid,
                arm,
                axes="xz",
                elem_a=f"side_pin_{i}",
                elem_b="slot_shadow",
                min_overlap=0.004,
                name=f"support arm {i} slot insert surrounds lid pin",
            )
        ctx.expect_overlap(
            latch,
            lid,
            axes="x",
            elem_a="button_stem",
            elem_b="lid_shell",
            min_overlap=0.006,
            name="push latch stem passes through lid",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    closed_arm_aabb = ctx.part_element_world_aabb(arm_0, elem="slotted_bar")
    rest_latch_pos = ctx.part_world_position(latch)
    with ctx.pose({lid_hinge: 1.15}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
        open_arm_aabb = ctx.part_element_world_aabb(arm_0, elem="slotted_bar")
        ctx.check(
            "lid opens upward and outward",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][0] > closed_lid_aabb[1][0] + 0.13
            and open_lid_aabb[0][2] > closed_lid_aabb[0][2] + 0.08,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )
        ctx.check(
            "support arm follows lid toward open stay angle",
            closed_arm_aabb is not None
            and open_arm_aabb is not None
            and open_arm_aabb[1][2] > closed_arm_aabb[1][2] + 0.08,
            details=f"closed={closed_arm_aabb}, open={open_arm_aabb}",
        )
        for i, arm in enumerate((arm_0, arm_1)):
            ctx.expect_overlap(
                lid,
                arm,
                axes="xz",
                elem_a=f"side_pin_{i}",
                elem_b="slotted_bar",
                min_overlap=0.004,
                name=f"support arm {i} still captures lid pin open",
            )

    with ctx.pose({latch_slide: 0.008}):
        pressed_latch_pos = ctx.part_world_position(latch)
        ctx.check(
            "push latch translates inward",
            rest_latch_pos is not None
            and pressed_latch_pos is not None
            and pressed_latch_pos[0] < rest_latch_pos[0] - 0.006,
            details=f"rest={rest_latch_pos}, pressed={pressed_latch_pos}",
        )

    return ctx.report()


object_model = build_object_model()
