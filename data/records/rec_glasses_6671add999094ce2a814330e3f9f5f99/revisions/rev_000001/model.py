from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


FRAME_TILT = (math.pi / 2.0, 0.0, 0.0)


def _rim_mesh(name: str):
    """A thin rounded spectacle rim with an open lens aperture."""
    outer = rounded_rect_profile(0.052, 0.036, 0.010, corner_segments=8)
    inner = rounded_rect_profile(0.044, 0.028, 0.007, corner_segments=8)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, [inner], 0.006, center=True),
        name,
    )


def _lens_mesh(name: str):
    """Slightly oversized transparent lens, captured under the rim lip."""
    return mesh_from_geometry(
        ExtrudeGeometry(superellipse_profile(0.045, 0.029, exponent=2.7, segments=56), 0.0018, center=True),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_travel_glasses")

    frame_mat = model.material("dark_gunmetal", rgba=(0.035, 0.038, 0.043, 1.0))
    temple_mat = model.material("matte_black", rgba=(0.015, 0.015, 0.017, 1.0))
    slide_mat = model.material("brushed_steel", rgba=(0.62, 0.63, 0.62, 1.0))
    lens_mat = model.material("pale_blue_lens", rgba=(0.62, 0.82, 0.95, 0.36))
    rubber_mat = model.material("soft_tip_black", rgba=(0.004, 0.004, 0.005, 1.0))

    # Root front frame: two small rounded rims, a narrow bridge, clear lenses,
    # and alternating hinge knuckles at the outer frame corners.
    front = model.part("front_frame")
    for side, x in (("0", -0.029), ("1", 0.029)):
        front.visual(
            _rim_mesh(f"rim_{side}"),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=FRAME_TILT),
            material=frame_mat,
            name=f"rim_{side}",
        )
        front.visual(
            _lens_mesh(f"lens_{side}"),
            origin=Origin(xyz=(x, -0.0005, 0.0), rpy=FRAME_TILT),
            material=lens_mat,
            name=f"lens_{side}",
        )

    front.visual(
        Box((0.017, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=frame_mat,
        name="upper_bridge",
    )
    front.visual(
        Box((0.010, 0.005, 0.005)),
        origin=Origin(xyz=(0.0, 0.001, -0.005)),
        material=frame_mat,
        name="nose_bridge",
    )

    hinge_x = 0.064
    for side_index, sign in enumerate((-1.0, 1.0)):
        x = sign * hinge_x
        tab_x = sign * 0.059
        for z_name, z in (("upper", 0.009), ("lower", -0.009)):
            front.visual(
                Box((0.014, 0.006, 0.006)),
                origin=Origin(xyz=(tab_x, 0.004, z)),
                material=frame_mat,
                name=f"hinge_tab_{side_index}_{z_name}",
            )
            front.visual(
                Cylinder(radius=0.0032, length=0.008),
                origin=Origin(xyz=(x, 0.004, z)),
                material=frame_mat,
                name=f"frame_knuckle_{side_index}_{z_name}",
            )
        front.visual(
            Cylinder(radius=0.0025, length=0.0016),
            origin=Origin(xyz=(x, 0.004, 0.0138)),
            material=slide_mat,
            name=f"pin_head_{side_index}_upper",
        )
        front.visual(
            Cylinder(radius=0.0025, length=0.0016),
            origin=Origin(xyz=(x, 0.004, -0.0138)),
            material=slide_mat,
            name=f"pin_head_{side_index}_lower",
        )

    # Each side has a short rectangular hollow sleeve hinged to the frame.  The
    # rear temple segment runs through the sleeve cavity and remains retained
    # even when pulled out for use.
    sleeve_length = 0.060
    sleeve_width = 0.0072
    sleeve_height = 0.0056
    wall = 0.0011
    slide_upper = 0.030

    for side_index, sign in enumerate((-1.0, 1.0)):
        temple = model.part(f"temple_{side_index}")
        rear = model.part(f"rear_temple_{side_index}")

        temple.visual(
            Cylinder(radius=0.0028, length=0.010),
            origin=Origin(xyz=(0.0, 0.004, 0.0)),
            material=frame_mat,
            name="temple_knuckle",
        )
        temple.visual(
            Box((0.0065, 0.012, 0.0050)),
            origin=Origin(xyz=(0.0, 0.010, 0.0)),
            material=frame_mat,
            name="hinge_leaf",
        )
        temple.visual(
            Box((sleeve_width, sleeve_length, wall)),
            origin=Origin(xyz=(0.0, 0.009 + sleeve_length / 2.0, sleeve_height / 2.0 - wall / 2.0)),
            material=temple_mat,
            name="top_wall",
        )
        temple.visual(
            Box((sleeve_width, sleeve_length, wall)),
            origin=Origin(xyz=(0.0, 0.009 + sleeve_length / 2.0, -sleeve_height / 2.0 + wall / 2.0)),
            material=temple_mat,
            name="bottom_wall",
        )
        temple.visual(
            Box((wall, sleeve_length, sleeve_height)),
            origin=Origin(xyz=(sleeve_width / 2.0 - wall / 2.0, 0.009 + sleeve_length / 2.0, 0.0)),
            material=temple_mat,
            name="outer_wall",
        )
        temple.visual(
            Box((wall, sleeve_length, sleeve_height)),
            origin=Origin(xyz=(-sleeve_width / 2.0 + wall / 2.0, 0.009 + sleeve_length / 2.0, 0.0)),
            material=temple_mat,
            name="inner_wall",
        )
        lip_y = 0.009 + sleeve_length + 0.002
        temple.visual(
            Box((sleeve_width + 0.0015, 0.004, wall)),
            origin=Origin(xyz=(0.0, lip_y, sleeve_height / 2.0 - wall / 2.0)),
            material=frame_mat,
            name="rear_lip_top",
        )
        temple.visual(
            Box((sleeve_width + 0.0015, 0.004, wall)),
            origin=Origin(xyz=(0.0, lip_y, -sleeve_height / 2.0 + wall / 2.0)),
            material=frame_mat,
            name="rear_lip_bottom",
        )
        temple.visual(
            Box((wall, 0.004, sleeve_height + 0.0010)),
            origin=Origin(xyz=(sleeve_width / 2.0 - wall / 2.0, lip_y, 0.0)),
            material=frame_mat,
            name="rear_lip_outer",
        )
        temple.visual(
            Box((wall, 0.004, sleeve_height + 0.0010)),
            origin=Origin(xyz=(-sleeve_width / 2.0 + wall / 2.0, lip_y, 0.0)),
            material=frame_mat,
            name="rear_lip_inner",
        )

        rear.visual(
            Box((0.0030, 0.090, 0.0034)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=slide_mat,
            name="inner_bar",
        )
        rear.visual(
            Box((0.0034, 0.015, 0.011)),
            origin=Origin(xyz=(0.0, 0.041, -0.0055)),
            material=rubber_mat,
            name="ear_hook",
        )
        rear.visual(
            Box((0.0040, 0.010, 0.0040)),
            origin=Origin(xyz=(0.0, 0.047, -0.011)),
            material=rubber_mat,
            name="soft_tip",
        )

        model.articulation(
            f"frame_to_temple_{side_index}",
            ArticulationType.REVOLUTE,
            parent=front,
            child=temple,
            origin=Origin(xyz=(sign * hinge_x, 0.004, 0.0)),
            axis=(0.0, 0.0, sign),
            motion_limits=MotionLimits(effort=0.25, velocity=3.0, lower=0.0, upper=1.65),
        )
        model.articulation(
            f"temple_to_rear_{side_index}",
            ArticulationType.PRISMATIC,
            parent=temple,
            child=rear,
            origin=Origin(xyz=(0.0, 0.009 + sleeve_length, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.12, lower=0.0, upper=slide_upper),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    for side_index in (0, 1):
        temple = object_model.get_part(f"temple_{side_index}")
        rear = object_model.get_part(f"rear_temple_{side_index}")
        hinge = object_model.get_articulation(f"frame_to_temple_{side_index}")
        slide = object_model.get_articulation(f"temple_to_rear_{side_index}")

        ctx.expect_gap(
            temple,
            rear,
            axis="z",
            positive_elem="top_wall",
            negative_elem="inner_bar",
            max_gap=0.0001,
            max_penetration=0.000001,
            name=f"rear segment clears sleeve top {side_index}",
        )
        ctx.expect_gap(
            rear,
            temple,
            axis="z",
            positive_elem="inner_bar",
            negative_elem="bottom_wall",
            max_gap=0.0001,
            max_penetration=0.000001,
            name=f"rear segment clears sleeve bottom {side_index}",
        )
        ctx.expect_overlap(
            rear,
            temple,
            axes="y",
            elem_a="inner_bar",
            elem_b="top_wall",
            min_overlap=0.040,
            name=f"collapsed rear segment remains inserted {side_index}",
        )

        rest_pos = ctx.part_world_position(rear)
        with ctx.pose({slide: 0.030}):
            ctx.expect_overlap(
                rear,
                temple,
                axes="y",
                elem_a="inner_bar",
                elem_b="top_wall",
                min_overlap=0.012,
                name=f"extended rear segment remains captured {side_index}",
            )
            extended_pos = ctx.part_world_position(rear)
        ctx.check(
            f"rear temple slides backward {side_index}",
            rest_pos is not None and extended_pos is not None and extended_pos[1] > rest_pos[1] + 0.025,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

        with ctx.pose({hinge: 1.25}):
            aabb = ctx.part_world_aabb(temple)
        if side_index == 0:
            folded_inward = aabb is not None and aabb[1][0] > -0.030
        else:
            folded_inward = aabb is not None and aabb[0][0] < 0.030
        ctx.check(
            f"hinged temple folds inward {side_index}",
            folded_inward,
            details=f"folded aabb={aabb}",
        )

    return ctx.report()


object_model = build_object_model()
