from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_sewing_box")

    body_paint = model.material("body_paint", rgba=(0.70, 0.62, 0.52, 1.0))
    lid_paint = model.material("lid_paint", rgba=(0.78, 0.71, 0.61, 1.0))
    lining = model.material("lining", rgba=(0.87, 0.82, 0.74, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.67, 0.57, 0.34, 1.0))

    outer_width = 0.260
    outer_depth = 0.180
    body_height = 0.105
    wall_thickness = 0.006
    bottom_thickness = 0.006
    side_wall_height = body_height - bottom_thickness

    lid_width = 0.264
    lid_depth = 0.186
    lid_panel_thickness = 0.008
    lid_skirt_thickness = 0.003
    lid_skirt_drop = 0.018

    hinge_radius = 0.005
    hinge_axis_backset = 0.004
    hinge_axis_y = outer_depth * 0.5 + hinge_axis_backset
    hinge_axis_z = body_height + 0.001 + lid_panel_thickness * 0.5
    lid_rear_overhang = -0.006
    lid_panel_center_y = -(lid_depth * 0.5 - lid_rear_overhang)

    body = model.part("body")
    body.visual(
        Box((outer_width, outer_depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness * 0.5)),
        material=body_paint,
        name="body_bottom",
    )
    body.visual(
        Box((outer_width, wall_thickness, side_wall_height)),
        origin=Origin(
            xyz=(0.0, -(outer_depth - wall_thickness) * 0.5, bottom_thickness + side_wall_height * 0.5)
        ),
        material=body_paint,
        name="front_wall",
    )
    body.visual(
        Box((outer_width, wall_thickness, side_wall_height)),
        origin=Origin(
            xyz=(0.0, (outer_depth - wall_thickness) * 0.5, bottom_thickness + side_wall_height * 0.5)
        ),
        material=body_paint,
        name="back_wall",
    )
    body.visual(
        Box((wall_thickness, outer_depth - wall_thickness * 2.0, side_wall_height)),
        origin=Origin(
            xyz=(
                -(outer_width - wall_thickness) * 0.5,
                0.0,
                bottom_thickness + side_wall_height * 0.5,
            )
        ),
        material=body_paint,
        name="left_wall",
    )
    body.visual(
        Box((wall_thickness, outer_depth - wall_thickness * 2.0, side_wall_height)),
        origin=Origin(
            xyz=(
                (outer_width - wall_thickness) * 0.5,
                0.0,
                bottom_thickness + side_wall_height * 0.5,
            )
        ),
        material=body_paint,
        name="right_wall",
    )
    body.visual(
        Box((outer_width - 0.020, outer_depth - 0.020, 0.0015)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness + 0.00075)),
        material=lining,
        name="interior_base",
    )
    for name, hinge_x in (("left_body_hinge", -0.071), ("right_body_hinge", 0.071)):
        body.visual(
            Box((0.058, 0.003, 0.022)),
            origin=Origin(xyz=(hinge_x, outer_depth * 0.5 + 0.0015, body_height - 0.011)),
            material=hinge_metal,
            name=f"{name}_leaf",
        )
        body.visual(
            Box((0.052, 0.004, 0.006)),
            origin=Origin(xyz=(hinge_x, outer_depth * 0.5 + 0.0025, body_height + 0.0005)),
            material=hinge_metal,
            name=f"{name}_bridge",
        )
        body.visual(
            Cylinder(radius=hinge_radius, length=0.052),
            origin=Origin(
                xyz=(hinge_x, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=hinge_metal,
            name=f"{name}_barrel",
        )
    body.inertial = Inertial.from_geometry(
        Box((outer_width, outer_depth, body_height)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_width, lid_depth, lid_panel_thickness)),
        origin=Origin(xyz=(0.0, lid_panel_center_y, 0.0)),
        material=lid_paint,
        name="lid_top",
    )
    lid.visual(
        Box((lid_width - 0.018, lid_depth - 0.018, 0.0015)),
        origin=Origin(xyz=(0.0, lid_panel_center_y, -lid_panel_thickness * 0.5 + 0.00075)),
        material=lining,
        name="lid_lining",
    )
    lid.visual(
        Box((lid_width, lid_skirt_thickness, lid_skirt_drop)),
        origin=Origin(
            xyz=(
                0.0,
                lid_panel_center_y - (lid_depth * 0.5 - lid_skirt_thickness * 0.5),
                -(lid_panel_thickness * 0.5 + lid_skirt_drop * 0.5),
            )
        ),
        material=lid_paint,
        name="lid_front_rail",
    )
    lid.visual(
        Box((lid_skirt_thickness, 0.184, lid_skirt_drop)),
        origin=Origin(
            xyz=(
                -(lid_width * 0.5 - lid_skirt_thickness * 0.5),
                lid_panel_center_y - 0.001,
                -(lid_panel_thickness * 0.5 + lid_skirt_drop * 0.5),
            )
        ),
        material=lid_paint,
        name="lid_left_rail",
    )
    lid.visual(
        Box((lid_skirt_thickness, 0.184, lid_skirt_drop)),
        origin=Origin(
            xyz=(
                lid_width * 0.5 - lid_skirt_thickness * 0.5,
                lid_panel_center_y - 0.001,
                -(lid_panel_thickness * 0.5 + lid_skirt_drop * 0.5),
            )
        ),
        material=lid_paint,
        name="lid_right_rail",
    )
    lid.visual(
        Box((0.082, 0.003, 0.018)),
        origin=Origin(xyz=(0.0, 0.0045, -0.012)),
        material=hinge_metal,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Box((0.082, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, -0.001, -0.006)),
        material=hinge_metal,
        name="lid_hinge_bridge",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_metal,
        name="lid_hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, 0.026)),
        mass=0.26,
        origin=Origin(xyz=(0.0, lid_panel_center_y, -0.007)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=2.2,
        ),
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

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_top",
            negative_elem="front_wall",
            max_gap=0.004,
            max_penetration=0.0,
            name="lid closes just above the box rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_top",
            elem_b="body_bottom",
            min_overlap=0.16,
            name="lid covers the sewing box footprint",
        )

    with ctx.pose({hinge: 1.9}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_front_rail",
            negative_elem="back_wall",
            min_gap=0.045,
            name="opened lid lifts the front edge well above the box",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
