from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sewing_box")

    body_depth = 0.22
    body_width = 0.32
    body_height = 0.14
    wall_thickness = 0.010
    bottom_thickness = 0.008

    band_thickness = 0.012
    band_width = 0.292
    band_height = 0.036

    lid_depth = body_depth + 0.004
    lid_width = body_width + 0.012
    lid_thickness = 0.012
    lid_clearance = 0.0008

    wood = model.material("wood", rgba=(0.70, 0.55, 0.38, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.50, 0.37, 0.24, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.62, 0.32, 1.0))

    shell = model.part("main_shell")
    shell.visual(
        Box((body_depth, body_width, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness / 2.0)),
        material=wood,
        name="bottom_panel",
    )
    shell.visual(
        Box((wall_thickness, body_width, body_height - bottom_thickness)),
        origin=Origin(
            xyz=(
                body_depth / 2.0 - wall_thickness / 2.0,
                0.0,
                bottom_thickness + (body_height - bottom_thickness) / 2.0,
            )
        ),
        material=wood,
        name="front_wall",
    )
    shell.visual(
        Box((wall_thickness, body_width, body_height - bottom_thickness)),
        origin=Origin(
            xyz=(
                -body_depth / 2.0 + wall_thickness / 2.0,
                0.0,
                bottom_thickness + (body_height - bottom_thickness) / 2.0,
            )
        ),
        material=wood,
        name="rear_wall",
    )
    shell.visual(
        Box((body_depth - 2.0 * wall_thickness, wall_thickness, body_height - bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                body_width / 2.0 - wall_thickness / 2.0,
                bottom_thickness + (body_height - bottom_thickness) / 2.0,
            )
        ),
        material=wood,
        name="right_wall",
    )
    shell.visual(
        Box((body_depth - 2.0 * wall_thickness, wall_thickness, body_height - bottom_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -body_width / 2.0 + wall_thickness / 2.0,
                bottom_thickness + (body_height - bottom_thickness) / 2.0,
            )
        ),
        material=wood,
        name="left_wall",
    )

    band = model.part("hinge_band")
    band.visual(
        Box((band_thickness, band_width, band_height)),
        material=wood_dark,
        name="band_strip",
    )
    band.visual(
        Box((band_thickness * 0.55, band_width * 0.92, 0.004)),
        origin=Origin(xyz=(band_thickness * 0.225, 0.0, band_height / 2.0 + 0.002)),
        material=brass,
        name="hinge_cap",
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_depth, lid_width, lid_thickness)),
        origin=Origin(
            xyz=(
                lid_depth / 2.0,
                0.0,
                lid_clearance + lid_thickness / 2.0,
            )
        ),
        material=wood,
        name="lid_panel",
    )
    lid.visual(
        Box((0.010, lid_width * 0.94, 0.003)),
        origin=Origin(
            xyz=(
                0.004,
                0.0,
                lid_clearance + lid_thickness + 0.0015,
            )
        ),
        material=brass,
        name="lid_hinge_leaf",
    )

    band_center = (
        -body_depth / 2.0 - band_thickness / 2.0,
        0.0,
        body_height - band_height / 2.0,
    )
    model.articulation(
        "shell_to_hinge_band",
        ArticulationType.FIXED,
        parent=shell,
        child=band,
        origin=Origin(xyz=band_center),
    )

    model.articulation(
        "hinge_band_to_lid",
        ArticulationType.REVOLUTE,
        parent=band,
        child=lid,
        origin=Origin(xyz=(band_thickness / 2.0, 0.0, band_height / 2.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("main_shell")
    band = object_model.get_part("hinge_band")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("hinge_band_to_lid")

    ctx.expect_gap(
        shell,
        band,
        axis="x",
        min_gap=0.0,
        max_gap=0.0,
        name="hinge band seats flush on the shell rear face",
    )
    ctx.expect_overlap(
        band,
        shell,
        axes="yz",
        min_overlap=0.03,
        name="hinge band spans the upper rear shell area",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            shell,
            axis="z",
            positive_elem="lid_panel",
            min_gap=0.0,
            max_gap=0.0015,
            name="closed lid panel sits just above the shell rim",
        )
        ctx.expect_overlap(
            lid,
            shell,
            axes="xy",
            elem_a="lid_panel",
            min_overlap=0.21,
            name="closed lid covers the shell opening footprint",
        )

        closed_panel_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    with ctx.pose({hinge: 1.2}):
        open_panel_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "lid swings upward when opened",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.18,
        details=f"closed_panel_aabb={closed_panel_aabb}, open_panel_aabb={open_panel_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
