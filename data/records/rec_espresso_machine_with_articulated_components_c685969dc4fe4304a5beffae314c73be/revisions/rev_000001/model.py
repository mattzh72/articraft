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
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _rounded_panel_mesh(
    width: float,
    depth: float,
    thickness: float,
    radius: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius),
            thickness,
            cap=True,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="capsule_espresso_machine")

    body_color = model.material("body_color", rgba=(0.18, 0.18, 0.20, 1.0))
    trim_color = model.material("trim_color", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))

    body = model.part("body")

    shell_sections = [
        (-0.160, 0.000, 0.292, 0.150),
        (-0.115, 0.000, 0.302, 0.168),
        (-0.040, 0.000, 0.304, 0.182),
        (0.040, 0.000, 0.292, 0.176),
        (0.105, 0.000, 0.252, 0.142),
        (0.148, 0.044, 0.214, 0.102),
    ]
    shell_mesh = mesh_from_geometry(
        superellipse_side_loft(shell_sections, exponents=3.0, segments=64),
        "body_shell",
    )
    body.visual(shell_mesh, material=body_color, name="shell")

    body.visual(
        Box((0.102, 0.094, 0.008)),
        origin=Origin(xyz=(0.0, 0.012, 0.301)),
        material=trim_color,
        name="brew_seat",
    )
    body.visual(
        Box((0.086, 0.082, 0.020)),
        origin=Origin(xyz=(0.0, -0.096, 0.302)),
        material=trim_color,
        name="tank_seat",
    )
    body.visual(
        Box((0.110, 0.026, 0.074)),
        origin=Origin(xyz=(0.0, 0.118, 0.121)),
        material=trim_color,
        name="shelf_mount",
    )
    body.visual(
        Box((0.010, 0.020, 0.058)),
        origin=Origin(xyz=(0.091, 0.048, 0.190)),
        material=trim_color,
        name="wand_mount",
    )
    body.visual(
        Box((0.046, 0.034, 0.036)),
        origin=Origin(xyz=(0.0, 0.130, 0.176)),
        material=trim_color,
        name="brew_head",
    )
    for x in (-0.010, 0.010):
        body.visual(
            Cylinder(radius=0.0035, length=0.012),
            origin=Origin(xyz=(x, 0.151, 0.156), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"spout_{0 if x < 0 else 1}",
        )

    brew_lid = model.part("brew_lid")
    brew_lid.visual(
        _rounded_panel_mesh(0.094, 0.104, 0.010, 0.010, "brew_lid_panel"),
        origin=Origin(xyz=(0.0, 0.052, 0.005)),
        material=body_color,
        name="panel",
    )
    brew_lid.visual(
        Box((0.054, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.084, 0.011)),
        material=steel,
        name="handle",
    )

    tank_lid = model.part("tank_lid")
    tank_lid.visual(
        _rounded_panel_mesh(0.084, 0.084, 0.010, 0.009, "tank_lid_panel"),
        origin=Origin(xyz=(0.0, 0.042, 0.005)),
        material=body_color,
        name="panel",
    )
    tank_lid.visual(
        Box((0.038, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.075, 0.009)),
        material=steel,
        name="tab",
    )

    cup_shelf = model.part("cup_shelf")
    cup_shelf.visual(
        Box((0.092, 0.006, 0.062)),
        origin=Origin(xyz=(0.0, 0.003, -0.031)),
        material=steel,
        name="plate",
    )
    cup_shelf.visual(
        Box((0.092, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.008, -0.056)),
        material=trim_color,
        name="lip",
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=steel,
        name="collar",
    )
    steam_wand.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.007, 0.000, -0.010),
                    (0.014, 0.003, -0.034),
                    (0.020, 0.008, -0.078),
                    (0.018, 0.016, -0.122),
                ],
                radius=0.0042,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
            "steam_wand_tube",
        ),
        material=steel,
        name="tube",
    )

    model.articulation(
        "brew_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=brew_lid,
        origin=Origin(xyz=(0.0, -0.035, 0.305)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "tank_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tank_lid,
        origin=Origin(xyz=(0.0, -0.137, 0.312)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.20),
    )
    model.articulation(
        "shelf_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cup_shelf,
        origin=Origin(xyz=(0.0, 0.147, 0.106)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=1.42),
    )
    model.articulation(
        "wand_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.105, 0.048, 0.196)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-1.10, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    brew_lid = object_model.get_part("brew_lid")
    tank_lid = object_model.get_part("tank_lid")
    cup_shelf = object_model.get_part("cup_shelf")
    steam_wand = object_model.get_part("steam_wand")

    brew_hinge = object_model.get_articulation("brew_hinge")
    tank_hinge = object_model.get_articulation("tank_hinge")
    shelf_hinge = object_model.get_articulation("shelf_hinge")
    wand_pivot = object_model.get_articulation("wand_pivot")

    def _span(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    ctx.expect_gap(
        brew_lid,
        body,
        axis="z",
        positive_elem="panel",
        negative_elem="brew_seat",
        max_gap=0.0005,
        max_penetration=0.0,
        name="brew lid seats on brew deck",
    )
    ctx.expect_overlap(
        brew_lid,
        body,
        axes="xy",
        elem_a="panel",
        elem_b="brew_seat",
        min_overlap=0.080,
        name="brew lid covers the capsule opening",
    )

    ctx.expect_gap(
        tank_lid,
        body,
        axis="z",
        positive_elem="panel",
        negative_elem="tank_seat",
        max_gap=0.0005,
        max_penetration=0.0,
        name="water tank lid seats on the rear tank rim",
    )
    ctx.expect_overlap(
        tank_lid,
        body,
        axes="xy",
        elem_a="panel",
        elem_b="tank_seat",
        min_overlap=0.080,
        name="water tank lid covers the tank opening",
    )

    ctx.expect_gap(
        steam_wand,
        body,
        axis="x",
        positive_elem="collar",
        negative_elem="wand_mount",
        max_gap=0.0005,
        max_penetration=0.0,
        name="steam wand collar meets the side pivot mount",
    )

    brew_closed = ctx.part_element_world_aabb(brew_lid, elem="panel")
    with ctx.pose({brew_hinge: 1.20}):
        brew_open = ctx.part_element_world_aabb(brew_lid, elem="panel")
    ctx.check(
        "brew lid flips upward",
        brew_closed is not None
        and brew_open is not None
        and brew_open[1][2] > brew_closed[1][2] + 0.070,
        details=f"closed={brew_closed}, open={brew_open}",
    )

    tank_closed = ctx.part_element_world_aabb(tank_lid, elem="panel")
    with ctx.pose({tank_hinge: 1.00}):
        tank_open = ctx.part_element_world_aabb(tank_lid, elem="panel")
    ctx.check(
        "rear tank lid flips upward",
        tank_closed is not None
        and tank_open is not None
        and tank_open[1][2] > tank_closed[1][2] + 0.050,
        details=f"closed={tank_closed}, open={tank_open}",
    )

    shelf_closed = ctx.part_element_world_aabb(cup_shelf, elem="plate")
    with ctx.pose({shelf_hinge: 1.35}):
        shelf_open = ctx.part_element_world_aabb(cup_shelf, elem="plate")
    ctx.check(
        "cup shelf folds down into a horizontal perch",
        shelf_closed is not None
        and shelf_open is not None
        and shelf_open[1][1] > shelf_closed[1][1] + 0.045
        and (_span(shelf_open, 2) or 0.0) < 0.025,
        details=f"closed={shelf_closed}, open={shelf_open}",
    )

    wand_closed = ctx.part_element_world_aabb(steam_wand, elem="tube")
    with ctx.pose({wand_pivot: 0.80}):
        wand_open = ctx.part_element_world_aabb(steam_wand, elem="tube")
    ctx.check(
        "steam wand swings forward on its vertical pivot",
        wand_closed is not None
        and wand_open is not None
        and wand_open[1][1] > wand_closed[1][1] + 0.008
        and wand_open[0][0] < wand_closed[0][0] - 0.010,
        details=f"closed={wand_closed}, open={wand_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
