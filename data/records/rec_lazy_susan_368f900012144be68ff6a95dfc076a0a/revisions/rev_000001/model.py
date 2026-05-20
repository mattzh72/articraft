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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    ExtrudeGeometry,
)


def kidney_profile() -> list[tuple[float, float]]:
    control_points = [
        (-0.060, 0.055),
        (-0.010, 0.120),
        (0.090, 0.195),
        (0.225, 0.192),
        (0.315, 0.125),
        (0.350, 0.020),
        (0.322, -0.095),
        (0.230, -0.175),
        (0.095, -0.195),
        (-0.015, -0.135),
        (-0.075, -0.040),
        (-0.050, 0.020),
    ]
    return sample_catmull_rom_spline_2d(
        control_points,
        samples_per_segment=10,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cabinet_insert_lazy_susan")

    cabinet_mat = model.material("cabinet_shell", color=(0.90, 0.88, 0.84))
    shelf_mat = model.material("shelf_panel", color=(0.96, 0.96, 0.93))
    metal_mat = model.material("chrome", color=(0.76, 0.79, 0.82))

    cabinet_width = 0.86
    cabinet_depth = 0.86
    cabinet_height = 0.72
    panel_thickness = 0.018
    shaft_xy = (0.39, 0.39)
    pole_radius = 0.015
    pole_length = cabinet_height - (2.0 * panel_thickness)
    shelf_thickness = 0.022

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, panel_thickness)),
        origin=Origin(xyz=(cabinet_width / 2.0, cabinet_depth / 2.0, panel_thickness / 2.0)),
        material=cabinet_mat,
        name="floor_panel",
    )
    cabinet.visual(
        Box((panel_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(panel_thickness / 2.0, cabinet_depth / 2.0, cabinet_height / 2.0)),
        material=cabinet_mat,
        name="left_wall",
    )
    cabinet.visual(
        Box((cabinet_width, panel_thickness, cabinet_height)),
        origin=Origin(xyz=(cabinet_width / 2.0, panel_thickness / 2.0, cabinet_height / 2.0)),
        material=cabinet_mat,
        name="back_wall",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, panel_thickness)),
        origin=Origin(xyz=(cabinet_width / 2.0, cabinet_depth / 2.0, cabinet_height - (panel_thickness / 2.0))),
        material=cabinet_mat,
        name="top_panel",
    )
    cabinet.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(shaft_xy[0], shaft_xy[1], panel_thickness / 2.0)),
        material=metal_mat,
        name="floor_bearing",
    )
    cabinet.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(shaft_xy[0], shaft_xy[1], cabinet_height - (panel_thickness / 2.0))),
        material=metal_mat,
        name="top_guide",
    )

    shelf_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(kidney_profile(), shelf_thickness, cap=True, closed=True),
        "kidney_shelf_panel",
    )

    carousel = model.part("carousel")
    carousel.visual(
        Cylinder(radius=pole_radius, length=pole_length),
        origin=Origin(xyz=(0.0, 0.0, pole_length / 2.0)),
        material=metal_mat,
        name="pole",
    )
    carousel.visual(
        Cylinder(radius=0.032, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=metal_mat,
        name="lower_retainer",
    )
    carousel.visual(
        shelf_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=shelf_mat,
        name="lower_shelf",
    )
    carousel.visual(
        Cylinder(radius=0.042, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=metal_mat,
        name="lower_hub",
    )
    carousel.visual(
        shelf_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=shelf_mat,
        name="upper_shelf",
    )
    carousel.visual(
        Cylinder(radius=0.042, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=metal_mat,
        name="upper_hub",
    )
    carousel.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, pole_length - 0.006)),
        material=metal_mat,
        name="top_retainer",
    )

    model.articulation(
        "cabinet_to_carousel",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=carousel,
        origin=Origin(xyz=(shaft_xy[0], shaft_xy[1], panel_thickness)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0),
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

    cabinet = object_model.get_part("cabinet")
    carousel = object_model.get_part("carousel")
    spin = object_model.get_articulation("cabinet_to_carousel")

    limits = spin.motion_limits
    ctx.check(
        "lazy Susan uses a continuous vertical spin joint",
        spin.joint_type == ArticulationType.CONTINUOUS
        and tuple(spin.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=f"type={spin.joint_type}, axis={spin.axis}, limits={limits}",
    )

    ctx.expect_contact(
        carousel,
        cabinet,
        elem_a="pole",
        elem_b="floor_panel",
        contact_tol=1e-5,
        name="pole seats on cabinet floor",
    )

    ctx.expect_gap(
        carousel,
        carousel,
        axis="z",
        positive_elem="upper_shelf",
        negative_elem="lower_shelf",
        min_gap=0.20,
        name="upper shelf is mounted well above lower shelf",
    )

    for angle, label in ((0.0, "closed"), (math.pi / 2.0, "quarter_turn")):
        with ctx.pose({spin: angle}):
            ctx.expect_gap(
                carousel,
                cabinet,
                axis="x",
                positive_elem="lower_shelf",
                negative_elem="left_wall",
                min_gap=0.025,
                name=f"lower shelf clears left wall at {label}",
            )
            ctx.expect_gap(
                carousel,
                cabinet,
                axis="y",
                positive_elem="lower_shelf",
                negative_elem="back_wall",
                min_gap=0.025,
                name=f"lower shelf clears back wall at {label}",
            )

    def aabb_center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3)) if aabb is not None else None

    rest_aabb = ctx.part_element_world_aabb(carousel, elem="lower_shelf")
    with ctx.pose({spin: math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(carousel, elem="lower_shelf")

    rest_center = aabb_center(rest_aabb)
    turned_center = aabb_center(turned_aabb)
    xy_shift = None
    z_shift = None
    if rest_center is not None and turned_center is not None:
        xy_shift = math.hypot(turned_center[0] - rest_center[0], turned_center[1] - rest_center[1])
        z_shift = abs(turned_center[2] - rest_center[2])

    ctx.check(
        "kidney shelves orbit around the common pole when rotated",
        xy_shift is not None and z_shift is not None and xy_shift > 0.06 and z_shift < 0.002,
        details=f"rest_center={rest_center}, turned_center={turned_center}, xy_shift={xy_shift}, z_shift={z_shift}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
