from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_box_between_y(
    part,
    *,
    x: float,
    y0: float,
    y1: float,
    z: float,
    size_x: float,
    size_z: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Box((size_x, abs(y1 - y0), size_z)),
        origin=Origin(xyz=(x, (y0 + y1) * 0.5, z)),
        material=material,
        name=name,
    )


def _hook_mesh(name: str):
    """A stowed roof hook rod, authored in the hook hinge frame."""
    hook_geom = tube_from_spline_points(
        [
            (0.0, -0.075, 0.0),
            (0.0, -0.075, -0.070),
            (0.0, -0.075, -0.16),
            (0.0, -0.105, -0.33),
            (0.0, -0.165, -0.50),
            (0.0, -0.285, -0.49),
            (0.0, -0.325, -0.37),
            (0.0, -0.285, -0.25),
        ],
        radius=0.012,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(1.0, 0.0, 0.0),
        cap_ends=True,
    )
    return mesh_from_geometry(hook_geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_access_extension_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.74, 0.76, 0.75, 1.0))
    darker_aluminum = model.material("darker_aluminum", rgba=(0.56, 0.58, 0.57, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber = model.material("black_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    warning_red = model.material("warning_red", rgba=(0.78, 0.09, 0.06, 1.0))
    wall_galv = model.material("galvanized_wall_mount", rgba=(0.48, 0.50, 0.50, 1.0))

    fixed_section = model.part("fixed_section")

    # Two wall-mounted side rails.
    for index, x in enumerate((-0.285, 0.285)):
        fixed_section.visual(
            Box((0.055, 0.045, 4.20)),
            origin=Origin(xyz=(x, 0.0, 2.10)),
            material=aluminum,
            name=f"fixed_rail_{index}",
        )
        fixed_section.visual(
            Box((0.065, 0.060, 0.080)),
            origin=Origin(xyz=(x, -0.004, 0.045)),
            material=rubber,
            name=f"fixed_foot_{index}",
        )

    for i, z in enumerate([0.36 + 0.34 * n for n in range(12)]):
        _add_member(
            fixed_section,
            (-0.290, -0.012, z),
            (0.290, -0.012, z),
            0.018,
            darker_aluminum,
            name=f"fixed_rung_{i}",
        )

    fixed_section.visual(
        Box((0.68, 0.055, 0.040)),
        origin=Origin(xyz=(0.0, -0.004, 4.13)),
        material=darker_aluminum,
        name="fixed_top_tie",
    )
    fixed_section.visual(
        Box((0.68, 0.050, 0.035)),
        origin=Origin(xyz=(0.0, -0.004, 0.20)),
        material=darker_aluminum,
        name="fixed_bottom_tie",
    )

    # Wall stand-off brackets make the root section read as fixed to a building.
    for level, z in enumerate((0.62, 2.10, 3.64)):
        fixed_section.visual(
            Box((0.78, 0.035, 0.075)),
            origin=Origin(xyz=(0.0, 0.245, z)),
            material=wall_galv,
            name=f"wall_plate_{level}",
        )
        for side, x in enumerate((-0.285, 0.285)):
            _add_box_between_y(
                fixed_section,
                x=x,
                y0=0.020,
                y1=0.245,
                z=z,
                size_x=0.040,
                size_z=0.040,
                material=wall_galv,
                name=f"standoff_{level}_{side}",
            )
            fixed_section.visual(
                Cylinder(radius=0.018, length=0.030),
                origin=Origin(xyz=(x, 0.275, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=dark_hardware,
                name=f"wall_bolt_{level}_{side}",
            )

    # Small stop pads at the sleeve area for the fly ladder to bear against.
    for side, x in enumerate((-0.225, 0.225)):
        fixed_section.visual(
            Box((0.055, 0.034, 0.180)),
            origin=Origin(xyz=(x, -0.045, 1.10)),
            material=dark_hardware,
            name=f"slide_stop_{side}",
        )

    fly_section = model.part("fly_section")

    for index, x in enumerate((-0.225, 0.225)):
        fly_section.visual(
            Box((0.050, 0.040, 4.18)),
            origin=Origin(xyz=(x, 0.0, 2.09)),
            material=aluminum,
            name=f"fly_rail_{index}",
        )
        fly_section.visual(
            Box((0.065, 0.050, 0.065)),
            origin=Origin(xyz=(x, 0.0, 0.035)),
            material=rubber,
            name=f"fly_shoe_{index}",
        )

    for i, z in enumerate([0.30 + 0.33 * n for n in range(12)]):
        _add_member(
            fly_section,
            (-0.230, -0.018, z),
            (0.230, -0.018, z),
            0.016,
            darker_aluminum,
            name=f"fly_rung_{i}",
        )

    fly_section.visual(
        Box((0.56, 0.050, 0.035)),
        origin=Origin(xyz=(0.0, -0.004, 4.095)),
        material=darker_aluminum,
        name="fly_top_tie",
    )

    # Guide shoes and rollers show the upper section captured on the fixed rails
    # without intersecting them.
    for level, z in enumerate((0.48, 2.05)):
        for side, x in enumerate((-0.225, 0.225)):
            fly_section.visual(
                Box((0.065, 0.155, 0.055)),
                origin=Origin(xyz=(x, 0.030, z)),
                material=dark_hardware,
                name=f"guide_bridge_{level}_{side}",
            )
            fly_section.visual(
                Cylinder(radius=0.026, length=0.040),
                origin=Origin(xyz=(x, 0.120, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=rubber,
                name=f"guide_roller_{level}_{side}",
            )

    # Spring rung locks are fixed details on the fly section.
    for side, x in enumerate((-0.180, 0.180)):
        fly_section.visual(
            Box((0.110, 0.028, 0.060)),
            origin=Origin(xyz=(x, -0.034, 0.89)),
            material=warning_red,
            name=f"rung_lock_{side}",
        )

    # Hinge pedestals for the two folding roof hooks.
    fly_section.visual(
        Box((0.105, 0.058, 0.056)),
        origin=Origin(xyz=(-0.165, 0.0, 4.128)),
        material=dark_hardware,
        name="hook_pedestal_0",
    )
    fly_section.visual(
        Box((0.105, 0.058, 0.056)),
        origin=Origin(xyz=(0.165, 0.0, 4.128)),
        material=dark_hardware,
        name="hook_pedestal_1",
    )

    hook_meshes = (_hook_mesh("roof_hook_0_rod"), _hook_mesh("roof_hook_1_rod"))
    hook_parts = []
    for index in range(2):
        hook = model.part(f"hook_{index}")
        hook.visual(
            Cylinder(radius=0.024, length=0.078),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_hardware,
            name="hinge_barrel",
        )
        hook.visual(
            hook_meshes[index],
            material=warning_red,
            name="curved_hook",
        )
        _add_member(
            hook,
            (0.0, -0.024, 0.0),
            (0.0, -0.075, 0.0),
            0.010,
            dark_hardware,
            name="hook_lug",
        )
        hook.visual(
            Cylinder(radius=0.038, length=0.085),
            origin=Origin(xyz=(0.0, -0.285, -0.250), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="ridge_roller",
        )
        hook.visual(
            Sphere(radius=0.018),
            origin=Origin(xyz=(0.0, -0.325, -0.370)),
            material=rubber,
            name="hook_tip",
        )
        hook_parts.append(hook)

    model.articulation(
        "fly_slide",
        ArticulationType.PRISMATIC,
        parent=fixed_section,
        child=fly_section,
        origin=Origin(xyz=(0.0, -0.095, 0.75)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.30, lower=0.0, upper=1.65),
    )

    for index, x in enumerate((-0.165, 0.165)):
        model.articulation(
            f"hook_{index}_hinge",
            ArticulationType.REVOLUTE,
            parent=fly_section,
            child=hook_parts[index],
            origin=Origin(xyz=(x, 0.0, 4.180)),
            # Positive rotation folds the stowed hook over toward the wall/roof side.
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.5,
                lower=0.0,
                upper=math.radians(105.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed = object_model.get_part("fixed_section")
    fly = object_model.get_part("fly_section")
    hook_0 = object_model.get_part("hook_0")
    hook_1 = object_model.get_part("hook_1")
    slide = object_model.get_articulation("fly_slide")
    hook_0_hinge = object_model.get_articulation("hook_0_hinge")
    hook_1_hinge = object_model.get_articulation("hook_1_hinge")

    ctx.expect_origin_gap(
        fixed,
        fly,
        axis="y",
        min_gap=0.070,
        max_gap=0.120,
        name="fly section rides in front of the fixed wall section",
    )
    ctx.expect_overlap(
        fly,
        fixed,
        axes="z",
        min_overlap=3.0,
        name="collapsed fly section has long retained overlap",
    )
    ctx.expect_within(
        fly,
        fixed,
        axes="x",
        margin=0.09,
        name="fly section stays within fixed section width",
    )
    ctx.expect_contact(
        hook_0,
        fly,
        elem_a="hinge_barrel",
        elem_b="hook_pedestal_0",
        contact_tol=0.002,
        name="first hook barrel sits on its hinge pedestal",
    )
    ctx.expect_contact(
        hook_1,
        fly,
        elem_a="hinge_barrel",
        elem_b="hook_pedestal_1",
        contact_tol=0.002,
        name="second hook barrel sits on its hinge pedestal",
    )

    rest_pos = ctx.part_world_position(fly)
    with ctx.pose({slide: 1.65}):
        ctx.expect_overlap(
            fly,
            fixed,
            axes="z",
            min_overlap=1.2,
            name="extended fly section remains engaged in the fixed rails",
        )
        extended_pos = ctx.part_world_position(fly)

    ctx.check(
        "fly slide extends upward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 1.55,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    stowed_aabb = ctx.part_world_aabb(hook_0)
    with ctx.pose({hook_0_hinge: math.radians(105.0), hook_1_hinge: math.radians(105.0)}):
        deployed_aabb = ctx.part_world_aabb(hook_0)
        ctx.expect_overlap(
            hook_0,
            hook_1,
            axes="z",
            min_overlap=0.10,
            name="paired roof hooks deploy together at matching height",
        )

    ctx.check(
        "roof hooks swing toward the ridge side",
        stowed_aabb is not None
        and deployed_aabb is not None
        and deployed_aabb[1][1] > stowed_aabb[1][1] + 0.14,
        details=f"stowed={stowed_aabb}, deployed={deployed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
