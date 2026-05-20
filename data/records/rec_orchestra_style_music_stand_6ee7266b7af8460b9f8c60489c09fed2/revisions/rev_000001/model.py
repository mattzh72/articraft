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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conductor_music_stand")

    black_steel = model.material("black_steel", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.63, 0.65, 0.68, 1.0))

    def tube_shell(
        *,
        outer_radius: float,
        inner_radius: float,
        z0: float,
        z1: float,
        name: str,
        segments: int = 56,
    ):
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(outer_radius, z0), (outer_radius, z1)],
                [(inner_radius, z0), (inner_radius, z1)],
                segments=segments,
                start_cap="flat",
                end_cap="flat",
            ),
            name,
        )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.19, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=graphite,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.055, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=graphite,
        name="base_collar",
    )
    pedestal.visual(
        tube_shell(
            outer_radius=0.031,
            inner_radius=0.0255,
            z0=0.028,
            z1=0.668,
            name="sleeve_shell_mesh",
        ),
        material=black_steel,
        name="sleeve_shell",
    )
    pedestal.visual(
        tube_shell(
            outer_radius=0.038,
            inner_radius=0.026,
            z0=0.583,
            z1=0.668,
            name="top_collar_mesh",
        ),
        material=graphite,
        name="top_collar",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=0.67),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, 0.335)),
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.0205, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=satin_metal,
        name="mast_tube",
    )
    mast.visual(
        Cylinder(radius=0.035, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=graphite,
        name="height_collar",
    )
    mast.visual(
        Box((0.072, 0.028, 0.04)),
        origin=Origin(xyz=(0.0, -0.029, 0.37)),
        material=black_steel,
        name="tilt_head",
    )
    mast.inertial = Inertial.from_geometry(
        Cylinder(radius=0.023, length=0.78),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
    )

    desk = model.part("desk")
    desk.visual(
        Box((0.76, 0.014, 0.46)),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=black_steel,
        name="back_panel",
    )
    desk.visual(
        Box((0.76, 0.075, 0.018)),
        origin=Origin(xyz=(0.0, 0.03, 0.021)),
        material=black_steel,
        name="bottom_shelf",
    )
    desk.visual(
        Box((0.76, 0.010, 0.038)),
        origin=Origin(xyz=(0.0, 0.068, 0.028)),
        material=black_steel,
        name="music_lip",
    )
    desk.visual(
        Box((0.018, 0.050, 0.40)),
        origin=Origin(xyz=(-0.371, 0.015, 0.235)),
        material=black_steel,
        name="left_side_flange",
    )
    desk.visual(
        Box((0.018, 0.050, 0.40)),
        origin=Origin(xyz=(0.371, 0.015, 0.235)),
        material=black_steel,
        name="right_side_flange",
    )
    desk.visual(
        Box((0.72, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, 0.007, 0.470)),
        material=black_steel,
        name="top_flange",
    )
    desk.visual(
        Box((0.026, 0.024, 0.040)),
        origin=Origin(xyz=(-0.049, -0.029, -0.020)),
        material=graphite,
        name="rear_left_lug",
    )
    desk.visual(
        Box((0.026, 0.024, 0.040)),
        origin=Origin(xyz=(0.049, -0.029, -0.020)),
        material=graphite,
        name="rear_right_lug",
    )
    desk.visual(
        Box((0.020, 0.028, 0.062)),
        origin=Origin(xyz=(-0.058, -0.015, 0.011)),
        material=graphite,
        name="rear_left_brace",
    )
    desk.visual(
        Box((0.020, 0.028, 0.062)),
        origin=Origin(xyz=(0.058, -0.015, 0.011)),
        material=graphite,
        name="rear_right_brace",
    )
    desk.visual(
        Box((0.022, 0.012, 0.018)),
        origin=Origin(xyz=(-0.388, 0.034, 0.125)),
        material=graphite,
        name="left_pivot_block",
    )
    desk.visual(
        Box((0.022, 0.012, 0.018)),
        origin=Origin(xyz=(0.388, 0.034, 0.125)),
        material=graphite,
        name="right_pivot_block",
    )
    desk.inertial = Inertial.from_geometry(
        Box((0.78, 0.09, 0.50)),
        mass=3.3,
        origin=Origin(xyz=(0.0, 0.018, 0.24)),
    )

    left_support = model.part("left_page_support")
    left_support.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="pivot_barrel",
    )
    left_support.visual(
        Cylinder(radius=0.0034, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=black_steel,
        name="support_rod",
    )
    left_support.visual(
        Cylinder(radius=0.0034, length=0.030),
        origin=Origin(xyz=(0.013, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_steel,
        name="lower_hook",
    )
    left_support.visual(
        Cylinder(radius=0.0034, length=0.045),
        origin=Origin(xyz=(0.018, 0.0, 0.220), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_steel,
        name="top_hook",
    )
    left_support.inertial = Inertial.from_geometry(
        Box((0.05, 0.012, 0.24)),
        mass=0.06,
        origin=Origin(xyz=(0.01, 0.0, 0.11)),
    )

    right_support = model.part("right_page_support")
    right_support.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="pivot_barrel",
    )
    right_support.visual(
        Cylinder(radius=0.0034, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=black_steel,
        name="support_rod",
    )
    right_support.visual(
        Cylinder(radius=0.0034, length=0.030),
        origin=Origin(xyz=(-0.013, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_steel,
        name="lower_hook",
    )
    right_support.visual(
        Cylinder(radius=0.0034, length=0.045),
        origin=Origin(xyz=(-0.018, 0.0, 0.220), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_steel,
        name="top_hook",
    )
    right_support.inertial = Inertial.from_geometry(
        Box((0.05, 0.012, 0.24)),
        mass=0.06,
        origin=Origin(xyz=(-0.01, 0.0, 0.11)),
    )

    mast_slide = model.articulation(
        "pedestal_to_mast",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.668)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=0.0,
            upper=0.24,
        ),
    )

    desk_tilt = model.articulation(
        "mast_to_desk",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-0.40,
            upper=0.55,
        ),
    )

    left_pivot = model.articulation(
        "desk_to_left_page_support",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=left_support,
        origin=Origin(xyz=(-0.389, 0.046, 0.125)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=0.95,
        ),
    )

    right_pivot = model.articulation(
        "desk_to_right_page_support",
        ArticulationType.REVOLUTE,
        parent=desk,
        child=right_support,
        origin=Origin(xyz=(0.389, 0.046, 0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=0.95,
        ),
    )

    pedestal.meta["primary_slide"] = mast_slide.name
    desk.meta["primary_tilt"] = desk_tilt.name
    desk.meta["page_supports"] = [left_pivot.name, right_pivot.name]
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

    pedestal = object_model.get_part("pedestal")
    mast = object_model.get_part("mast")
    desk = object_model.get_part("desk")
    left_support = object_model.get_part("left_page_support")
    right_support = object_model.get_part("right_page_support")

    mast_slide = object_model.get_articulation("pedestal_to_mast")
    desk_tilt = object_model.get_articulation("mast_to_desk")
    left_pivot = object_model.get_articulation("desk_to_left_page_support")
    right_pivot = object_model.get_articulation("desk_to_right_page_support")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.check(
        "articulations use the intended axes",
        mast_slide.axis == (0.0, 0.0, 1.0)
        and desk_tilt.axis == (1.0, 0.0, 0.0)
        and left_pivot.axis == (0.0, 1.0, 0.0)
        and right_pivot.axis == (0.0, -1.0, 0.0),
        details=(
            f"mast={mast_slide.axis}, desk={desk_tilt.axis}, "
            f"left={left_pivot.axis}, right={right_pivot.axis}"
        ),
    )

    with ctx.pose({mast_slide: 0.0, desk_tilt: 0.0}):
        ctx.expect_within(
            mast,
            pedestal,
            axes="xy",
            inner_elem="mast_tube",
            outer_elem="sleeve_shell",
            margin=0.012,
            name="mast stays centered inside the sleeve at rest",
        )
        ctx.expect_overlap(
            mast,
            pedestal,
            axes="z",
            elem_a="mast_tube",
            elem_b="sleeve_shell",
            min_overlap=0.40,
            name="mast remains deeply inserted at rest",
        )
        ctx.expect_contact(
            desk,
            mast,
            elem_a="rear_left_lug",
            elem_b="tilt_head",
            contact_tol=0.001,
            name="left desk lug seats on the tilt head",
        )
        ctx.expect_contact(
            desk,
            mast,
            elem_a="rear_right_lug",
            elem_b="tilt_head",
            contact_tol=0.001,
            name="right desk lug seats on the tilt head",
        )
        ctx.expect_contact(
            left_support,
            desk,
            elem_a="pivot_barrel",
            elem_b="left_pivot_block",
            contact_tol=0.001,
            name="left page support is mounted to its side pivot",
        )
        ctx.expect_contact(
            right_support,
            desk,
            elem_a="pivot_barrel",
            elem_b="right_pivot_block",
            contact_tol=0.001,
            name="right page support is mounted to its side pivot",
        )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.24}):
        ctx.expect_within(
            mast,
            pedestal,
            axes="xy",
            inner_elem="mast_tube",
            outer_elem="sleeve_shell",
            margin=0.012,
            name="mast stays centered inside the sleeve when extended",
        )
        ctx.expect_overlap(
            mast,
            pedestal,
            axes="z",
            elem_a="mast_tube",
            elem_b="sleeve_shell",
            min_overlap=0.17,
            name="mast keeps retained insertion at max height",
        )
        extended_mast_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast extends upward",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.20,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    with ctx.pose({desk_tilt: 0.0}):
        top_rest = aabb_center(ctx.part_element_world_aabb(desk, elem="top_flange"))
    with ctx.pose({desk_tilt: 0.45}):
        top_tilted = aabb_center(ctx.part_element_world_aabb(desk, elem="top_flange"))
    ctx.check(
        "desk positive tilt leans the top backward",
        top_rest is not None and top_tilted is not None and top_tilted[1] < top_rest[1] - 0.10,
        details=f"rest={top_rest}, tilted={top_tilted}",
    )

    with ctx.pose({left_pivot: 0.0}):
        left_hook_rest = aabb_center(ctx.part_element_world_aabb(left_support, elem="top_hook"))
    with ctx.pose({left_pivot: 0.70}):
        left_hook_open = aabb_center(ctx.part_element_world_aabb(left_support, elem="top_hook"))
    ctx.check(
        "left page support swings inward over the desk",
        left_hook_rest is not None
        and left_hook_open is not None
        and left_hook_open[0] > left_hook_rest[0] + 0.10,
        details=f"rest={left_hook_rest}, open={left_hook_open}",
    )

    with ctx.pose({right_pivot: 0.0}):
        right_hook_rest = aabb_center(ctx.part_element_world_aabb(right_support, elem="top_hook"))
    with ctx.pose({right_pivot: 0.70}):
        right_hook_open = aabb_center(ctx.part_element_world_aabb(right_support, elem="top_hook"))
    ctx.check(
        "right page support swings inward over the desk",
        right_hook_rest is not None
        and right_hook_open is not None
        and right_hook_open[0] < right_hook_rest[0] - 0.10,
        details=f"rest={right_hook_rest}, open={right_hook_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
