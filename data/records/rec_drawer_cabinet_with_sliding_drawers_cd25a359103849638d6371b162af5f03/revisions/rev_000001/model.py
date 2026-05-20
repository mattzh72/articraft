from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="childrens_three_drawer_dresser")

    birch = model.material("birch", rgba=(0.82, 0.72, 0.58, 1.0))
    sky = model.material("sky_paint", rgba=(0.72, 0.84, 0.94, 1.0))
    mint = model.material("mint_paint", rgba=(0.73, 0.88, 0.78, 1.0))
    peach = model.material("peach_paint", rgba=(0.95, 0.78, 0.67, 1.0))
    cream = model.material("cream_knob", rgba=(0.96, 0.95, 0.91, 1.0))

    width = 0.82
    depth = 0.42
    height = 0.78
    side_t = 0.018
    top_t = 0.022
    bottom_t = 0.018
    divider_t = 0.014
    back_t = 0.008
    plinth_h = 0.040

    inner_width = width - 2.0 * side_t
    front_y = -depth / 2.0
    inner_bottom = plinth_h + bottom_t
    inner_top = height - top_t
    opening_height = (inner_top - inner_bottom - 2.0 * divider_t) / 3.0
    opening_centers = (
        inner_bottom + opening_height / 2.0,
        inner_bottom + opening_height + divider_t + opening_height / 2.0,
        inner_bottom + 2.0 * (opening_height + divider_t) + opening_height / 2.0,
    )

    drawer_width = inner_width - 0.008
    drawer_front_t = 0.018
    drawer_front_h = opening_height - 0.010
    drawer_depth = 0.340
    drawer_side_t = 0.012
    drawer_back_t = 0.012
    drawer_side_h = 0.160
    drawer_side_bottom = -0.088
    drawer_side_center_z = drawer_side_bottom + drawer_side_h / 2.0
    drawer_side_len = drawer_depth - drawer_front_t
    drawer_bottom_t = 0.008
    drawer_bottom_width = drawer_width - 2.0 * drawer_side_t
    drawer_bottom_len = drawer_depth - drawer_front_t - 0.008
    drawer_bottom_center_y = drawer_front_t + drawer_bottom_len / 2.0
    drawer_bottom_center_z = drawer_side_bottom + 0.008
    runner_w = 0.016
    runner_h = 0.010
    runner_len = 0.275
    runner_center_y = 0.040 + runner_len / 2.0
    runner_center_z = drawer_side_bottom - runner_h / 2.0
    runner_center_x = drawer_width / 2.0 - runner_w / 2.0

    rail_w = 0.016
    rail_h = 0.012
    rail_len = 0.320
    rail_center_y = -0.030
    rail_center_x = inner_width / 2.0 - rail_w / 2.0
    rail_center_z_offset = runner_center_z - runner_h / 2.0 - rail_h / 2.0

    knob_stem_r = 0.007
    knob_stem_l = 0.016
    knob_r = 0.013
    knob_x = drawer_width * 0.22

    carcass = model.part("carcass")
    carcass.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + side_t / 2.0, 0.0, height / 2.0)),
        material=birch,
        name="left_side",
    )
    carcass.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=(width / 2.0 - side_t / 2.0, 0.0, height / 2.0)),
        material=birch,
        name="right_side",
    )
    carcass.visual(
        Box((inner_width, depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, height - top_t / 2.0)),
        material=birch,
        name="top_panel",
    )
    carcass.visual(
        Box((inner_width, depth, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h + bottom_t / 2.0)),
        material=birch,
        name="bottom_panel",
    )
    carcass.visual(
        Box((inner_width, depth - 0.050, plinth_h)),
        origin=Origin(xyz=(0.0, 0.010, plinth_h / 2.0)),
        material=birch,
        name="plinth",
    )
    carcass.visual(
        Box((inner_width, back_t, height - 0.020)),
        origin=Origin(xyz=(0.0, depth / 2.0 - back_t / 2.0, (height - 0.020) / 2.0 + 0.010)),
        material=birch,
        name="back_panel",
    )

    for divider_idx, divider_center_z in enumerate(
        (
            inner_bottom + opening_height + divider_t / 2.0,
            inner_bottom + 2.0 * opening_height + 1.5 * divider_t,
        ),
        start=1,
    ):
        carcass.visual(
            Box((inner_width, depth, divider_t)),
            origin=Origin(xyz=(0.0, 0.0, divider_center_z)),
            material=birch,
            name=f"divider_{divider_idx}",
        )

    drawer_specs = (
        ("bottom_drawer", opening_centers[0], sky, 1),
        ("middle_drawer", opening_centers[1], mint, 2),
        ("top_drawer", opening_centers[2], peach, 3),
    )

    for drawer_name, drawer_center_z, drawer_front_color, drawer_idx in drawer_specs:
        carcass.visual(
            Box((rail_w, rail_len, rail_h)),
            origin=Origin(
                xyz=(-rail_center_x, rail_center_y, drawer_center_z + rail_center_z_offset)
            ),
            material=birch,
            name=f"guide_rail_left_{drawer_idx}",
        )
        carcass.visual(
            Box((rail_w, rail_len, rail_h)),
            origin=Origin(
                xyz=(rail_center_x, rail_center_y, drawer_center_z + rail_center_z_offset)
            ),
            material=birch,
            name=f"guide_rail_right_{drawer_idx}",
        )

        drawer = model.part(drawer_name)
        drawer.visual(
            Box((drawer_width, drawer_front_t, drawer_front_h)),
            origin=Origin(xyz=(0.0, drawer_front_t / 2.0, 0.0)),
            material=drawer_front_color,
            name="drawer_front",
        )
        drawer.visual(
            Box((drawer_side_t, drawer_side_len, drawer_side_h)),
            origin=Origin(
                xyz=(
                    -drawer_width / 2.0 + drawer_side_t / 2.0,
                    drawer_front_t + drawer_side_len / 2.0,
                    drawer_side_center_z,
                )
            ),
            material=birch,
            name="left_side",
        )
        drawer.visual(
            Box((drawer_side_t, drawer_side_len, drawer_side_h)),
            origin=Origin(
                xyz=(
                    drawer_width / 2.0 - drawer_side_t / 2.0,
                    drawer_front_t + drawer_side_len / 2.0,
                    drawer_side_center_z,
                )
            ),
            material=birch,
            name="right_side",
        )
        drawer.visual(
            Box((drawer_bottom_width, drawer_bottom_len, drawer_bottom_t)),
            origin=Origin(
                xyz=(0.0, drawer_bottom_center_y, drawer_bottom_center_z)
            ),
            material=birch,
            name="drawer_bottom",
        )
        drawer.visual(
            Box((drawer_bottom_width, drawer_back_t, drawer_side_h - 0.010)),
            origin=Origin(
                xyz=(
                    0.0,
                    drawer_depth - drawer_back_t / 2.0,
                    drawer_side_bottom + (drawer_side_h - 0.010) / 2.0,
                )
            ),
            material=birch,
            name="drawer_back",
        )
        drawer.visual(
            Box((runner_w, runner_len, runner_h)),
            origin=Origin(
                xyz=(-runner_center_x, runner_center_y, runner_center_z)
            ),
            material=birch,
            name="runner_left",
        )
        drawer.visual(
            Box((runner_w, runner_len, runner_h)),
            origin=Origin(
                xyz=(runner_center_x, runner_center_y, runner_center_z)
            ),
            material=birch,
            name="runner_right",
        )

        for knob_side, knob_sign in (("left", -1.0), ("right", 1.0)):
            drawer.visual(
                Cylinder(radius=knob_stem_r, length=knob_stem_l),
                origin=Origin(
                    xyz=(knob_sign * knob_x, -knob_stem_l / 2.0, 0.0),
                    rpy=(pi / 2.0, 0.0, 0.0),
                ),
                material=cream,
                name=f"knob_stem_{knob_side}",
            )
            drawer.visual(
                Sphere(radius=knob_r),
                origin=Origin(
                    xyz=(knob_sign * knob_x, -knob_stem_l - knob_r * 0.80, 0.0)
                ),
                material=cream,
                name=f"knob_{knob_side}",
            )

        model.articulation(
            f"carcass_to_{drawer_name}",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(0.0, front_y, drawer_center_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=60.0,
                velocity=0.35,
                lower=0.0,
                upper=0.220,
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
    carcass = object_model.get_part("carcass")
    drawer_names = ("bottom_drawer", "middle_drawer", "top_drawer")
    joint_names = tuple(f"carcass_to_{name}" for name in drawer_names)

    ctx.check(
        "dresser has three drawers",
        len(drawer_names) == 3 and len(joint_names) == 3,
        details=f"drawers={drawer_names}, joints={joint_names}",
    )

    for drawer_idx, drawer_name in enumerate(drawer_names, start=1):
        drawer = object_model.get_part(drawer_name)
        slide = object_model.get_articulation(f"carcass_to_{drawer_name}")
        upper = slide.motion_limits.upper if slide.motion_limits is not None else None

        with ctx.pose({slide: 0.0}):
            carcass_aabb = ctx.part_world_aabb(carcass)
            front_aabb = ctx.part_element_world_aabb(drawer, elem="drawer_front")
            flush_ok = (
                carcass_aabb is not None
                and front_aabb is not None
                and abs(front_aabb[0][1] - carcass_aabb[0][1]) <= 0.001
            )
            ctx.check(
                f"{drawer_name} front sits flush when closed",
                flush_ok,
                details=f"carcass_aabb={carcass_aabb}, drawer_front_aabb={front_aabb}",
            )
            ctx.expect_contact(
                drawer,
                carcass,
                elem_a="runner_left",
                elem_b=f"guide_rail_left_{drawer_idx}",
                contact_tol=0.0005,
                name=f"{drawer_name} left runner rests on guide rail closed",
            )
            ctx.expect_contact(
                drawer,
                carcass,
                elem_a="runner_right",
                elem_b=f"guide_rail_right_{drawer_idx}",
                contact_tol=0.0005,
                name=f"{drawer_name} right runner rests on guide rail closed",
            )
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="y",
                elem_a="runner_left",
                elem_b=f"guide_rail_left_{drawer_idx}",
                min_overlap=0.26,
                name=f"{drawer_name} left runner stays deeply engaged when closed",
            )
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="y",
                elem_a="runner_right",
                elem_b=f"guide_rail_right_{drawer_idx}",
                min_overlap=0.26,
                name=f"{drawer_name} right runner stays deeply engaged when closed",
            )

        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({slide: upper}):
            ctx.expect_contact(
                drawer,
                carcass,
                elem_a="runner_left",
                elem_b=f"guide_rail_left_{drawer_idx}",
                contact_tol=0.0005,
                name=f"{drawer_name} left runner stays supported when open",
            )
            ctx.expect_contact(
                drawer,
                carcass,
                elem_a="runner_right",
                elem_b=f"guide_rail_right_{drawer_idx}",
                contact_tol=0.0005,
                name=f"{drawer_name} right runner stays supported when open",
            )
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="y",
                elem_a="runner_left",
                elem_b=f"guide_rail_left_{drawer_idx}",
                min_overlap=0.07,
                name=f"{drawer_name} left runner retains insertion when open",
            )
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="y",
                elem_a="runner_right",
                elem_b=f"guide_rail_right_{drawer_idx}",
                min_overlap=0.07,
                name=f"{drawer_name} right runner retains insertion when open",
            )
            open_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"{drawer_name} opens outward",
            rest_pos is not None
            and open_pos is not None
            and open_pos[1] < rest_pos[1] - 0.15,
            details=f"rest={rest_pos}, open={open_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
