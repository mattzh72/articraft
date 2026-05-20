from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="museum_flat_file_plan_cabinet")

    # Overall proportions sized for a museum flat-file cabinet that holds
    # large-format prints and drawings.
    cabinet_w = 1.34
    cabinet_d = 0.94
    cabinet_h = 0.79

    side_t = 0.022
    top_t = 0.022
    bottom_t = 0.025
    back_t = 0.012
    plinth_h = 0.070
    plinth_t = 0.018
    plinth_inset = 0.050

    inner_w = cabinet_w - 2.0 * side_t
    inner_z0 = plinth_h + bottom_t
    inner_h = cabinet_h - plinth_h - bottom_t - top_t

    drawer_count = 6
    drawer_depth = 0.89
    drawer_width = inner_w - 0.014
    drawer_side_h = 0.072
    drawer_bottom_t = 0.008
    drawer_front_h = 0.100
    drawer_front_t = 0.018
    drawer_travel = 0.50

    slide_t = 0.006
    slide_top_z0 = inner_z0 + 0.010
    bay_pitch = 0.109
    running_gap = 0.0

    drawer_front_face_closed_y = cabinet_d / 2.0 - 0.001
    drawer_origin_y = drawer_front_face_closed_y - drawer_depth
    slide_depth = 0.84
    slide_y_min = drawer_origin_y - 0.003
    slide_y_center = slide_y_min + slide_depth / 2.0

    oak = model.material("oak_case", rgba=(0.48, 0.35, 0.22, 1.0))
    drawer_wood = model.material("drawer_wood", rgba=(0.63, 0.50, 0.34, 1.0))
    brass = model.material("brass_hardware", rgba=(0.73, 0.61, 0.33, 1.0))
    shadow = model.material("shadow_line", rgba=(0.18, 0.14, 0.10, 1.0))

    body = model.part("cabinet_body")
    case_h = cabinet_h - plinth_h

    body.visual(
        Box((side_t, cabinet_d, case_h)),
        origin=Origin(xyz=(-cabinet_w / 2.0 + side_t / 2.0, 0.0, plinth_h + case_h / 2.0)),
        material=oak,
        name="left_side",
    )
    body.visual(
        Box((side_t, cabinet_d, case_h)),
        origin=Origin(xyz=(cabinet_w / 2.0 - side_t / 2.0, 0.0, plinth_h + case_h / 2.0)),
        material=oak,
        name="right_side",
    )
    body.visual(
        Box((cabinet_w, cabinet_d, top_t)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_h - top_t / 2.0)),
        material=oak,
        name="top_panel",
    )
    body.visual(
        Box((cabinet_w, cabinet_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h + bottom_t / 2.0)),
        material=oak,
        name="bottom_panel",
    )
    body.visual(
        Box((inner_w, back_t, inner_h)),
        origin=Origin(
            xyz=(0.0, -cabinet_d / 2.0 + back_t / 2.0, inner_z0 + inner_h / 2.0)
        ),
        material=oak,
        name="back_panel",
    )

    plinth_w = cabinet_w - 2.0 * plinth_inset
    plinth_d = cabinet_d - 2.0 * plinth_inset
    body.visual(
        Box((plinth_w, plinth_t, plinth_h)),
        origin=Origin(
            xyz=(0.0, cabinet_d / 2.0 - plinth_inset - plinth_t / 2.0, plinth_h / 2.0)
        ),
        material=shadow,
        name="plinth_front",
    )
    body.visual(
        Box((plinth_w, plinth_t, plinth_h)),
        origin=Origin(
            xyz=(0.0, -cabinet_d / 2.0 + plinth_inset + plinth_t / 2.0, plinth_h / 2.0)
        ),
        material=shadow,
        name="plinth_back",
    )
    body.visual(
        Box((plinth_t, plinth_d, plinth_h)),
        origin=Origin(
            xyz=(-cabinet_w / 2.0 + plinth_inset + plinth_t / 2.0, 0.0, plinth_h / 2.0)
        ),
        material=shadow,
        name="plinth_left",
    )
    body.visual(
        Box((plinth_t, plinth_d, plinth_h)),
        origin=Origin(
            xyz=(cabinet_w / 2.0 - plinth_inset - plinth_t / 2.0, 0.0, plinth_h / 2.0)
        ),
        material=shadow,
        name="plinth_right",
    )

    for index in range(drawer_count):
        slide_top_z = slide_top_z0 + bay_pitch * index
        body.visual(
            Box((inner_w, slide_depth, slide_t)),
            origin=Origin(xyz=(0.0, slide_y_center, slide_top_z - slide_t / 2.0)),
            material=oak,
            name=f"slide_deck_{index + 1}",
        )

    for index in range(drawer_count):
        drawer = model.part(f"drawer_{index + 1}")

        drawer.visual(
            Box((drawer_width, drawer_depth, drawer_bottom_t)),
            origin=Origin(
                xyz=(0.0, drawer_depth / 2.0, drawer_bottom_t / 2.0),
            ),
            material=drawer_wood,
            name="drawer_bottom",
        )
        drawer.visual(
            Box((0.012, drawer_depth, drawer_side_h)),
            origin=Origin(
                xyz=(-drawer_width / 2.0 + 0.006, drawer_depth / 2.0, drawer_side_h / 2.0)
            ),
            material=drawer_wood,
            name="left_wall",
        )
        drawer.visual(
            Box((0.012, drawer_depth, drawer_side_h)),
            origin=Origin(
                xyz=(drawer_width / 2.0 - 0.006, drawer_depth / 2.0, drawer_side_h / 2.0)
            ),
            material=drawer_wood,
            name="right_wall",
        )
        drawer.visual(
            Box((drawer_width, 0.012, drawer_side_h)),
            origin=Origin(xyz=(0.0, 0.006, drawer_side_h / 2.0)),
            material=drawer_wood,
            name="back_wall",
        )
        drawer.visual(
            Box((drawer_width, drawer_front_t, drawer_front_h)),
            origin=Origin(
                xyz=(0.0, drawer_depth - drawer_front_t / 2.0, 0.002 + drawer_front_h / 2.0)
            ),
            material=oak,
            name="drawer_front",
        )
        drawer.visual(
            Box((0.140, 0.006, 0.032)),
            origin=Origin(xyz=(0.0, drawer_depth + 0.003, 0.060)),
            material=brass,
            name="label_holder",
        )
        drawer.visual(
            Cylinder(radius=0.004, length=0.018),
            origin=Origin(
                xyz=(-0.060, drawer_depth + 0.009, 0.035),
                rpy=(1.5707963267948966, 0.0, 0.0),
            ),
            material=brass,
            name="pull_post_left",
        )
        drawer.visual(
            Cylinder(radius=0.004, length=0.018),
            origin=Origin(
                xyz=(0.060, drawer_depth + 0.009, 0.035),
                rpy=(1.5707963267948966, 0.0, 0.0),
            ),
            material=brass,
            name="pull_post_right",
        )
        drawer.visual(
            Cylinder(radius=0.006, length=0.180),
            origin=Origin(
                xyz=(0.0, drawer_depth + 0.018, 0.035),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=brass,
            name="pull_bar",
        )

        model.articulation(
            f"cabinet_to_drawer_{index + 1}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(
                xyz=(0.0, drawer_origin_y, slide_top_z0 + bay_pitch * index + running_gap)
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=180.0,
                velocity=0.35,
                lower=0.0,
                upper=drawer_travel,
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

    body = object_model.get_part("cabinet_body")

    for index in range(1, 7):
        drawer = object_model.get_part(f"drawer_{index}")
        slide_joint = object_model.get_articulation(f"cabinet_to_drawer_{index}")
        slide_name = f"slide_deck_{index}"
        upper = slide_joint.motion_limits.upper if slide_joint.motion_limits else 0.0

        with ctx.pose({slide_joint: 0.0}):
            ctx.expect_within(
                drawer,
                body,
                axes="x",
                inner_elem="drawer_bottom",
                outer_elem=slide_name,
                margin=0.010,
                name=f"drawer {index} stays centered on its full-width slide",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                elem_a="drawer_bottom",
                elem_b=slide_name,
                min_overlap=0.80,
                name=f"drawer {index} bottom fully engages its slide when closed",
            )
            ctx.expect_contact(
                drawer,
                body,
                elem_a="drawer_bottom",
                elem_b=slide_name,
                contact_tol=1e-6,
                name=f"drawer {index} rests on its slide deck",
            )
            closed_pos = ctx.part_world_position(drawer)

        with ctx.pose({slide_joint: upper}):
            ctx.expect_within(
                drawer,
                body,
                axes="x",
                inner_elem="drawer_bottom",
                outer_elem=slide_name,
                margin=0.010,
                name=f"drawer {index} remains laterally guided when extended",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                elem_a="drawer_bottom",
                elem_b=slide_name,
                min_overlap=0.30,
                name=f"drawer {index} retains insertion on its slide at full extension",
            )
            ctx.expect_contact(
                drawer,
                body,
                elem_a="drawer_bottom",
                elem_b=slide_name,
                contact_tol=1e-6,
                name=f"drawer {index} stays supported by its slide deck when extended",
            )
            open_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"drawer {index} opens forward",
            closed_pos is not None
            and open_pos is not None
            and open_pos[1] > closed_pos[1] + 0.30,
            details=f"closed={closed_pos}, open={open_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
