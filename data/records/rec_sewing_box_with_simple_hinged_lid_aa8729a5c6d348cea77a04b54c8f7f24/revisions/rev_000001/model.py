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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_sewing_box")

    outer_w = 0.28
    outer_d = 0.18
    body_h = 0.11
    wall_t = 0.008
    bottom_t = 0.006
    lip_t = 0.006
    lip_h = 0.010
    lid_t = 0.008
    barrel_r = 0.005
    body_barrel_len = 0.050
    lid_barrel_len = 0.100
    knuckle_x = 0.084
    wall_h = body_h - bottom_t

    wood = model.material("wood", color=(0.66, 0.52, 0.35))
    wood_dark = model.material("wood_dark", color=(0.53, 0.39, 0.25))
    brass = model.material("brass", color=(0.72, 0.60, 0.31))

    body = model.part("body")
    body.visual(
        Box((outer_w, outer_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=wood_dark,
        name="bottom_panel",
    )
    body.visual(
        Box((wall_t, outer_d, wall_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + wall_t / 2.0, 0.0, bottom_t + wall_h / 2.0)),
        material=wood,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, outer_d, wall_h)),
        origin=Origin(xyz=(outer_w / 2.0 - wall_t / 2.0, 0.0, bottom_t + wall_h / 2.0)),
        material=wood,
        name="right_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, outer_d / 2.0 - wall_t / 2.0, bottom_t + wall_h / 2.0)),
        material=wood,
        name="front_wall",
    )
    body.visual(
        Box((outer_w - 2.0 * wall_t, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -outer_d / 2.0 + wall_t / 2.0, bottom_t + wall_h / 2.0)),
        material=wood,
        name="rear_wall",
    )

    body.visual(
        Box((lip_t, outer_d - 2.0 * wall_t, lip_h)),
        origin=Origin(
            xyz=(-outer_w / 2.0 + wall_t + lip_t / 2.0, 0.0, body_h - lip_h / 2.0)
        ),
        material=wood_dark,
        name="opening_lip_left",
    )
    body.visual(
        Box((lip_t, outer_d - 2.0 * wall_t, lip_h)),
        origin=Origin(
            xyz=(outer_w / 2.0 - wall_t - lip_t / 2.0, 0.0, body_h - lip_h / 2.0)
        ),
        material=wood_dark,
        name="opening_lip_right",
    )
    body.visual(
        Box((outer_w - 2.0 * (wall_t + lip_t), lip_t, lip_h)),
        origin=Origin(
            xyz=(0.0, outer_d / 2.0 - wall_t - lip_t / 2.0, body_h - lip_h / 2.0)
        ),
        material=wood_dark,
        name="opening_lip_front",
    )
    body.visual(
        Box((outer_w - 2.0 * (wall_t + lip_t), lip_t, lip_h)),
        origin=Origin(
            xyz=(0.0, -outer_d / 2.0 + wall_t + lip_t / 2.0, body_h - lip_h / 2.0)
        ),
        material=wood_dark,
        name="opening_lip_rear",
    )

    for side, x in (("left", -knuckle_x), ("right", knuckle_x)):
        body.visual(
            Box((body_barrel_len + 0.008, 0.008, 0.010)),
            origin=Origin(xyz=(x, -outer_d / 2.0 - 0.002, body_h + 0.001)),
            material=wood_dark,
            name=f"hinge_support_{side}",
        )
        body.visual(
            Cylinder(radius=barrel_r, length=body_barrel_len),
            origin=Origin(
                xyz=(x, -outer_d / 2.0, body_h + barrel_r),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=brass,
            name=f"hinge_knuckle_{side}",
        )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=barrel_r, length=lid_barrel_len),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((outer_w - 0.016, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.008, -0.004)),
        material=wood_dark,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Box((outer_w, outer_d - 0.014, lid_t)),
        origin=Origin(xyz=(0.0, 0.097, -barrel_r + lid_t / 2.0)),
        material=wood,
        name="lid_panel",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -outer_d / 2.0, body_h + barrel_r)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.95,
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
    hinge = object_model.get_articulation("lid_hinge")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="opening_lip_front",
        max_gap=0.001,
        max_penetration=0.0,
        name="lid closes flush to the body opening",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        min_overlap=0.16,
        name="lid panel covers the sewing box footprint",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.2}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    closed_center_z = None
    open_center_z = None
    if closed_aabb is not None:
        closed_center_z = 0.5 * (closed_aabb[0][2] + closed_aabb[1][2])
    if open_aabb is not None:
        open_center_z = 0.5 * (open_aabb[0][2] + open_aabb[1][2])

    ctx.check(
        "lid opens upward on the rear hinge",
        closed_center_z is not None
        and open_center_z is not None
        and open_center_z > closed_center_z + 0.07,
        details=f"closed_center_z={closed_center_z}, open_center_z={open_center_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
