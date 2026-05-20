from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tackle_box")

    body_color = model.material("body_color", rgba=(0.22, 0.29, 0.20, 1.0))
    lid_color = model.material("lid_color", rgba=(0.28, 0.36, 0.24, 1.0))
    tray_color = model.material("tray_color", rgba=(0.76, 0.77, 0.72, 1.0))

    body_w = 0.36
    body_d = 0.22
    body_h = 0.16
    wall_t = 0.008
    panel_t = 0.006
    side_gap = 0.004
    front_gap = 0.004
    rear_gap = 0.0

    inner_w = body_w - 2.0 * wall_t
    inner_d = body_d - 2.0 * wall_t
    lid_w = inner_w - 2.0 * side_gap
    lid_d = inner_d - front_gap - rear_gap
    rear_inner_y = -body_d / 2.0 + wall_t

    tray_w = 0.304
    tray_d = 0.152
    tray_floor_t = 0.004
    tray_wall_t = 0.004
    tray_height = 0.022
    tray_floor_z = 0.085
    tray_center_y = 0.014
    support_w = 0.024
    support_t = 0.006

    body = model.part("body")
    body.visual(
        Box((body_w, body_d, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, wall_t / 2.0)),
        material=body_color,
        name="bottom_shell",
    )
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(-body_w / 2.0 + wall_t / 2.0, 0.0, body_h / 2.0)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, body_d, body_h)),
        origin=Origin(xyz=(body_w / 2.0 - wall_t / 2.0, 0.0, body_h / 2.0)),
        material=body_color,
        name="right_wall",
    )
    body.visual(
        Box((body_w - 1.5 * wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, body_d / 2.0 - wall_t / 2.0, body_h / 2.0)),
        material=body_color,
        name="front_wall",
    )
    body.visual(
        Box((body_w - 1.5 * wall_t, wall_t, body_h)),
        origin=Origin(xyz=(0.0, -body_d / 2.0 + wall_t / 2.0, body_h / 2.0)),
        material=body_color,
        name="rear_wall",
    )
    body.visual(
        Box((support_w, tray_d + 0.008, support_t)),
        origin=Origin(
            xyz=(
                -body_w / 2.0 + wall_t + support_w / 2.0,
                tray_center_y,
                tray_floor_z - support_t / 2.0,
            )
        ),
        material=tray_color,
        name="tray_support_left",
    )
    body.visual(
        Box((support_w, tray_d + 0.008, support_t)),
        origin=Origin(
            xyz=(
                body_w / 2.0 - wall_t - support_w / 2.0,
                tray_center_y,
                tray_floor_z - support_t / 2.0,
            )
        ),
        material=tray_color,
        name="tray_support_right",
    )
    body.visual(
        Box((tray_w, tray_d, tray_floor_t)),
        origin=Origin(xyz=(0.0, tray_center_y, tray_floor_z)),
        material=tray_color,
        name="tray_floor",
    )
    body.visual(
        Box((tray_wall_t, tray_d, tray_height)),
        origin=Origin(
            xyz=(
                -tray_w / 2.0 + tray_wall_t / 2.0,
                tray_center_y,
                tray_floor_z + tray_height / 2.0,
            )
        ),
        material=tray_color,
        name="tray_left_side",
    )
    body.visual(
        Box((tray_wall_t, tray_d, tray_height)),
        origin=Origin(
            xyz=(
                tray_w / 2.0 - tray_wall_t / 2.0,
                tray_center_y,
                tray_floor_z + tray_height / 2.0,
            )
        ),
        material=tray_color,
        name="tray_right_side",
    )
    body.visual(
        Box((tray_w, tray_wall_t, tray_height)),
        origin=Origin(
            xyz=(
                0.0,
                tray_center_y + tray_d / 2.0 - tray_wall_t / 2.0,
                tray_floor_z + tray_height / 2.0,
            )
        ),
        material=tray_color,
        name="tray_front_side",
    )
    body.visual(
        Box((tray_w, tray_wall_t, tray_height)),
        origin=Origin(
            xyz=(
                0.0,
                tray_center_y - tray_d / 2.0 + tray_wall_t / 2.0,
                tray_floor_z + tray_height / 2.0,
            )
        ),
        material=tray_color,
        name="tray_rear_side",
    )
    body.visual(
        Box((tray_wall_t, tray_d - 0.016, tray_height)),
        origin=Origin(
            xyz=(-0.052, tray_center_y, tray_floor_z + tray_height / 2.0),
        ),
        material=tray_color,
        name="tray_divider_left",
    )
    body.visual(
        Box((tray_wall_t, tray_d - 0.016, tray_height)),
        origin=Origin(
            xyz=(0.056, tray_center_y, tray_floor_z + tray_height / 2.0),
        ),
        material=tray_color,
        name="tray_divider_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, lid_d, panel_t)),
        origin=Origin(xyz=(0.0, lid_d / 2.0, panel_t / 2.0)),
        material=lid_color,
        name="lid_panel",
    )
    lid.visual(
        Box((lid_w, 0.02, 0.003)),
        origin=Origin(xyz=(0.0, lid_d - 0.014, panel_t + 0.0015)),
        material=lid_color,
        name="front_stiffener",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, 0.02)),
        mass=0.55,
        origin=Origin(xyz=(0.0, lid_d / 2.0, 0.01)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, rear_inner_y + rear_gap, body_h)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.0065,
            max_penetration=0.0,
            positive_elem="lid_panel",
            name="lid panel rests cleanly on the box rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.19,
            elem_a="lid_panel",
            name="lid footprint covers the body opening",
        )

        lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        left_wall_aabb = ctx.part_element_world_aabb(body, elem="left_wall")
        right_wall_aabb = ctx.part_element_world_aabb(body, elem="right_wall")
        if lid_aabb is None or left_wall_aabb is None or right_wall_aabb is None:
            ctx.fail("closed lid side margins measurable", "expected lid panel and body walls to exist")
        else:
            left_margin = lid_aabb[0][0] - left_wall_aabb[1][0]
            right_margin = right_wall_aabb[0][0] - lid_aabb[1][0]
            ctx.check(
                "lid stays centered side to side",
                abs(left_margin - right_margin) <= 0.0015
                and left_margin >= 0.0025
                and right_margin >= 0.0025,
                details=f"left_margin={left_margin:.4f}, right_margin={right_margin:.4f}",
            )

    closed_panel_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: math.radians(105.0)}):
        open_panel_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        if closed_panel_aabb is None or open_panel_aabb is None:
            ctx.fail("open lid pose measurable", "expected lid panel AABB in both poses")
        else:
            ctx.check(
                "lid opens upward from the rear hinge",
                open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.10
                and open_panel_aabb[0][1] < closed_panel_aabb[0][1] + 0.01,
                details=(
                    f"closed_max_z={closed_panel_aabb[1][2]:.4f}, "
                    f"open_max_z={open_panel_aabb[1][2]:.4f}, "
                    f"closed_min_y={closed_panel_aabb[0][1]:.4f}, "
                    f"open_min_y={open_panel_aabb[0][1]:.4f}"
                ),
            )

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
