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
    model = ArticulatedObject(name="lab_sample_cabinet")

    body_width = 0.82
    body_depth = 0.72
    body_height = 1.18
    insulation = 0.04
    front_frame_depth = 0.03

    top_service_depth = 0.18
    panel_width = 0.72
    panel_length = 0.14
    panel_thickness = 0.022
    hinge_axis_offset = 0.014

    drawer_face_width = 0.714
    drawer_face_height = 0.236
    drawer_face_thickness = 0.028
    drawer_bin_width = 0.64
    drawer_bin_depth = 0.58
    drawer_bin_height = 0.176
    drawer_wall = 0.006

    slide_length = 0.58
    slide_height = 0.045
    slide_flange = 0.003
    slide_web = 0.003
    slide_depth = 0.012
    moving_slide_width = 0.008
    slide_center_z_local = 0.094

    drawer_extension = 0.44
    drawer_bottoms = (0.04, 0.30, 0.56, 0.82)

    body_paint = model.material("body_paint", color=(0.88, 0.9, 0.93, 1.0))
    liner_gray = model.material("liner_gray", color=(0.78, 0.81, 0.84, 1.0))
    stainless = model.material("stainless", color=(0.72, 0.75, 0.78, 1.0))
    steel = model.material("steel", color=(0.46, 0.48, 0.51, 1.0))
    dark_trim = model.material("dark_trim", color=(0.18, 0.19, 0.2, 1.0))

    body = model.part("cabinet_body")
    body.visual(
        Box((body_width, body_depth, insulation)),
        origin=Origin(xyz=(0.0, body_depth / 2.0, insulation / 2.0)),
        material=body_paint,
        name="base_plinth",
    )
    body.visual(
        Box((insulation, body_depth, body_height)),
        origin=Origin(
            xyz=(-body_width / 2.0 + insulation / 2.0, body_depth / 2.0, body_height / 2.0)
        ),
        material=body_paint,
        name="left_wall",
    )
    body.visual(
        Box((insulation, body_depth, body_height)),
        origin=Origin(
            xyz=(body_width / 2.0 - insulation / 2.0, body_depth / 2.0, body_height / 2.0)
        ),
        material=body_paint,
        name="right_wall",
    )
    body.visual(
        Box((body_width, insulation, body_height - insulation)),
        origin=Origin(
            xyz=(0.0, body_depth - insulation / 2.0, insulation + (body_height - insulation) / 2.0)
        ),
        material=body_paint,
        name="back_wall",
    )
    body.visual(
        Box((body_width, body_depth - top_service_depth, insulation)),
        origin=Origin(
            xyz=(
                0.0,
                (body_depth - top_service_depth) / 2.0,
                body_height - insulation / 2.0,
            )
        ),
        material=body_paint,
        name="top_front",
    )
    body.visual(
        Box((insulation, top_service_depth, insulation)),
        origin=Origin(
            xyz=(
                -body_width / 2.0 + insulation / 2.0,
                body_depth - top_service_depth / 2.0,
                body_height - insulation / 2.0,
            )
        ),
        material=body_paint,
        name="top_rear_left_border",
    )
    body.visual(
        Box((insulation, top_service_depth, insulation)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - insulation / 2.0,
                body_depth - top_service_depth / 2.0,
                body_height - insulation / 2.0,
            )
        ),
        material=body_paint,
        name="top_rear_right_border",
    )
    body.visual(
        Box((insulation, front_frame_depth, body_height)),
        origin=Origin(
            xyz=(-body_width / 2.0 + insulation / 2.0, front_frame_depth / 2.0, body_height / 2.0)
        ),
        material=liner_gray,
        name="front_left_post",
    )
    body.visual(
        Box((insulation, front_frame_depth, body_height)),
        origin=Origin(
            xyz=(body_width / 2.0 - insulation / 2.0, front_frame_depth / 2.0, body_height / 2.0)
        ),
        material=liner_gray,
        name="front_right_post",
    )
    body.visual(
        Box((body_width, front_frame_depth, 0.02)),
        origin=Origin(xyz=(0.0, front_frame_depth / 2.0, 0.29)),
        material=liner_gray,
        name="divider_1",
    )
    body.visual(
        Box((body_width, front_frame_depth, 0.02)),
        origin=Origin(xyz=(0.0, front_frame_depth / 2.0, 0.55)),
        material=liner_gray,
        name="divider_2",
    )
    body.visual(
        Box((body_width, front_frame_depth, 0.02)),
        origin=Origin(xyz=(0.0, front_frame_depth / 2.0, 0.81)),
        material=liner_gray,
        name="divider_3",
    )

    hinge_y = body_depth - insulation + hinge_axis_offset
    hinge_z = body_height - panel_thickness / 2.0
    body.visual(
        Box((0.50, 0.018, 0.020)),
        origin=Origin(
            xyz=(0.0, body_depth - insulation / 2.0 + 0.001, body_height - 0.014)
        ),
        material=steel,
        name="rear_hinge_mount",
    )
    for hinge_name, hinge_x in (("rear_hinge_left", -0.18), ("rear_hinge_right", 0.18)):
        body.visual(
            Cylinder(radius=0.007, length=0.11),
            origin=Origin(
                xyz=(hinge_x, hinge_y, hinge_z - 0.004),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=steel,
            name=hinge_name,
        )

    def add_fixed_slide(drawer_index: int, z_center: float) -> None:
        for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
            web_x = side_sign * (body_width / 2.0 - insulation - slide_web / 2.0)
            flange_x = side_sign * (
                body_width / 2.0 - insulation - slide_depth / 2.0
            )
            slide_y_center = 0.04 + slide_length / 2.0
            prefix = f"drawer_{drawer_index}_{side_name}_slide"
            body.visual(
                Box((slide_web, slide_length, slide_height)),
                origin=Origin(xyz=(web_x, slide_y_center, z_center)),
                material=steel,
                name=f"{prefix}_web",
            )
            body.visual(
                Box((slide_depth, slide_length, slide_flange)),
                origin=Origin(
                    xyz=(flange_x, slide_y_center, z_center + slide_height / 2.0 - slide_flange / 2.0)
                ),
                material=steel,
                name=f"{prefix}_top_flange",
            )
            body.visual(
                Box((slide_depth, slide_length, slide_flange)),
                origin=Origin(
                    xyz=(flange_x, slide_y_center, z_center - slide_height / 2.0 + slide_flange / 2.0)
                ),
                material=steel,
                name=f"{prefix}_bottom_flange",
            )

    for drawer_index, drawer_bottom in enumerate(drawer_bottoms, start=1):
        add_fixed_slide(drawer_index, drawer_bottom + slide_center_z_local)

    def add_drawer_part(name: str) -> None:
        drawer = model.part(name)
        drawer.visual(
            Box((drawer_face_width, drawer_face_thickness, drawer_face_height)),
            origin=Origin(
                xyz=(0.0, drawer_face_thickness / 2.0, drawer_face_height / 2.0)
            ),
            material=stainless,
            name="drawer_face",
        )
        drawer.visual(
            Box((drawer_bin_width, drawer_bin_depth, drawer_wall)),
            origin=Origin(
                xyz=(
                    0.0,
                    drawer_face_thickness + drawer_bin_depth / 2.0,
                    drawer_wall / 2.0,
                )
            ),
            material=stainless,
            name="drawer_bottom",
        )
        drawer.visual(
            Box((drawer_wall, drawer_bin_depth, drawer_bin_height)),
            origin=Origin(
                xyz=(
                    -drawer_bin_width / 2.0 + drawer_wall / 2.0,
                    drawer_face_thickness + drawer_bin_depth / 2.0,
                    drawer_bin_height / 2.0,
                )
            ),
            material=stainless,
            name="left_drawer_wall",
        )
        drawer.visual(
            Box((drawer_wall, drawer_bin_depth, drawer_bin_height)),
            origin=Origin(
                xyz=(
                    drawer_bin_width / 2.0 - drawer_wall / 2.0,
                    drawer_face_thickness + drawer_bin_depth / 2.0,
                    drawer_bin_height / 2.0,
                )
            ),
            material=stainless,
            name="right_drawer_wall",
        )
        drawer.visual(
            Box((drawer_bin_width, drawer_wall, drawer_bin_height)),
            origin=Origin(
                xyz=(
                    0.0,
                    drawer_face_thickness + drawer_bin_depth - drawer_wall / 2.0,
                    drawer_bin_height / 2.0,
                )
            ),
            material=stainless,
            name="drawer_back",
        )

        handle_z = drawer_face_height * 0.56
        drawer.visual(
            Cylinder(radius=0.008, length=0.48),
            origin=Origin(xyz=(0.0, -0.018, handle_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_trim,
            name="handle_bar",
        )
        for handle_side, handle_x in (("left", -0.21), ("right", 0.21)):
            drawer.visual(
                Box((0.016, 0.010, 0.016)),
                origin=Origin(xyz=(handle_x, -0.005, handle_z)),
                material=dark_trim,
                name=f"handle_post_{handle_side}",
            )

        rail_y_center = 0.04 + slide_length / 2.0
        for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
            moving_x = side_sign * 0.350
            spacer_x = side_sign * 0.337
            drawer.visual(
                Box((moving_slide_width, slide_length, 0.030)),
                origin=Origin(xyz=(moving_x, rail_y_center, slide_center_z_local)),
                material=steel,
                name=f"{side_name}_slide_member",
            )
            drawer.visual(
                Box((0.034, 0.08, 0.006)),
                origin=Origin(xyz=(spacer_x, 0.11, slide_center_z_local)),
                material=steel,
                name=f"{side_name}_slide_front_spacer",
            )
            drawer.visual(
                Box((0.034, 0.08, 0.006)),
                origin=Origin(xyz=(spacer_x, 0.49, slide_center_z_local)),
                material=steel,
                name=f"{side_name}_slide_rear_spacer",
            )

    for drawer_index in range(1, 5):
        drawer_name = f"drawer_{drawer_index}"
        add_drawer_part(drawer_name)
        model.articulation(
            f"cabinet_to_{drawer_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer_name,
            origin=Origin(xyz=(0.0, 0.0, drawer_bottoms[drawer_index - 1])),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=180.0,
                velocity=0.35,
                lower=0.0,
                upper=drawer_extension,
            ),
        )

    panel = model.part("rear_access_panel")
    panel.visual(
        Box((panel_width, panel_length, panel_thickness)),
        origin=Origin(xyz=(0.0, -(panel_length / 2.0 + hinge_axis_offset), 0.0)),
        material=body_paint,
        name="panel_cover",
    )
    panel.visual(
        Box((0.24, 0.010, 0.012)),
        origin=Origin(
            xyz=(0.0, -(panel_length + hinge_axis_offset) + 0.006, -0.005)
        ),
        material=dark_trim,
        name="panel_pull_lip",
    )
    for hinge_name, hinge_x in (
        ("panel_hinge_leaf_left", -0.18),
        ("panel_hinge_leaf_right", 0.18),
    ):
        panel.visual(
            Box((0.08, 0.014, 0.010)),
            origin=Origin(xyz=(hinge_x, -0.021, -0.006)),
            material=steel,
            name=hinge_name,
        )
    model.articulation(
        "cabinet_to_rear_access_panel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=panel,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
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
    panel = object_model.get_part("rear_access_panel")
    panel_hinge = object_model.get_articulation("cabinet_to_rear_access_panel")

    top_front = body.get_visual("top_front")
    panel_cover = panel.get_visual("panel_cover")

    with ctx.pose({panel_hinge: 0.0}):
        panel_rest_aabb = ctx.part_element_world_aabb(panel, elem=panel_cover)
        top_front_aabb = ctx.part_element_world_aabb(body, elem=top_front)

    with ctx.pose({panel_hinge: 1.0}):
        panel_open_aabb = ctx.part_element_world_aabb(panel, elem=panel_cover)

    panel_top_flush = (
        panel_rest_aabb is not None
        and top_front_aabb is not None
        and abs(panel_rest_aabb[1][2] - top_front_aabb[1][2]) <= 0.003
        and panel_rest_aabb[0][1] >= top_front_aabb[1][1] - 0.005
    )
    ctx.check(
        "rear access panel sits flush with the cabinet top",
        panel_top_flush,
        details=f"panel={panel_rest_aabb}, top_front={top_front_aabb}",
    )

    ctx.check(
        "rear access panel opens upward",
        panel_rest_aabb is not None
        and panel_open_aabb is not None
        and ((panel_open_aabb[0][2] + panel_open_aabb[1][2]) / 2.0)
        > ((panel_rest_aabb[0][2] + panel_rest_aabb[1][2]) / 2.0) + 0.05
        and panel_open_aabb[0][1] > panel_rest_aabb[0][1] + 0.02,
        details=f"closed={panel_rest_aabb}, open={panel_open_aabb}",
    )

    for drawer_index in range(1, 5):
        drawer = object_model.get_part(f"drawer_{drawer_index}")
        drawer_joint = object_model.get_articulation(f"cabinet_to_drawer_{drawer_index}")
        fixed_web = body.get_visual(f"drawer_{drawer_index}_right_slide_web")
        moving_member = drawer.get_visual("right_slide_member")

        upper = drawer_joint.motion_limits.upper if drawer_joint.motion_limits else None
        if upper is None:
            ctx.fail(
                f"drawer {drawer_index} has extension travel",
                "Expected a prismatic motion upper limit for the drawer extension.",
            )
            continue

        with ctx.pose({drawer_joint: 0.0}):
            ctx.expect_within(
                drawer,
                body,
                axes="x",
                margin=0.0,
                name=f"drawer {drawer_index} stays within cabinet width",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                elem_a=moving_member,
                elem_b=fixed_web,
                min_overlap=0.54,
                name=f"drawer {drawer_index} slide is fully nested when closed",
            )
            drawer_rest_pos = ctx.part_world_position(drawer)

        with ctx.pose({drawer_joint: upper}):
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                elem_a=moving_member,
                elem_b=fixed_web,
                min_overlap=0.12,
                name=f"drawer {drawer_index} retains slide engagement at full extension",
            )
            drawer_open_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"drawer {drawer_index} extends out the front",
            drawer_rest_pos is not None
            and drawer_open_pos is not None
            and drawer_open_pos[1] < drawer_rest_pos[1] - 0.30,
            details=f"closed={drawer_rest_pos}, open={drawer_open_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
