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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tackle_box")

    body_green = model.material("body_green", rgba=(0.27, 0.39, 0.22, 1.0))
    tray_beige = model.material("tray_beige", rgba=(0.80, 0.77, 0.67, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))

    body_length = 0.36
    body_width = 0.22
    body_height = 0.14
    wall = 0.005
    bottom = 0.006

    body = model.part("body")
    body.visual(
        Box((body_length, body_width, bottom)),
        origin=Origin(xyz=(0.0, 0.0, bottom / 2.0)),
        material=body_green,
        name="bottom_shell",
    )
    wall_height = body_height - bottom
    wall_center_z = bottom + wall_height / 2.0
    body.visual(
        Box((wall, body_width, wall_height)),
        origin=Origin(xyz=(body_length / 2.0 - wall / 2.0, 0.0, wall_center_z)),
        material=body_green,
        name="front_wall",
    )
    body.visual(
        Box((wall, body_width, wall_height)),
        origin=Origin(xyz=(-body_length / 2.0 + wall / 2.0, 0.0, wall_center_z)),
        material=body_green,
        name="rear_wall",
    )
    side_length = body_length - 2.0 * wall
    body.visual(
        Box((side_length, wall, wall_height)),
        origin=Origin(xyz=(0.0, body_width / 2.0 - wall / 2.0, wall_center_z)),
        material=body_green,
        name="left_wall",
    )
    body.visual(
        Box((side_length, wall, wall_height)),
        origin=Origin(xyz=(0.0, -body_width / 2.0 + wall / 2.0, wall_center_z)),
        material=body_green,
        name="right_wall",
    )

    cheek_length = 0.082
    cheek_thickness = 0.004
    cheek_height = 0.050
    cheek_center_x = -body_length / 2.0 + cheek_length / 2.0 - 0.004
    cheek_center_z = 0.137
    body.visual(
        Box((cheek_length, cheek_thickness, cheek_height)),
        origin=Origin(
            xyz=(cheek_center_x, body_width / 2.0 - cheek_thickness / 2.0, cheek_center_z)
        ),
        material=body_green,
        name="left_cheek",
    )
    body.visual(
        Box((cheek_length, cheek_thickness, cheek_height)),
        origin=Origin(
            xyz=(cheek_center_x, -body_width / 2.0 + cheek_thickness / 2.0, cheek_center_z)
        ),
        material=body_green,
        name="right_cheek",
    )

    tray_floor_z = 0.080
    tray_floor_thickness = 0.004
    tray_floor_length = 0.286
    tray_floor_width = 0.178
    rail_height = 0.012
    rail_width = 0.016
    rail_z = tray_floor_z - tray_floor_thickness / 2.0 - rail_height / 2.0
    tray_center_x = 0.005
    body.visual(
        Box((tray_floor_length, tray_floor_width, tray_floor_thickness)),
        origin=Origin(xyz=(tray_center_x, 0.0, tray_floor_z)),
        material=tray_beige,
        name="tray_floor",
    )
    body.visual(
        Box((tray_floor_length, rail_width, rail_height)),
        origin=Origin(xyz=(tray_center_x, 0.097, rail_z)),
        material=tray_beige,
        name="left_tray_rail",
    )
    body.visual(
        Box((tray_floor_length, rail_width, rail_height)),
        origin=Origin(xyz=(tray_center_x, -0.097, rail_z)),
        material=tray_beige,
        name="right_tray_rail",
    )
    divider_height = 0.024
    divider_center_z = tray_floor_z + tray_floor_thickness / 2.0 + divider_height / 2.0
    body.visual(
        Box((0.240, 0.003, divider_height)),
        origin=Origin(xyz=(tray_center_x, 0.0, divider_center_z)),
        material=tray_beige,
        name="center_divider",
    )
    body.visual(
        Box((0.003, 0.158, divider_height)),
        origin=Origin(xyz=(-0.078, 0.0, divider_center_z)),
        material=tray_beige,
        name="rear_divider",
    )
    body.visual(
        Box((0.003, 0.158, divider_height)),
        origin=Origin(xyz=(0.088, 0.0, divider_center_z)),
        material=tray_beige,
        name="front_divider",
    )
    body.visual(
        Box((0.020, 0.170, 0.016)),
        origin=Origin(xyz=(-0.133, 0.0, tray_floor_z - 0.004)),
        material=dark_trim,
        name="tray_rear_support",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_length, body_width, body_height + 0.03)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, (body_height + 0.03) / 2.0)),
    )

    lid = model.part("lid")
    lid_panel_length = 0.360
    lid_panel_width = 0.206
    lid_thickness = 0.008
    lid.visual(
        Box((lid_panel_length, lid_panel_width, lid_thickness)),
        origin=Origin(xyz=(0.192, 0.0, -0.002)),
        material=body_green,
        name="top_panel",
    )
    lip_height = 0.018
    lip_center_z = -0.015
    lid.visual(
        Box((0.012, lid_panel_width, lip_height)),
        origin=Origin(xyz=(0.006, 0.0, lip_center_z)),
        material=body_green,
        name="rear_lip",
    )
    lid.visual(
        Box((0.006, 0.198, lip_height)),
        origin=Origin(xyz=(0.375, 0.0, lip_center_z)),
        material=body_green,
        name="front_lip",
    )
    lid.visual(
        Box((0.338, 0.006, lip_height)),
        origin=Origin(xyz=(0.191, 0.100, lip_center_z)),
        material=body_green,
        name="left_lip",
    )
    lid.visual(
        Box((0.338, 0.006, lip_height)),
        origin=Origin(xyz=(0.191, -0.100, lip_center_z)),
        material=body_green,
        name="right_lip",
    )
    barrel_radius = 0.006
    barrel_length = 0.038
    lid.visual(
        Cylinder(radius=barrel_radius, length=barrel_length),
        origin=Origin(xyz=(0.0, 0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="left_hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=barrel_radius, length=barrel_length),
        origin=Origin(xyz=(0.0, -0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_steel,
        name="right_hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.360, 0.210, 0.040)),
        mass=0.55,
        origin=Origin(xyz=(0.180, 0.0, -0.004)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.192, 0.0, 0.146)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(118.0),
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
    hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="top_panel",
        negative_elem="front_wall",
        max_gap=0.001,
        max_penetration=1e-5,
        name="lid panel sits on the box rim",
    )
    ctx.expect_gap(
        body,
        lid,
        axis="y",
        positive_elem="left_cheek",
        negative_elem="top_panel",
        min_gap=0.002,
        max_gap=0.004,
        name="left cheek clears the lid side",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="y",
        positive_elem="top_panel",
        negative_elem="right_cheek",
        min_gap=0.002,
        max_gap=0.004,
        name="right cheek clears the lid side",
    )
    ctx.expect_overlap(
        body,
        lid,
        axes="x",
        elem_a="left_cheek",
        elem_b="top_panel",
        min_overlap=0.075,
        name="left cheek brackets the rear of the lid",
    )
    ctx.expect_overlap(
        body,
        lid,
        axes="x",
        elem_a="right_cheek",
        elem_b="top_panel",
        min_overlap=0.075,
        name="right cheek brackets the rear of the lid",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="top_panel",
        negative_elem="center_divider",
        min_gap=0.03,
        name="fixed tray dividers stay below the closed lid",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_lip")
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        opened_front = ctx.part_element_world_aabb(lid, elem="front_lip")
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_front is not None
        and opened_front is not None
        and opened_front[1][2] > closed_front[1][2] + 0.30
        and opened_front[1][0] < closed_front[0][0] - 0.10,
        details=f"closed_front={closed_front}, opened_front={opened_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
