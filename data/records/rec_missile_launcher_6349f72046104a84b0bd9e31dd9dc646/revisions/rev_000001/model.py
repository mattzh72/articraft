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
    model = ArticulatedObject(name="short_range_launcher_turret")

    base_dark = model.material("base_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    turret_gray = model.material("turret_gray", rgba=(0.39, 0.42, 0.44, 1.0))
    launcher_olive = model.material("launcher_olive", rgba=(0.37, 0.43, 0.31, 1.0))
    sensor_black = model.material("sensor_black", rgba=(0.08, 0.09, 0.10, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.20, 0.48, 0.58, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.47, 0.50, 0.52, 1.0))

    yaw_base = model.part("yaw_base")
    yaw_base.visual(
        Cylinder(radius=0.34, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=base_dark,
        name="base_plate",
    )
    yaw_base.visual(
        Cylinder(radius=0.23, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=turret_gray,
        name="pedestal_skirt",
    )
    yaw_base.visual(
        Cylinder(radius=0.18, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=base_dark,
        name="bearing_housing",
    )
    yaw_base.visual(
        Cylinder(radius=0.21, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        material=turret_gray,
        name="bearing_cap",
    )
    yaw_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=0.28),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    turret_head = model.part("turret_head")
    turret_head.visual(
        Cylinder(radius=0.20, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=base_dark,
        name="yaw_bearing_ring",
    )
    turret_head.visual(
        Box((0.26, 0.20, 0.16)),
        origin=Origin(xyz=(-0.04, -0.02, 0.13)),
        material=turret_gray,
        name="body_lower",
    )
    turret_head.visual(
        Box((0.22, 0.18, 0.08)),
        origin=Origin(xyz=(-0.07, -0.02, 0.24)),
        material=turret_gray,
        name="body_roof",
    )
    turret_head.visual(
        Box((0.10, 0.16, 0.08)),
        origin=Origin(xyz=(0.05, -0.02, 0.14)),
        material=turret_gray,
        name="front_nose",
    )
    turret_head.visual(
        Box((0.08, 0.34, 0.12)),
        origin=Origin(xyz=(0.085, 0.12, 0.18)),
        material=turret_gray,
        name="launcher_spine",
    )
    turret_head.visual(
        Box((0.07, 0.02, 0.16)),
        origin=Origin(xyz=(0.135, -0.04, 0.23)),
        material=base_dark,
        name="inner_cradle_arm",
    )
    turret_head.visual(
        Box((0.07, 0.02, 0.16)),
        origin=Origin(xyz=(0.135, 0.28, 0.23)),
        material=base_dark,
        name="outer_cradle_arm",
    )
    turret_head.visual(
        Box((0.08, 0.08, 0.08)),
        origin=Origin(xyz=(0.10, -0.14, 0.18)),
        material=turret_gray,
        name="sensor_mount",
    )
    turret_head.visual(
        Box((0.18, 0.12, 0.10)),
        origin=Origin(xyz=(0.20, -0.18, 0.24)),
        material=sensor_black,
        name="sensor_pod_body",
    )
    turret_head.visual(
        Box((0.06, 0.12, 0.12)),
        origin=Origin(xyz=(0.31, -0.18, 0.24)),
        material=base_dark,
        name="sensor_pod_shade",
    )
    turret_head.visual(
        Cylinder(radius=0.032, length=0.05),
        origin=Origin(xyz=(0.345, -0.18, 0.24), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_blue,
        name="sensor_primary_lens",
    )
    turret_head.visual(
        Cylinder(radius=0.018, length=0.03),
        origin=Origin(xyz=(0.335, -0.145, 0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_blue,
        name="sensor_secondary_lens",
    )
    turret_head.inertial = Inertial.from_geometry(
        Box((0.46, 0.48, 0.36)),
        mass=68.0,
        origin=Origin(xyz=(0.03, 0.02, 0.18)),
    )

    launcher_box = model.part("launcher_box")
    launcher_box.visual(
        Box((0.40, 0.24, 0.22)),
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
        material=launcher_olive,
        name="launcher_shell",
    )
    launcher_box.visual(
        Box((0.08, 0.18, 0.18)),
        origin=Origin(xyz=(-0.03, 0.0, 0.0)),
        material=turret_gray,
        name="launcher_rear_block",
    )
    launcher_box.visual(
        Box((0.03, 0.22, 0.20)),
        origin=Origin(xyz=(0.385, 0.0, 0.0)),
        material=base_dark,
        name="launcher_faceplate",
    )
    launcher_box.visual(
        Cylinder(radius=0.045, length=0.03),
        origin=Origin(xyz=(0.0, -0.135, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=base_dark,
        name="left_trunnion",
    )
    launcher_box.visual(
        Cylinder(radius=0.045, length=0.03),
        origin=Origin(xyz=(0.0, 0.135, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=base_dark,
        name="right_trunnion",
    )
    for name, y_pos, z_pos in (
        ("tube_upper_left", -0.065, 0.055),
        ("tube_upper_right", 0.065, 0.055),
        ("tube_lower_left", -0.065, -0.055),
        ("tube_lower_right", 0.065, -0.055),
    ):
        launcher_box.visual(
            Cylinder(radius=0.040, length=0.05),
            origin=Origin(xyz=(0.395, y_pos, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=base_dark,
            name=name,
        )
    launcher_box.visual(
        Box((0.02, 0.03, 0.03)),
        origin=Origin(xyz=(-0.04, -0.075, 0.125)),
        material=turret_gray,
        name="hinge_lug_left",
    )
    launcher_box.visual(
        Box((0.02, 0.03, 0.03)),
        origin=Origin(xyz=(-0.04, 0.075, 0.125)),
        material=turret_gray,
        name="hinge_lug_right",
    )
    launcher_box.inertial = Inertial.from_geometry(
        Box((0.47, 0.30, 0.28)),
        mass=28.0,
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
    )

    reload_cover = model.part("reload_cover")
    reload_cover.visual(
        Box((0.28, 0.18, 0.02)),
        origin=Origin(xyz=(0.14, 0.0, 0.01)),
        material=cover_gray,
        name="reload_cover_panel",
    )
    reload_cover.visual(
        Box((0.24, 0.015, 0.035)),
        origin=Origin(xyz=(0.15, -0.0825, 0.0175)),
        material=cover_gray,
        name="cover_left_flange",
    )
    reload_cover.visual(
        Box((0.24, 0.015, 0.035)),
        origin=Origin(xyz=(0.15, 0.0825, 0.0175)),
        material=cover_gray,
        name="cover_right_flange",
    )
    reload_cover.visual(
        Box((0.05, 0.018, 0.025)),
        origin=Origin(xyz=(0.23, 0.0, 0.0325)),
        material=base_dark,
        name="cover_handle",
    )
    reload_cover.inertial = Inertial.from_geometry(
        Box((0.30, 0.20, 0.05)),
        mass=3.0,
        origin=Origin(xyz=(0.15, 0.0, 0.02)),
    )

    model.articulation(
        "yaw_rotation",
        ArticulationType.CONTINUOUS,
        parent=yaw_base,
        child=turret_head,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=1.8),
    )
    model.articulation(
        "launcher_tilt",
        ArticulationType.REVOLUTE,
        parent=turret_head,
        child=launcher_box,
        origin=Origin(xyz=(0.20, 0.12, 0.24)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=-0.35,
            upper=0.95,
        ),
    )
    model.articulation(
        "reload_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=launcher_box,
        child=reload_cover,
        origin=Origin(xyz=(-0.02, 0.0, 0.11)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=0.0,
            upper=2.15,
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

    yaw_base = object_model.get_part("yaw_base")
    turret_head = object_model.get_part("turret_head")
    launcher_box = object_model.get_part("launcher_box")
    reload_cover = object_model.get_part("reload_cover")

    yaw_rotation = object_model.get_articulation("yaw_rotation")
    launcher_tilt = object_model.get_articulation("launcher_tilt")
    reload_cover_hinge = object_model.get_articulation("reload_cover_hinge")

    ctx.check(
        "all major parts exist",
        all(part is not None for part in (yaw_base, turret_head, launcher_box, reload_cover)),
    )
    ctx.check(
        "joint axes match turret mechanisms",
        yaw_rotation.axis == (0.0, 0.0, 1.0)
        and launcher_tilt.axis == (0.0, -1.0, 0.0)
        and reload_cover_hinge.axis == (0.0, -1.0, 0.0),
        details=(
            f"yaw={yaw_rotation.axis}, tilt={launcher_tilt.axis}, "
            f"cover={reload_cover_hinge.axis}"
        ),
    )

    with ctx.pose({launcher_tilt: 0.0, reload_cover_hinge: 0.0}):
        ctx.expect_contact(
            turret_head,
            yaw_base,
            elem_a="yaw_bearing_ring",
            elem_b="bearing_cap",
            name="turret head seats on yaw bearing",
        )
        ctx.expect_contact(
            launcher_box,
            turret_head,
            elem_a="left_trunnion",
            elem_b="inner_cradle_arm",
            name="left trunnion is carried by the inner cradle arm",
        )
        ctx.expect_contact(
            launcher_box,
            turret_head,
            elem_a="right_trunnion",
            elem_b="outer_cradle_arm",
            name="right trunnion is carried by the outer cradle arm",
        )
        ctx.expect_gap(
            reload_cover,
            launcher_box,
            axis="z",
            positive_elem="reload_cover_panel",
            negative_elem="launcher_shell",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed reload cover sits flush on the launcher roof",
        )
        ctx.expect_overlap(
            reload_cover,
            launcher_box,
            axes="xy",
            elem_a="reload_cover_panel",
            elem_b="launcher_shell",
            min_overlap=0.16,
            name="reload cover stays centered over the launcher opening",
        )

    rest_face = ctx.part_element_world_aabb(launcher_box, elem="launcher_faceplate")
    with ctx.pose({launcher_tilt: 0.75}):
        raised_face = ctx.part_element_world_aabb(launcher_box, elem="launcher_faceplate")
    ctx.check(
        "positive launcher tilt raises the muzzle box",
        rest_face is not None
        and raised_face is not None
        and ((raised_face[0][2] + raised_face[1][2]) * 0.5) > ((rest_face[0][2] + rest_face[1][2]) * 0.5) + 0.10,
        details=f"rest_face={rest_face}, raised_face={raised_face}",
    )

    rest_sensor = ctx.part_element_world_aabb(turret_head, elem="sensor_primary_lens")
    with ctx.pose({yaw_rotation: 0.70}):
        yawed_sensor = ctx.part_element_world_aabb(turret_head, elem="sensor_primary_lens")
    ctx.check(
        "positive yaw rotates the head around the vertical base axis",
        rest_sensor is not None
        and yawed_sensor is not None
        and ((yawed_sensor[0][1] + yawed_sensor[1][1]) * 0.5) > ((rest_sensor[0][1] + rest_sensor[1][1]) * 0.5) + 0.20,
        details=f"rest_sensor={rest_sensor}, yawed_sensor={yawed_sensor}",
    )

    closed_cover = ctx.part_element_world_aabb(reload_cover, elem="reload_cover_panel")
    with ctx.pose({reload_cover_hinge: 1.65}):
        open_cover = ctx.part_element_world_aabb(reload_cover, elem="reload_cover_panel")
    ctx.check(
        "reload cover rotates upward from the launcher roof",
        closed_cover is not None
        and open_cover is not None
        and open_cover[1][2] > closed_cover[1][2] + 0.20
        and open_cover[0][0] < closed_cover[0][0] - 0.01,
        details=f"closed_cover={closed_cover}, open_cover={open_cover}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
