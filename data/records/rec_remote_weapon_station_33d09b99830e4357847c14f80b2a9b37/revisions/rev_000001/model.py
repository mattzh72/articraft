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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_weapon_station")

    pedestal_paint = model.material("pedestal_paint", rgba=(0.18, 0.19, 0.20, 1.0))
    armor_paint = model.material("armor_paint", rgba=(0.34, 0.38, 0.28, 1.0))
    weapon_finish = model.material("weapon_finish", rgba=(0.14, 0.15, 0.16, 1.0))
    hinge_finish = model.material("hinge_finish", rgba=(0.24, 0.24, 0.24, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.30, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=pedestal_paint,
        name="base_skirt",
    )
    pedestal.visual(
        Cylinder(radius=0.15, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        material=pedestal_paint,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.21, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
        material=pedestal_paint,
        name="bearing_collar",
    )

    upper_body = model.part("upper_body")
    upper_body.visual(
        Box((0.46, 0.36, 0.03)),
        origin=Origin(xyz=(0.00, 0.00, 0.015)),
        material=armor_paint,
        name="hull_floor",
    )
    upper_body.visual(
        Box((0.40, 0.34, 0.03)),
        origin=Origin(xyz=(-0.02, 0.00, 0.265)),
        material=armor_paint,
        name="hull_roof",
    )
    upper_body.visual(
        Box((0.03, 0.36, 0.26)),
        origin=Origin(xyz=(-0.215, 0.00, 0.145)),
        material=armor_paint,
        name="rear_wall",
    )
    upper_body.visual(
        Box((0.40, 0.03, 0.22)),
        origin=Origin(xyz=(-0.02, 0.165, 0.140)),
        material=armor_paint,
        name="right_side_wall",
    )
    upper_body.visual(
        Box((0.12, 0.03, 0.22)),
        origin=Origin(xyz=(-0.16, -0.165, 0.140)),
        material=armor_paint,
        name="left_side_rear_strip",
    )
    upper_body.visual(
        Box((0.05, 0.03, 0.22)),
        origin=Origin(xyz=(0.155, -0.165, 0.140)),
        material=armor_paint,
        name="left_side_front_strip",
    )
    upper_body.visual(
        Box((0.23, 0.03, 0.06)),
        origin=Origin(xyz=(0.015, -0.165, 0.250)),
        material=armor_paint,
        name="left_side_top_strip",
    )
    upper_body.visual(
        Box((0.23, 0.03, 0.08)),
        origin=Origin(xyz=(0.015, -0.165, 0.040)),
        material=armor_paint,
        name="left_side_bottom_strip",
    )
    upper_body.visual(
        Box((0.14, 0.08, 0.18)),
        origin=Origin(xyz=(0.24, -0.14, 0.160)),
        material=armor_paint,
        name="left_yoke_arm",
    )
    upper_body.visual(
        Box((0.14, 0.08, 0.18)),
        origin=Origin(xyz=(0.24, 0.14, 0.160)),
        material=armor_paint,
        name="right_yoke_arm",
    )
    upper_body.visual(
        Box((0.06, 0.08, 0.14)),
        origin=Origin(xyz=(0.18, -0.14, 0.070)),
        material=armor_paint,
        name="left_front_shoulder",
    )
    upper_body.visual(
        Box((0.06, 0.08, 0.14)),
        origin=Origin(xyz=(0.18, 0.14, 0.070)),
        material=armor_paint,
        name="right_front_shoulder",
    )
    upper_body.visual(
        Box((0.04, 0.02, 0.06)),
        origin=Origin(xyz=(0.24, -0.11, 0.160)),
        material=hinge_finish,
        name="left_trunnion_pad",
    )
    upper_body.visual(
        Box((0.04, 0.02, 0.06)),
        origin=Origin(xyz=(0.24, 0.11, 0.160)),
        material=hinge_finish,
        name="right_trunnion_pad",
    )
    upper_body.visual(
        Cylinder(radius=0.007, length=0.036),
        origin=Origin(xyz=(0.13, -0.187, 0.202)),
        material=hinge_finish,
        name="panel_hinge_upper",
    )
    upper_body.visual(
        Cylinder(radius=0.007, length=0.036),
        origin=Origin(xyz=(0.13, -0.187, 0.098)),
        material=hinge_finish,
        name="panel_hinge_lower",
    )

    weapon_mount = model.part("weapon_mount")
    weapon_mount.visual(
        Cylinder(radius=0.018, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_finish,
        name="trunnion_axle",
    )
    weapon_mount.visual(
        Box((0.30, 0.18, 0.18)),
        origin=Origin(xyz=(0.08, 0.00, 0.000)),
        material=weapon_finish,
        name="receiver_body",
    )
    weapon_mount.visual(
        Cylinder(radius=0.045, length=0.42),
        origin=Origin(xyz=(0.44, 0.00, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weapon_finish,
        name="barrel_shroud",
    )
    weapon_mount.visual(
        Cylinder(radius=0.022, length=0.12),
        origin=Origin(xyz=(0.71, 0.00, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=weapon_finish,
        name="muzzle",
    )
    weapon_mount.visual(
        Box((0.11, 0.10, 0.08)),
        origin=Origin(xyz=(0.05, 0.00, 0.130)),
        material=weapon_finish,
        name="sight_housing",
    )
    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((0.23, 0.012, 0.14)),
        origin=Origin(xyz=(-0.115, 0.013, 0.0)),
        material=armor_paint,
        name="panel_plate",
    )
    service_panel.visual(
        Cylinder(radius=0.007, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=hinge_finish,
        name="panel_center_knuckle",
    )
    service_panel.visual(
        Box((0.030, 0.010, 0.028)),
        origin=Origin(xyz=(-0.165, 0.023, 0.0)),
        material=hinge_finish,
        name="panel_latch",
    )

    model.articulation(
        "pedestal_pan",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=upper_body,
        origin=Origin(xyz=(0.0, 0.0, 0.84)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=150.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "weapon_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_body,
        child=weapon_mount,
        origin=Origin(xyz=(0.24, 0.0, 0.16)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=-0.35, upper=1.05),
    )
    model.articulation(
        "service_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=upper_body,
        child=service_panel,
        origin=Origin(xyz=(0.13, -0.187, 0.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.45),
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

    pedestal = object_model.get_part("pedestal")
    upper_body = object_model.get_part("upper_body")
    weapon_mount = object_model.get_part("weapon_mount")
    service_panel = object_model.get_part("service_panel")

    pan = object_model.get_articulation("pedestal_pan")
    pitch = object_model.get_articulation("weapon_pitch")
    panel_hinge = object_model.get_articulation("service_panel_hinge")

    ctx.check(
        "pedestal pan uses vertical axis",
        tuple(pan.axis) == (0.0, 0.0, 1.0),
        details=f"axis={pan.axis}",
    )
    ctx.check(
        "weapon pitch uses horizontal axis",
        tuple(pitch.axis) == (0.0, -1.0, 0.0),
        details=f"axis={pitch.axis}",
    )
    ctx.check(
        "service panel hinge uses vertical axis",
        tuple(panel_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={panel_hinge.axis}",
    )

    ctx.expect_contact(
        pedestal,
        upper_body,
        elem_a="bearing_collar",
        elem_b="hull_floor",
        contact_tol=0.001,
        name="upper body seats on bearing collar",
    )
    ctx.expect_contact(
        weapon_mount,
        upper_body,
        elem_a="trunnion_axle",
        elem_b="left_trunnion_pad",
        contact_tol=0.001,
        name="weapon mount is carried by left trunnion pad",
    )
    ctx.expect_contact(
        weapon_mount,
        upper_body,
        elem_a="trunnion_axle",
        elem_b="right_trunnion_pad",
        contact_tol=0.001,
        name="weapon mount is carried by right trunnion pad",
    )
    ctx.expect_overlap(
        service_panel,
        upper_body,
        axes="xz",
        elem_a="panel_plate",
        min_overlap=0.12,
        name="service panel sits within the side-skin footprint",
    )

    rest_weapon_pos = ctx.part_world_position(weapon_mount)
    with ctx.pose({pan: 0.70}):
        panned_weapon_pos = ctx.part_world_position(weapon_mount)
    ctx.check(
        "pan rotation swings the weapon mount around the pedestal",
        rest_weapon_pos is not None
        and panned_weapon_pos is not None
        and panned_weapon_pos[1] > rest_weapon_pos[1] + 0.10,
        details=f"rest={rest_weapon_pos}, panned={panned_weapon_pos}",
    )

    rest_barrel_aabb = ctx.part_element_world_aabb(weapon_mount, elem="barrel_shroud")
    with ctx.pose({pitch: 0.75}):
        elevated_barrel_aabb = ctx.part_element_world_aabb(weapon_mount, elem="barrel_shroud")
    ctx.check(
        "weapon pitch raises the barrel",
        rest_barrel_aabb is not None
        and elevated_barrel_aabb is not None
        and elevated_barrel_aabb[1][2] > rest_barrel_aabb[1][2] + 0.18,
        details=f"rest={rest_barrel_aabb}, elevated={elevated_barrel_aabb}",
    )

    rest_panel_aabb = ctx.part_element_world_aabb(service_panel, elem="panel_plate")
    with ctx.pose({panel_hinge: 1.10}):
        open_panel_aabb = ctx.part_element_world_aabb(service_panel, elem="panel_plate")
    ctx.check(
        "service panel swings outward on its hinge",
        rest_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[0][1] < rest_panel_aabb[0][1] - 0.08,
        details=f"rest={rest_panel_aabb}, open={open_panel_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
