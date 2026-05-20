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
    model = ArticulatedObject(name="compact_missile_launcher")

    base_gray = model.material("base_gray", rgba=(0.35, 0.37, 0.39, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    launcher_green = model.material("launcher_green", rgba=(0.31, 0.36, 0.25, 1.0))
    launcher_green_dark = model.material("launcher_green_dark", rgba=(0.24, 0.28, 0.18, 1.0))
    interior_black = model.material("interior_black", rgba=(0.08, 0.09, 0.10, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Box((1.05, 0.90, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=base_gray,
        name="base_sled",
    )
    pedestal_base.visual(
        Box((0.84, 0.18, 0.12)),
        origin=Origin(xyz=(0.10, 0.0, 0.16)),
        material=base_gray,
        name="upper_plinth",
    )
    pedestal_base.visual(
        Cylinder(radius=0.18, length=0.74),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=steel_dark,
        name="pedestal_column",
    )
    pedestal_base.visual(
        Cylinder(radius=0.26, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
        material=steel_dark,
        name="yaw_bearing_housing",
    )
    pedestal_base.visual(
        Box((0.38, 0.34, 0.22)),
        origin=Origin(xyz=(-0.18, 0.0, 0.22)),
        material=base_gray,
        name="service_cabinet",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((1.05, 0.90, 0.96)),
        mass=620.0,
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
    )

    elevating_frame = model.part("elevating_frame")
    elevating_frame.visual(
        Cylinder(radius=0.28, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=steel_dark,
        name="turntable_ring",
    )
    elevating_frame.visual(
        Box((0.50, 0.40, 0.08)),
        origin=Origin(xyz=(0.05, 0.0, 0.10)),
        material=base_gray,
        name="rotating_deck",
    )
    elevating_frame.visual(
        Box((0.18, 0.24, 0.22)),
        origin=Origin(xyz=(-0.18, 0.0, 0.20)),
        material=steel_dark,
        name="elevation_drive_block",
    )
    elevating_frame.visual(
        Box((0.28, 0.18, 0.16)),
        origin=Origin(xyz=(-0.26, 0.0, 0.22)),
        material=base_gray,
        name="rear_machine_housing",
    )
    elevating_frame.visual(
        Box((0.12, 0.08, 0.54)),
        origin=Origin(xyz=(0.20, 0.34, 0.43)),
        material=launcher_green_dark,
        name="right_yoke_bracket",
    )
    elevating_frame.visual(
        Box((0.12, 0.08, 0.54)),
        origin=Origin(xyz=(0.20, -0.34, 0.43)),
        material=launcher_green_dark,
        name="left_yoke_bracket",
    )
    elevating_frame.visual(
        Box((0.10, 0.08, 0.16)),
        origin=Origin(xyz=(0.18, 0.34, 0.12)),
        material=launcher_green_dark,
        name="right_support_web",
    )
    elevating_frame.visual(
        Box((0.10, 0.08, 0.16)),
        origin=Origin(xyz=(0.18, -0.34, 0.12)),
        material=launcher_green_dark,
        name="left_support_web",
    )
    elevating_frame.visual(
        Box((0.16, 0.20, 0.10)),
        origin=Origin(xyz=(0.18, 0.25, 0.10)),
        material=launcher_green_dark,
        name="right_side_outrigger",
    )
    elevating_frame.visual(
        Box((0.16, 0.20, 0.10)),
        origin=Origin(xyz=(0.18, -0.25, 0.10)),
        material=launcher_green_dark,
        name="left_side_outrigger",
    )
    elevating_frame.visual(
        Box((0.12, 0.76, 0.06)),
        origin=Origin(xyz=(0.20, 0.0, 0.69)),
        material=launcher_green_dark,
        name="top_bridge",
    )
    elevating_frame.visual(
        Cylinder(radius=0.06, length=0.08),
        origin=Origin(xyz=(0.20, 0.34, 0.40), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="right_trunnion_bearing",
    )
    elevating_frame.visual(
        Cylinder(radius=0.06, length=0.08),
        origin=Origin(xyz=(0.20, -0.34, 0.40), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="left_trunnion_bearing",
    )
    elevating_frame.inertial = Inertial.from_geometry(
        Box((0.52, 0.76, 0.58)),
        mass=210.0,
        origin=Origin(xyz=(0.02, 0.0, 0.29)),
    )

    launcher_pack = model.part("launcher_pack")
    launcher_pack.visual(
        Box((0.08, 0.50, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel_dark,
        name="trunnion_spine",
    )
    launcher_pack.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=Origin(xyz=(0.0, 0.26, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="right_trunnion_stub",
    )
    launcher_pack.visual(
        Cylinder(radius=0.055, length=0.08),
        origin=Origin(xyz=(0.0, -0.26, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="left_trunnion_stub",
    )
    launcher_pack.visual(
        Box((0.03, 0.56, 0.40)),
        origin=Origin(xyz=(-0.245, 0.0, 0.0)),
        material=launcher_green_dark,
        name="rear_bulkhead",
    )
    launcher_pack.visual(
        Box((0.75, 0.56, 0.04)),
        origin=Origin(xyz=(0.11, 0.0, 0.20)),
        material=launcher_green,
        name="roof_panel",
    )
    launcher_pack.visual(
        Box((0.75, 0.56, 0.04)),
        origin=Origin(xyz=(0.11, 0.0, -0.20)),
        material=launcher_green,
        name="floor_panel",
    )
    launcher_pack.visual(
        Box((0.75, 0.04, 0.38)),
        origin=Origin(xyz=(0.11, -0.26, 0.0)),
        material=launcher_green,
        name="left_side_wall",
    )
    launcher_pack.visual(
        Box((0.44, 0.04, 0.38)),
        origin=Origin(xyz=(-0.04, 0.26, 0.0)),
        material=launcher_green,
        name="right_rear_wall",
    )
    launcher_pack.visual(
        Box((0.06, 0.04, 0.38)),
        origin=Origin(xyz=(0.46, 0.26, 0.0)),
        material=launcher_green,
        name="right_front_jamb",
    )
    launcher_pack.visual(
        Box((0.25, 0.04, 0.08)),
        origin=Origin(xyz=(0.305, 0.26, 0.18)),
        material=launcher_green,
        name="door_top_rail",
    )
    launcher_pack.visual(
        Box((0.25, 0.04, 0.08)),
        origin=Origin(xyz=(0.305, 0.26, -0.18)),
        material=launcher_green,
        name="door_bottom_rail",
    )
    launcher_pack.visual(
        Box((0.03, 0.56, 0.06)),
        origin=Origin(xyz=(0.475, 0.0, 0.19)),
        material=launcher_green,
        name="front_top_beam",
    )
    launcher_pack.visual(
        Box((0.03, 0.56, 0.06)),
        origin=Origin(xyz=(0.475, 0.0, -0.19)),
        material=launcher_green,
        name="front_bottom_beam",
    )
    launcher_pack.visual(
        Box((0.03, 0.04, 0.36)),
        origin=Origin(xyz=(0.475, -0.26, 0.0)),
        material=launcher_green,
        name="front_left_stile",
    )
    launcher_pack.visual(
        Box((0.03, 0.04, 0.36)),
        origin=Origin(xyz=(0.475, 0.26, 0.0)),
        material=launcher_green,
        name="front_right_stile",
    )
    launcher_pack.visual(
        Box((0.03, 0.04, 0.36)),
        origin=Origin(xyz=(0.475, 0.0, 0.0)),
        material=launcher_green_dark,
        name="front_vertical_divider",
    )
    launcher_pack.visual(
        Box((0.03, 0.56, 0.04)),
        origin=Origin(xyz=(0.475, 0.0, 0.0)),
        material=launcher_green_dark,
        name="front_horizontal_divider",
    )
    for row_index, z_center in enumerate((0.11, -0.11)):
        for col_index, y_center in enumerate((-0.12, 0.12)):
            launcher_pack.visual(
                Box((0.72, 0.18, 0.18)),
                origin=Origin(xyz=(0.10, y_center, z_center)),
                material=launcher_green_dark,
                name=f"cell_body_{row_index}_{col_index}",
            )
            launcher_pack.visual(
                Box((0.02, 0.15, 0.15)),
                origin=Origin(xyz=(0.44, y_center, z_center)),
                material=interior_black,
                name=f"cell_face_{row_index}_{col_index}",
            )
    launcher_pack.inertial = Inertial.from_geometry(
        Box((0.78, 0.58, 0.42)),
        mass=340.0,
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
    )

    access_door = model.part("access_door")
    access_door.visual(
        Box((0.25, 0.008, 0.30)),
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        material=launcher_green,
        name="door_skin",
    )
    access_door.visual(
        Box((0.18, 0.004, 0.05)),
        origin=Origin(xyz=(0.13, 0.006, 0.09)),
        material=launcher_green_dark,
        name="door_upper_stiffener",
    )
    access_door.visual(
        Box((0.18, 0.004, 0.05)),
        origin=Origin(xyz=(0.13, 0.006, -0.09)),
        material=launcher_green_dark,
        name="door_lower_stiffener",
    )
    access_door.visual(
        Cylinder(radius=0.010, length=0.08),
        origin=Origin(xyz=(0.0, 0.014, 0.09)),
        material=steel_dark,
        name="hinge_upper_knuckle",
    )
    access_door.visual(
        Cylinder(radius=0.010, length=0.08),
        origin=Origin(xyz=(0.0, 0.014, -0.09)),
        material=steel_dark,
        name="hinge_lower_knuckle",
    )
    access_door.visual(
        Box((0.035, 0.016, 0.09)),
        origin=Origin(xyz=(0.205, 0.012, 0.0)),
        material=steel_dark,
        name="door_pull",
    )
    access_door.inertial = Inertial.from_geometry(
        Box((0.25, 0.02, 0.30)),
        mass=18.0,
        origin=Origin(xyz=(0.125, 0.01, 0.0)),
    )

    model.articulation(
        "pedestal_yaw",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=elevating_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=0.9),
    )
    model.articulation(
        "frame_pitch",
        ArticulationType.REVOLUTE,
        parent=elevating_frame,
        child=launcher_pack,
        origin=Origin(xyz=(0.20, 0.0, 0.40)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4200.0, velocity=0.8, lower=-0.20, upper=1.05),
    )
    model.articulation(
        "pack_access_door",
        ArticulationType.REVOLUTE,
        parent=launcher_pack,
        child=access_door,
        origin=Origin(xyz=(0.18, 0.284, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=0.0, upper=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal_base = object_model.get_part("pedestal_base")
    elevating_frame = object_model.get_part("elevating_frame")
    launcher_pack = object_model.get_part("launcher_pack")
    access_door = object_model.get_part("access_door")

    yaw = object_model.get_articulation("pedestal_yaw")
    pitch = object_model.get_articulation("frame_pitch")
    door_hinge = object_model.get_articulation("pack_access_door")

    ctx.check(
        "pedestal yaw axis is vertical",
        yaw.axis == (0.0, 0.0, 1.0),
        details=f"axis={yaw.axis}",
    )
    ctx.check(
        "launcher pitch axis elevates upward",
        pitch.axis == (0.0, -1.0, 0.0),
        details=f"axis={pitch.axis}",
    )
    ctx.check(
        "side access door uses vertical hinge",
        door_hinge.axis == (0.0, 0.0, 1.0),
        details=f"axis={door_hinge.axis}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            access_door,
            launcher_pack,
            axis="y",
            positive_elem="door_skin",
            negative_elem="door_top_rail",
            max_gap=0.0025,
            max_penetration=0.0005,
            name="door closes flush with launcher side wall",
        )
        ctx.expect_overlap(
            access_door,
            launcher_pack,
            axes="xz",
            elem_a="door_skin",
            min_overlap=0.22,
            name="door covers the side access aperture",
        )

    rest_pack_pos = ctx.part_world_position(launcher_pack)
    with ctx.pose({yaw: math.pi / 2.0}):
        yawed_pack_pos = ctx.part_world_position(launcher_pack)
    ctx.check(
        "yaw rotates the launcher pack around the pedestal",
        rest_pack_pos is not None
        and yawed_pack_pos is not None
        and rest_pack_pos[0] > 0.05
        and abs(rest_pack_pos[1]) < 0.02
        and abs(yawed_pack_pos[0]) < 0.03
        and yawed_pack_pos[1] > 0.05,
        details=f"rest={rest_pack_pos}, yawed={yawed_pack_pos}",
    )

    rest_pack_aabb = ctx.part_world_aabb(launcher_pack)
    with ctx.pose({pitch: 0.80}):
        pitched_pack_aabb = ctx.part_world_aabb(launcher_pack)
    ctx.check(
        "pitch articulation raises the launcher nose",
        rest_pack_aabb is not None
        and pitched_pack_aabb is not None
        and pitched_pack_aabb[1][2] > rest_pack_aabb[1][2] + 0.14,
        details=f"rest={rest_pack_aabb}, pitched={pitched_pack_aabb}",
    )

    rest_door_aabb = ctx.part_world_aabb(access_door)
    with ctx.pose({door_hinge: 1.15}):
        open_door_aabb = ctx.part_world_aabb(access_door)
    ctx.check(
        "door swings outward from the launcher side",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > rest_door_aabb[1][1] + 0.12,
        details=f"rest={rest_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
