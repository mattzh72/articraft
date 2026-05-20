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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_itx_cube_case")

    outer_w = 0.305
    outer_d = 0.315
    outer_h = 0.325
    sheet_t = 0.003
    bottom_t = 0.004
    lid_t = 0.004
    door_t = 0.005
    hinge_r = 0.0045
    foot_r = 0.012
    foot_h = 0.010
    door_base_z = 0.010
    door_h = 0.302

    powder_coat = model.material("powder_coat", rgba=(0.15, 0.16, 0.18, 1.0))
    inner_frame = model.material("inner_frame", rgba=(0.09, 0.10, 0.11, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.18, 0.22, 0.24, 0.42))
    brushed_metal = model.material("brushed_metal", rgba=(0.62, 0.64, 0.67, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((outer_w, outer_d, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t * 0.5)),
        material=inner_frame,
        name="bottom_tray",
    )
    chassis.visual(
        Box((sheet_t, outer_d, outer_h - bottom_t)),
        origin=Origin(xyz=(-(outer_w * 0.5) + (sheet_t * 0.5), 0.0, (outer_h + bottom_t) * 0.5)),
        material=powder_coat,
        name="left_wall",
    )
    chassis.visual(
        Box((sheet_t, outer_d, outer_h - bottom_t)),
        origin=Origin(xyz=((outer_w * 0.5) - (sheet_t * 0.5), 0.0, (outer_h + bottom_t) * 0.5)),
        material=powder_coat,
        name="right_wall",
    )
    chassis.visual(
        Box((outer_w - (2.0 * sheet_t), sheet_t, outer_h - bottom_t)),
        origin=Origin(
            xyz=(0.0, -(outer_d * 0.5) + (sheet_t * 0.5), (outer_h + bottom_t) * 0.5)
        ),
        material=powder_coat,
        name="rear_panel",
    )
    chassis.visual(
        Box((outer_w - (2.0 * sheet_t), sheet_t, 0.014)),
        origin=Origin(xyz=(0.0, (outer_d * 0.5) - (sheet_t * 0.5), outer_h - 0.007)),
        material=powder_coat,
        name="front_top_lip",
    )
    chassis.visual(
        Box((outer_w - (2.0 * sheet_t), sheet_t, 0.022)),
        origin=Origin(xyz=(0.0, (outer_d * 0.5) - (sheet_t * 0.5), 0.011)),
        material=powder_coat,
        name="front_bottom_sill",
    )
    chassis.visual(
        Box((outer_w - 0.040, 0.009, 0.009)),
        origin=Origin(xyz=(0.0, -(outer_d * 0.5) - 0.0045, outer_h - 0.0045)),
        material=brushed_metal,
        name="top_hinge_mount",
    )
    chassis.visual(
        Box((0.009, 0.010, door_h - 0.040)),
        origin=Origin(
            xyz=(
                -(outer_w * 0.5) - 0.0045,
                (outer_d * 0.5) - 0.005,
                door_base_z + (door_h * 0.5),
            )
        ),
        material=brushed_metal,
        name="door_hinge_mount",
    )

    foot_positions = (
        (-(outer_w * 0.5) + 0.045, -(outer_d * 0.5) + 0.045),
        ((outer_w * 0.5) - 0.045, -(outer_d * 0.5) + 0.045),
        (-(outer_w * 0.5) + 0.045, (outer_d * 0.5) - 0.045),
        ((outer_w * 0.5) - 0.045, (outer_d * 0.5) - 0.045),
    )
    for foot_index, (foot_x, foot_y) in enumerate(foot_positions):
        chassis.visual(
            Cylinder(radius=foot_r, length=foot_h),
            origin=Origin(xyz=(foot_x, foot_y, -(foot_h * 0.5))),
            material=rubber,
            name=f"foot_{foot_index}",
        )

    chassis.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, outer_h + foot_h)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, (outer_h - foot_h) * 0.5)),
    )

    top_panel = model.part("top_panel")
    top_panel.visual(
        Box((outer_w, outer_d, lid_t)),
        origin=Origin(
            xyz=(
                0.0,
                (outer_d * 0.5) + hinge_r,
                (lid_t * 0.5) - hinge_r,
            )
        ),
        material=powder_coat,
        name="top_skin",
    )
    top_panel.visual(
        Box((0.016, outer_d - 0.012, 0.018)),
        origin=Origin(
            xyz=(
                -(outer_w * 0.5) + 0.008,
                (outer_d * 0.5) + hinge_r + 0.006,
                -0.007,
            )
        ),
        material=inner_frame,
        name="top_left_flange",
    )
    top_panel.visual(
        Box((0.016, outer_d - 0.012, 0.018)),
        origin=Origin(
            xyz=(
                (outer_w * 0.5) - 0.008,
                (outer_d * 0.5) + hinge_r + 0.006,
                -0.007,
            )
        ),
        material=inner_frame,
        name="top_right_flange",
    )
    top_panel.visual(
        Box((outer_w - 0.050, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.006, -0.006)),
        material=inner_frame,
        name="top_rear_flange",
    )
    top_panel.visual(
        Box((outer_w - 0.030, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, outer_d + hinge_r - 0.006, -0.007)),
        material=inner_frame,
        name="top_front_return",
    )
    for rib_index, rib_x in enumerate((-0.080, -0.040, 0.000, 0.040, 0.080)):
        top_panel.visual(
            Box((0.010, outer_d - 0.090, 0.003)),
            origin=Origin(xyz=(rib_x, (outer_d * 0.5) + hinge_r + 0.020, 0.0005)),
            material=brushed_metal,
            name=f"top_rib_{rib_index}",
        )
    for barrel_name, barrel_x, barrel_len in (
        ("top_hinge_barrel_left", -0.095, 0.070),
        ("top_hinge_barrel_center", 0.000, 0.050),
        ("top_hinge_barrel_right", 0.095, 0.070),
    ):
        top_panel.visual(
            Cylinder(radius=hinge_r, length=barrel_len),
            origin=Origin(xyz=(barrel_x, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
            material=brushed_metal,
            name=barrel_name,
        )
    top_panel.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, 0.022)),
        mass=1.3,
        origin=Origin(xyz=(0.0, (outer_d * 0.5) + hinge_r, 0.004)),
    )

    front_door = model.part("front_door")
    front_door.visual(
        Box((outer_w, door_t, door_h)),
        origin=Origin(
            xyz=(
                (outer_w * 0.5) + hinge_r,
                (door_t * 0.5) - hinge_r,
                door_h * 0.5,
            )
        ),
        material=powder_coat,
        name="door_leaf",
    )
    front_door.visual(
        Box((outer_w - 0.060, 0.003, door_h - 0.080)),
        origin=Origin(
            xyz=(
                (outer_w * 0.5) + hinge_r,
                -0.0005,
                (door_h * 0.5) + 0.002,
            )
        ),
        material=smoked_glass,
        name="door_glass",
    )
    front_door.visual(
        Box((0.012, 0.014, 0.090)),
        origin=Origin(
            xyz=(
                outer_w - 0.022,
                0.004,
                0.165,
            )
        ),
        material=brushed_metal,
        name="door_handle",
    )
    for barrel_name, barrel_z, barrel_len in (
        ("door_hinge_barrel_bottom", 0.050, 0.070),
        ("door_hinge_barrel_mid", 0.151, 0.050),
        ("door_hinge_barrel_top", 0.252, 0.070),
    ):
        front_door.visual(
            Cylinder(radius=hinge_r, length=barrel_len),
            origin=Origin(xyz=(0.0, 0.0, barrel_z)),
            material=brushed_metal,
            name=barrel_name,
        )
    front_door.inertial = Inertial.from_geometry(
        Box((outer_w, door_t + 0.012, door_h)),
        mass=1.6,
        origin=Origin(xyz=((outer_w * 0.5) + hinge_r, 0.001, door_h * 0.5)),
    )

    model.articulation(
        "chassis_to_top_panel",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=top_panel,
        origin=Origin(xyz=(0.0, -(outer_d * 0.5) - hinge_r, outer_h + hinge_r)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.75),
    )
    model.articulation(
        "chassis_to_front_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_door,
        origin=Origin(xyz=(-(outer_w * 0.5) - hinge_r, (outer_d * 0.5) + hinge_r, door_base_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.8, lower=0.0, upper=2.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    top_panel = object_model.get_part("top_panel")
    front_door = object_model.get_part("front_door")
    top_hinge = object_model.get_articulation("chassis_to_top_panel")
    door_hinge = object_model.get_articulation("chassis_to_front_door")

    with ctx.pose({top_hinge: 0.0, door_hinge: 0.0}):
        ctx.expect_gap(
            top_panel,
            chassis,
            axis="z",
            positive_elem="top_skin",
            max_gap=0.0005,
            max_penetration=0.0,
            name="top panel closes flush to the chassis roof line",
        )
        ctx.expect_overlap(
            top_panel,
            chassis,
            axes="xy",
            elem_a="top_skin",
            min_overlap=0.280,
            name="top panel covers the compact cube body in plan",
        )
        ctx.expect_gap(
            front_door,
            chassis,
            axis="y",
            positive_elem="door_leaf",
            max_gap=0.0005,
            max_penetration=0.0,
            name="front door sits flush with the front face",
        )
        ctx.expect_overlap(
            front_door,
            chassis,
            axes="xz",
            elem_a="door_leaf",
            min_overlap=0.260,
            name="front door spans the front opening",
        )

        closed_top = ctx.part_element_world_aabb(top_panel, elem="top_skin")
        closed_door = ctx.part_element_world_aabb(front_door, elem="door_leaf")

    with ctx.pose({top_hinge: 1.30}):
        open_top = ctx.part_element_world_aabb(top_panel, elem="top_skin")
        ctx.check(
            "top panel opens upward from the rear hinge",
            closed_top is not None
            and open_top is not None
            and open_top[1][2] > closed_top[1][2] + 0.12,
            details=f"closed={closed_top}, open={open_top}",
        )

    with ctx.pose({door_hinge: 1.45}):
        open_door = ctx.part_element_world_aabb(front_door, elem="door_leaf")
        ctx.check(
            "front door swings outward on its vertical edge",
            closed_door is not None
            and open_door is not None
            and open_door[1][1] > closed_door[1][1] + 0.12,
            details=f"closed={closed_door}, open={open_door}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
