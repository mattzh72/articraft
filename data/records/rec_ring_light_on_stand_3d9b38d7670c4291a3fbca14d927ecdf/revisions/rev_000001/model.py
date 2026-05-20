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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def circle_profile(
    radius: float,
    *,
    segments: int = 64,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for i in range(segments):
        angle = (2.0 * math.pi * i) / segments
        if clockwise:
            angle = -angle
        points.append((radius * math.cos(angle), radius * math.sin(angle)))
    return points


def annulus_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    segments: int = 72,
):
    geom = ExtrudeWithHolesGeometry(
        circle_profile(outer_radius, segments=segments),
        [circle_profile(inner_radius, segments=segments, clockwise=True)],
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="salon_ring_light")

    powder_black = model.material("powder_black", rgba=(0.13, 0.13, 0.14, 1.0))
    satin_black = model.material("satin_black", rgba=(0.19, 0.19, 0.20, 1.0))
    soft_white = model.material("soft_white", rgba=(0.93, 0.94, 0.95, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.69, 0.73, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base_stand = model.part("base_stand")
    base_stand.visual(
        Cylinder(radius=0.053, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=powder_black,
        name="tripod_hub",
    )
    base_stand.visual(
        Cylinder(radius=0.030, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
        material=powder_black,
        name="lower_socket",
    )
    base_stand.visual(
        annulus_mesh(
            "outer_sleeve_mesh",
            outer_radius=0.019,
            inner_radius=0.0162,
            thickness=0.760,
            segments=84,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.540)),
        material=satin_black,
        name="outer_sleeve",
    )
    base_stand.visual(
        annulus_mesh(
            "stand_collar_mesh",
            outer_radius=0.032,
            inner_radius=0.0185,
            thickness=0.082,
            segments=84,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.959)),
        material=powder_black,
        name="stand_collar",
    )
    base_stand.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(
            xyz=(-0.040, 0.0, 0.958),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brushed_steel,
        name="clamp_stem",
    )
    base_stand.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(-0.058, 0.0, 0.958)),
        material=rubber,
        name="clamp_knob",
    )
    base_stand.visual(
        Box((0.012, 0.030, 0.042)),
        origin=Origin(xyz=(0.036, 0.0, 0.958)),
        material=powder_black,
        name="arm_mount_block",
    )
    base_stand.visual(
        Box((0.010, 0.006, 0.040)),
        origin=Origin(xyz=(0.045, -0.011, 0.958)),
        material=powder_black,
        name="arm_mount_lug_lower",
    )
    base_stand.visual(
        Box((0.010, 0.006, 0.040)),
        origin=Origin(xyz=(0.045, 0.011, 0.958)),
        material=powder_black,
        name="arm_mount_lug_upper",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_geom = tube_from_spline_points(
            [
                (0.030 * c, 0.030 * s, 0.114),
                (0.195 * c, 0.195 * s, 0.080),
                (0.425 * c, 0.425 * s, 0.018),
            ],
            radius=0.010,
            samples_per_segment=20,
            radial_segments=18,
            cap_ends=True,
        )
        base_stand.visual(
            mesh_from_geometry(leg_geom, f"tripod_leg_{index}"),
            material=powder_black,
            name=f"leg_{index}",
        )
        base_stand.visual(
            Sphere(radius=0.013),
            origin=Origin(xyz=(0.425 * c, 0.425 * s, 0.018)),
            material=rubber,
            name=f"foot_{index}",
        )

    base_stand.inertial = Inertial.from_geometry(
        Box((0.88, 0.88, 1.02)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.510)),
    )

    upper_column = model.part("upper_column")
    upper_column.visual(
        Cylinder(radius=0.0145, length=0.900),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=brushed_steel,
        name="inner_column",
    )
    upper_column.visual(
        annulus_mesh(
            "column_stop_collar_mesh",
            outer_radius=0.028,
            inner_radius=0.0140,
            thickness=0.010,
            segments=72,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=powder_black,
        name="stop_collar",
    )
    upper_column.visual(
        Cylinder(radius=0.021, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
        material=powder_black,
        name="yoke_receiver",
    )
    upper_column.visual(
        Box((0.110, 0.042, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        material=powder_black,
        name="yoke_block",
    )

    left_arm = tube_from_spline_points(
        [
            (0.0, 0.0, 0.395),
            (-0.110, 0.0, 0.535),
            (-0.215, 0.0, 0.585),
            (-0.270, 0.0, 0.660),
        ],
        radius=0.009,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    right_arm = tube_from_spline_points(
        [
            (0.0, 0.0, 0.395),
            (0.110, 0.0, 0.535),
            (0.215, 0.0, 0.585),
            (0.270, 0.0, 0.660),
        ],
        radius=0.009,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    upper_column.visual(
        mesh_from_geometry(left_arm, "left_yoke_arm"),
        material=powder_black,
        name="left_yoke_arm",
    )
    upper_column.visual(
        mesh_from_geometry(right_arm, "right_yoke_arm"),
        material=powder_black,
        name="right_yoke_arm",
    )
    upper_column.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(
            xyz=(-0.257, 0.0, 0.660),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=powder_black,
        name="left_yoke_tip",
    )
    upper_column.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(
            xyz=(0.257, 0.0, 0.660),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=powder_black,
        name="right_yoke_tip",
    )
    upper_column.inertial = Inertial.from_geometry(
        Box((0.56, 0.08, 1.02)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
    )

    model.articulation(
        "stand_height",
        ArticulationType.PRISMATIC,
        parent=base_stand,
        child=upper_column,
        origin=Origin(xyz=(0.0, 0.0, 0.920)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.18,
            lower=0.0,
            upper=0.240,
        ),
    )

    ring_head = model.part("ring_head")
    ring_head.visual(
        annulus_mesh(
            "ring_housing_mesh",
            outer_radius=0.225,
            inner_radius=0.145,
            thickness=0.036,
            segments=96,
        ),
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="head_housing",
    )
    ring_head.visual(
        annulus_mesh(
            "ring_diffuser_mesh",
            outer_radius=0.217,
            inner_radius=0.151,
            thickness=0.004,
            segments=96,
        ),
        origin=Origin(
            xyz=(0.0, 0.075, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=soft_white,
        name="front_diffuser",
    )
    ring_head.visual(
        Box((0.048, 0.022, 0.040)),
        origin=Origin(xyz=(0.108, 0.044, -0.120)),
        material=powder_black,
        name="driver_mount_spine",
    )
    ring_head.visual(
        Box((0.084, 0.020, 0.054)),
        origin=Origin(xyz=(0.118, 0.026, -0.158)),
        material=powder_black,
        name="rear_driver_box",
    )
    ring_head.visual(
        Box((0.032, 0.040, 0.070)),
        origin=Origin(xyz=(-0.208, 0.018, 0.0)),
        material=powder_black,
        name="left_trunnion_ear",
    )
    ring_head.visual(
        Cylinder(radius=0.015, length=0.023),
        origin=Origin(
            xyz=(-0.2355, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=powder_black,
        name="left_trunnion",
    )
    ring_head.visual(
        Box((0.032, 0.040, 0.070)),
        origin=Origin(xyz=(0.208, 0.018, 0.0)),
        material=powder_black,
        name="right_trunnion_ear",
    )
    ring_head.visual(
        Cylinder(radius=0.015, length=0.023),
        origin=Origin(
            xyz=(0.2355, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=powder_black,
        name="right_trunnion",
    )
    ring_head.inertial = Inertial.from_geometry(
        Box((0.470, 0.070, 0.470)),
        mass=1.8,
        origin=Origin(),
    )

    model.articulation(
        "ring_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_column,
        child=ring_head,
        origin=Origin(xyz=(0.0, 0.0, 0.660)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-0.75,
            upper=0.75,
        ),
    )

    accessory_arm = model.part("accessory_arm")
    accessory_arm.visual(
        Cylinder(radius=0.007, length=0.036),
        origin=Origin(),
        material=powder_black,
        name="arm_hinge_barrel",
    )
    accessory_arm.visual(
        Box((0.022, 0.010, 0.024)),
        origin=Origin(xyz=(0.011, 0.0, -0.004)),
        material=powder_black,
        name="arm_gusset",
    )
    accessory_arm.visual(
        Cylinder(radius=0.010, length=0.175),
        origin=Origin(
            xyz=(0.0875, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_black,
        name="arm_tube",
    )
    accessory_arm.visual(
        Cylinder(radius=0.0065, length=0.070),
        origin=Origin(xyz=(0.175, 0.0, 0.030)),
        material=powder_black,
        name="device_post",
    )
    accessory_arm.visual(
        Box((0.080, 0.012, 0.138)),
        origin=Origin(xyz=(0.195, 0.0, 0.083)),
        material=soft_white,
        name="device_plate",
    )
    accessory_arm.inertial = Inertial.from_geometry(
        Box((0.285, 0.040, 0.150)),
        mass=0.35,
        origin=Origin(xyz=(0.140, 0.0, 0.050)),
    )

    model.articulation(
        "accessory_fold",
        ArticulationType.REVOLUTE,
        parent=base_stand,
        child=accessory_arm,
        origin=Origin(xyz=(0.060, 0.0, 0.958), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_stand = object_model.get_part("base_stand")
    upper_column = object_model.get_part("upper_column")
    ring_head = object_model.get_part("ring_head")
    accessory_arm = object_model.get_part("accessory_arm")

    stand_height = object_model.get_articulation("stand_height")
    ring_tilt = object_model.get_articulation("ring_tilt")
    accessory_fold = object_model.get_articulation("accessory_fold")

    ctx.expect_within(
        upper_column,
        base_stand,
        axes="xy",
        inner_elem="inner_column",
        outer_elem="outer_sleeve",
        margin=0.003,
        name="upper column stays centered in the outer sleeve",
    )
    ctx.expect_overlap(
        upper_column,
        base_stand,
        axes="z",
        elem_a="inner_column",
        elem_b="outer_sleeve",
        min_overlap=0.120,
        name="inner column retains insertion in the sleeve at rest",
    )
    ctx.expect_contact(
        upper_column,
        ring_head,
        elem_a="left_yoke_tip",
        elem_b="left_trunnion",
        name="left trunnion seats in the yoke tip",
    )
    ctx.expect_contact(
        upper_column,
        ring_head,
        elem_a="right_yoke_tip",
        elem_b="right_trunnion",
        name="right trunnion seats in the yoke tip",
    )

    rest_upper_pos = ctx.part_world_position(upper_column)
    with ctx.pose({stand_height: 0.240}):
        ctx.expect_within(
            upper_column,
            base_stand,
            axes="xy",
            inner_elem="inner_column",
            outer_elem="outer_sleeve",
            margin=0.003,
            name="extended column stays centered in the sleeve",
        )
        ctx.expect_overlap(
            upper_column,
            base_stand,
            axes="z",
            elem_a="inner_column",
            elem_b="outer_sleeve",
            min_overlap=0.050,
            name="extended column keeps retained insertion",
        )
        extended_upper_pos = ctx.part_world_position(upper_column)
    ctx.check(
        "stand height motion lifts the upper column",
        rest_upper_pos is not None
        and extended_upper_pos is not None
        and extended_upper_pos[2] > rest_upper_pos[2] + 0.20,
        details=f"rest={rest_upper_pos}, extended={extended_upper_pos}",
    )

    rest_ring = ctx.part_element_world_aabb(ring_head, elem="front_diffuser")
    with ctx.pose({ring_tilt: 0.60}):
        tilted_ring = ctx.part_element_world_aabb(ring_head, elem="front_diffuser")
    ring_rest_dy = None if rest_ring is None else rest_ring[1][1] - rest_ring[0][1]
    ring_tilt_dy = None if tilted_ring is None else tilted_ring[1][1] - tilted_ring[0][1]
    ctx.check(
        "ring tilt increases front-back projection",
        ring_rest_dy is not None
        and ring_tilt_dy is not None
        and ring_tilt_dy > ring_rest_dy + 0.09,
        details=f"rest_dy={ring_rest_dy}, tilted_dy={ring_tilt_dy}",
    )

    rest_plate = ctx.part_element_world_aabb(accessory_arm, elem="device_plate")
    with ctx.pose({accessory_fold: 1.05}):
        open_plate = ctx.part_element_world_aabb(accessory_arm, elem="device_plate")
    rest_plate_center = (
        None
        if rest_plate is None
        else (
            0.5 * (rest_plate[0][0] + rest_plate[1][0]),
            0.5 * (rest_plate[0][1] + rest_plate[1][1]),
            0.5 * (rest_plate[0][2] + rest_plate[1][2]),
        )
    )
    open_plate_center = (
        None
        if open_plate is None
        else (
            0.5 * (open_plate[0][0] + open_plate[1][0]),
            0.5 * (open_plate[0][1] + open_plate[1][1]),
            0.5 * (open_plate[0][2] + open_plate[1][2]),
        )
    )
    ctx.check(
        "accessory arm swings out from the collar side",
        rest_plate_center is not None
        and open_plate_center is not None
        and open_plate_center[0] > rest_plate_center[0] + 0.10
        and abs(open_plate_center[1]) < abs(rest_plate_center[1]),
        details=f"rest={rest_plate_center}, open={open_plate_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
