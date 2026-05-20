from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_office_router")

    shell_black = model.material("shell_black", rgba=(0.14, 0.15, 0.16, 1.0))
    trim_black = model.material("trim_black", rgba=(0.09, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))
    smoked_cover = model.material("smoked_cover", rgba=(0.16, 0.20, 0.22, 0.72))
    smoked_lens = model.material("smoked_lens", rgba=(0.12, 0.15, 0.17, 0.55))
    led_green = model.material("led_green", rgba=(0.20, 0.86, 0.38, 1.0))
    led_amber = model.material("led_amber", rgba=(0.93, 0.72, 0.20, 1.0))
    led_blue = model.material("led_blue", rgba=(0.25, 0.58, 0.95, 1.0))

    body_width = 0.300
    body_depth = 0.180
    foot_height = 0.006
    body_height = 0.036
    body_top_z = foot_height + body_height

    def shift_profile(profile, dx=0.0, dy=0.0):
        return [(x + dx, y + dy) for x, y in profile]

    shell_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(body_width, body_depth, radius=0.012, corner_segments=8),
            body_height,
        ),
        "router_shell",
    )

    vent_outer = rounded_rect_profile(0.148, 0.072, radius=0.004, corner_segments=6)
    vent_slot = rounded_rect_profile(0.012, 0.056, radius=0.0010, corner_segments=4)
    vent_holes = [
        shift_profile(vent_slot, dx=offset_x)
        for offset_x in (-0.045, -0.027, -0.009, 0.009, 0.027, 0.045)
    ]
    vent_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            vent_outer,
            vent_holes,
            height=0.0012,
            center=True,
        ),
        "router_top_vent",
    )

    antenna_mesh = mesh_from_geometry(
        CapsuleGeometry(radius=0.0042, length=0.072, radial_segments=20, height_segments=10),
        "router_antenna_stub",
    )

    chassis = model.part("chassis")
    chassis.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, foot_height)),
        material=shell_black,
        name="shell",
    )
    chassis.visual(
        Box((0.252, 0.0030, 0.026)),
        origin=Origin(xyz=(0.0, -0.0915, 0.020)),
        material=trim_black,
        name="front_fascia",
    )
    chassis.visual(
        Box((0.152, 0.0014, 0.013)),
        origin=Origin(xyz=(0.0, -0.0908, 0.021)),
        material=smoked_lens,
        name="status_lens",
    )
    led_offsets = (-0.050, -0.026, 0.0, 0.026, 0.050)
    led_materials = (led_green, led_amber, led_blue, led_green, led_green)
    for index, (offset_x, led_material) in enumerate(zip(led_offsets, led_materials)):
        chassis.visual(
            Box((0.010, 0.0008, 0.0030)),
            origin=Origin(xyz=(offset_x, -0.0919, 0.021)),
            material=led_material,
            name=f"status_led_{index}",
        )
    chassis.visual(
        Box((0.156, 0.0062, 0.0044)),
        origin=Origin(xyz=(0.0, -0.0937, 0.0022)),
        material=trim_black,
        name="door_sill",
    )
    chassis.visual(
        Box((0.162, 0.0014, 0.0030)),
        origin=Origin(xyz=(0.0, -0.0961, 0.0125)),
        material=trim_black,
        name="door_clip_front",
    )
    chassis.visual(
        Box((0.146, 0.0010, 0.0030)),
        origin=Origin(xyz=(0.0, -0.09235, 0.0125)),
        material=trim_black,
        name="door_clip_back",
    )
    for side in (-1.0, 1.0):
        chassis.visual(
            Box((0.008, 0.0049, 0.0140)),
            origin=Origin(xyz=(side * 0.081, -0.09435, 0.0070)),
            material=trim_black,
            name=f"door_clip_side_{'left' if side < 0.0 else 'right'}",
        )
    chassis.visual(
        vent_mesh,
        origin=Origin(xyz=(0.0, 0.006, body_top_z + 0.0006)),
        material=trim_black,
        name="top_vent",
    )
    chassis.visual(
        Box((0.190, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.086, 0.017)),
        material=trim_black,
        name="rear_io_strip",
    )
    for index, x_center in enumerate((-0.056, -0.026, 0.0, 0.026, 0.056)):
        chassis.visual(
            Box((0.018, 0.004, 0.006)),
            origin=Origin(xyz=(x_center, 0.089, 0.017)),
            material=shell_black,
            name=f"rear_port_{index}",
        )
    for foot_name, foot_x, foot_y in (
        ("front_left_foot", -0.116, -0.060),
        ("front_right_foot", 0.116, -0.060),
        ("rear_left_foot", -0.116, 0.060),
        ("rear_right_foot", 0.116, 0.060),
    ):
        chassis.visual(
            Cylinder(radius=0.008, length=foot_height),
            origin=Origin(xyz=(foot_x, foot_y, foot_height / 2.0)),
            material=rubber_black,
            name=foot_name,
        )
    chassis.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height + foot_height)),
        mass=1.45,
        origin=Origin(xyz=(0.0, 0.0, (body_height + foot_height) / 2.0)),
    )

    status_cover = model.part("status_cover")
    status_cover.visual(
        Cylinder(radius=0.00155, length=0.146),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_black,
        name="hinge_barrel",
    )
    status_cover.visual(
        Box((0.148, 0.0022, 0.0186)),
        origin=Origin(xyz=(0.0, 0.0002, 0.0105)),
        material=smoked_cover,
        name="cover_panel",
    )
    status_cover.visual(
        Box((0.120, 0.0030, 0.0034)),
        origin=Origin(xyz=(0.0, -0.0008, 0.0187)),
        material=trim_black,
        name="finger_lip",
    )
    status_cover.inertial = Inertial.from_geometry(
        Box((0.148, 0.0035, 0.0205)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0001, 0.0105)),
    )

    model.articulation(
        "status_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=status_cover,
        origin=Origin(xyz=(0.0, -0.0944, 0.0125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=0.0,
            upper=1.28,
        ),
    )

    def make_antenna_base(name: str):
        base = model.part(name)
        base.visual(
            Box((0.020, 0.014, 0.005)),
            origin=Origin(xyz=(0.0, 0.0, 0.0025)),
            material=trim_black,
            name="mount_pad",
        )
        base.visual(
            Box((0.016, 0.0012, 0.0038)),
            origin=Origin(xyz=(0.0, -0.0022, 0.0068)),
            material=trim_black,
            name="clip_front",
        )
        base.visual(
            Box((0.016, 0.0012, 0.0038)),
            origin=Origin(xyz=(0.0, 0.0022, 0.0068)),
            material=trim_black,
            name="clip_back",
        )
        base.inertial = Inertial.from_geometry(
            Box((0.020, 0.014, 0.009)),
            mass=0.025,
            origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        )
        return base

    def make_antenna(name: str):
        antenna = model.part(name)
        antenna.visual(
            Cylinder(radius=0.0016, length=0.012),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=trim_black,
            name="hinge_barrel",
        )
        antenna.visual(
            Box((0.008, 0.0026, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=trim_black,
            name="neck",
        )
        antenna.visual(
            antenna_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.047)),
            material=rubber_black,
            name="stub",
        )
        antenna.inertial = Inertial.from_geometry(
            Box((0.012, 0.009, 0.086)),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.0, 0.043)),
        )
        return antenna

    antenna_base_left = make_antenna_base("antenna_base_left")
    antenna_base_right = make_antenna_base("antenna_base_right")
    antenna_left = make_antenna("antenna_left")
    antenna_right = make_antenna("antenna_right")

    model.articulation(
        "chassis_to_antenna_base_left",
        ArticulationType.FIXED,
        parent=chassis,
        child=antenna_base_left,
        origin=Origin(xyz=(-0.118, 0.062, body_top_z)),
    )
    model.articulation(
        "chassis_to_antenna_base_right",
        ArticulationType.FIXED,
        parent=chassis,
        child=antenna_base_right,
        origin=Origin(xyz=(0.118, 0.062, body_top_z)),
    )

    antenna_limits = MotionLimits(
        effort=1.0,
        velocity=2.5,
        lower=-1.25,
        upper=0.20,
    )
    model.articulation(
        "antenna_left_hinge",
        ArticulationType.REVOLUTE,
        parent=antenna_base_left,
        child=antenna_left,
        origin=Origin(xyz=(0.0, 0.0, 0.0068)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=antenna_limits,
    )
    model.articulation(
        "antenna_right_hinge",
        ArticulationType.REVOLUTE,
        parent=antenna_base_right,
        child=antenna_right,
        origin=Origin(xyz=(0.0, 0.0, 0.0068)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=-1.25,
            upper=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    chassis = object_model.get_part("chassis")
    status_cover = object_model.get_part("status_cover")
    antenna_base_left = object_model.get_part("antenna_base_left")
    antenna_base_right = object_model.get_part("antenna_base_right")
    antenna_left = object_model.get_part("antenna_left")
    antenna_right = object_model.get_part("antenna_right")

    status_cover_hinge = object_model.get_articulation("status_cover_hinge")
    antenna_left_hinge = object_model.get_articulation("antenna_left_hinge")
    antenna_right_hinge = object_model.get_articulation("antenna_right_hinge")

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "status_cover_hinge_axis_is_x",
        tuple(status_cover_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={status_cover_hinge.axis}",
    )
    ctx.check(
        "antenna_left_hinge_axis_is_x",
        tuple(antenna_left_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={antenna_left_hinge.axis}",
    )
    ctx.check(
        "antenna_right_hinge_axis_is_x",
        tuple(antenna_right_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={antenna_right_hinge.axis}",
    )

    ctx.expect_contact(status_cover, chassis, contact_tol=5e-4)
    ctx.expect_gap(
        status_cover,
        chassis,
        axis="y",
        min_gap=-0.006,
        max_gap=0.0,
        positive_elem="cover_panel",
        negative_elem="status_lens",
        name="status_cover_sits_immediately_in_front_of_lens",
    )
    ctx.expect_overlap(
        status_cover,
        chassis,
        axes="x",
        min_overlap=0.11,
        elem_a="cover_panel",
        elem_b="status_lens",
        name="status_cover_spans_indicator_strip_width",
    )
    ctx.expect_overlap(
        status_cover,
        chassis,
        axes="z",
        min_overlap=0.010,
        elem_a="cover_panel",
        elem_b="status_lens",
        name="status_cover_spans_indicator_strip_height",
    )
    ctx.expect_contact(antenna_base_left, chassis, contact_tol=5e-4)
    ctx.expect_contact(antenna_base_right, chassis, contact_tol=5e-4)
    ctx.expect_contact(antenna_left, antenna_base_left, contact_tol=5e-4)
    ctx.expect_contact(antenna_right, antenna_base_right, contact_tol=5e-4)

    door_rest = ctx.part_world_aabb(status_cover)
    left_rest = ctx.part_world_aabb(antenna_left)
    right_rest = ctx.part_world_aabb(antenna_right)
    ctx.check(
        "status_cover_rest_aabb_available",
        door_rest is not None,
        details="status cover AABB unavailable in rest pose",
    )
    ctx.check(
        "antenna_left_rest_aabb_available",
        left_rest is not None,
        details="left antenna AABB unavailable in rest pose",
    )
    ctx.check(
        "antenna_right_rest_aabb_available",
        right_rest is not None,
        details="right antenna AABB unavailable in rest pose",
    )

    if door_rest is not None:
        with ctx.pose({status_cover_hinge: 1.10}):
            door_open = ctx.part_world_aabb(status_cover)
            ctx.check(
                "status_cover_open_aabb_available",
                door_open is not None,
                details="status cover AABB unavailable in open pose",
            )
            if door_open is not None:
                ctx.check(
                    "status_cover_swings_forward",
                    door_open[0][1] < door_rest[0][1] - 0.006,
                    details=f"rest_min_y={door_rest[0][1]:.6f}, open_min_y={door_open[0][1]:.6f}",
                )
                ctx.check(
                    "status_cover_top_edge_drops_when_open",
                    door_open[1][2] < door_rest[1][2] - 0.008,
                    details=f"rest_max_z={door_rest[1][2]:.6f}, open_max_z={door_open[1][2]:.6f}",
                )
                ctx.check(
                    "status_cover_lower_hinge_line_stays_clipped",
                    abs(door_open[0][2] - door_rest[0][2]) <= 0.002,
                    details=f"rest_min_z={door_rest[0][2]:.6f}, open_min_z={door_open[0][2]:.6f}",
                )
            ctx.expect_contact(status_cover, chassis, contact_tol=5e-4)

    if left_rest is not None and right_rest is not None:
        with ctx.pose({antenna_left_hinge: -1.10, antenna_right_hinge: -1.10}):
            left_folded = ctx.part_world_aabb(antenna_left)
            right_folded = ctx.part_world_aabb(antenna_right)
            ctx.check(
                "antenna_left_folded_aabb_available",
                left_folded is not None,
                details="left antenna AABB unavailable in folded pose",
            )
            ctx.check(
                "antenna_right_folded_aabb_available",
                right_folded is not None,
                details="right antenna AABB unavailable in folded pose",
            )
            if left_folded is not None:
                ctx.check(
                    "antenna_left_folds_back",
                    left_folded[1][2] < left_rest[1][2] - 0.030 and left_folded[1][1] > left_rest[1][1] + 0.030,
                    details=(
                        f"rest_max_y={left_rest[1][1]:.6f}, folded_max_y={left_folded[1][1]:.6f}; "
                        f"rest_max_z={left_rest[1][2]:.6f}, folded_max_z={left_folded[1][2]:.6f}"
                    ),
                )
            if right_folded is not None:
                ctx.check(
                    "antenna_right_folds_back",
                    right_folded[1][2] < right_rest[1][2] - 0.030 and right_folded[1][1] > right_rest[1][1] + 0.030,
                    details=(
                        f"rest_max_y={right_rest[1][1]:.6f}, folded_max_y={right_folded[1][1]:.6f}; "
                        f"rest_max_z={right_rest[1][2]:.6f}, folded_max_z={right_folded[1][2]:.6f}"
                    ),
                )
            ctx.expect_contact(antenna_left, antenna_base_left, contact_tol=5e-4)
            ctx.expect_contact(antenna_right, antenna_base_right, contact_tol=5e-4)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
