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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_glove_box")

    dash_dark = model.material("dash_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    tub_black = model.material("tub_black", rgba=(0.10, 0.11, 0.12, 1.0))
    arm_steel = model.material("arm_steel", rgba=(0.52, 0.55, 0.58, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    latch_black = model.material("latch_black", rgba=(0.08, 0.08, 0.09, 1.0))

    opening_w = 0.360
    opening_h = 0.172
    tub_depth = 0.220
    shell_t = 0.006
    bezel_t = 0.010
    pivot_spacing = 0.344
    arm_drop = (0.0, 0.022, -0.048)
    arm_length = math.sqrt(arm_drop[1] ** 2 + arm_drop[2] ** 2)
    support_y = -0.016
    support_z = -0.024
    left_support_x = -pivot_spacing / 2.0
    right_support_x = pivot_spacing / 2.0

    def curved_face_mesh(width: float, height: float, name: str):
        def section(z: float, thickness: float, y_center: float) -> list[tuple[float, float, float]]:
            profile = rounded_rect_profile(
                width,
                thickness,
                radius=min(0.005, thickness * 0.45),
                corner_segments=8,
            )
            return [(x, y + y_center, z) for x, y in profile]

        geom = section_loft(
            [
                section(-0.006, 0.014, 0.006),
                section(height * 0.48, 0.024, 0.012),
                section(height, 0.015, 0.005),
            ]
        )
        return mesh_from_geometry(geom, name)

    storage_tub = model.part("storage_tub")
    storage_tub.visual(
        Box((opening_w - 0.010, tub_depth, shell_t)),
        origin=Origin(xyz=(0.0, -tub_depth / 2.0, -opening_h / 2.0 - shell_t / 2.0 + 0.002)),
        material=tub_black,
        name="floor_shell",
    )
    storage_tub.visual(
        Box((opening_w - 0.010, tub_depth, shell_t)),
        origin=Origin(xyz=(0.0, -tub_depth / 2.0, opening_h / 2.0 + shell_t / 2.0 - 0.002)),
        material=tub_black,
        name="roof_shell",
    )
    storage_tub.visual(
        Box((shell_t, tub_depth, opening_h - 0.004)),
        origin=Origin(xyz=(-opening_w / 2.0 - shell_t / 2.0 + 0.001, -tub_depth / 2.0, 0.0)),
        material=tub_black,
        name="left_wall",
    )
    storage_tub.visual(
        Box((shell_t, tub_depth, opening_h - 0.004)),
        origin=Origin(xyz=(opening_w / 2.0 + shell_t / 2.0 - 0.001, -tub_depth / 2.0, 0.0)),
        material=tub_black,
        name="right_wall",
    )
    storage_tub.visual(
        Box((opening_w - 0.010, shell_t, opening_h - 0.004)),
        origin=Origin(xyz=(0.0, -tub_depth + shell_t / 2.0, 0.0)),
        material=tub_black,
        name="back_shell",
    )
    storage_tub.visual(
        Box((opening_w + 0.050, bezel_t, 0.022)),
        origin=Origin(xyz=(0.0, bezel_t / 2.0, opening_h / 2.0 + 0.011)),
        material=dash_dark,
        name="top_bezel",
    )
    storage_tub.visual(
        Box((opening_w + 0.050, bezel_t, 0.020)),
        origin=Origin(xyz=(0.0, bezel_t / 2.0, -opening_h / 2.0 - 0.010)),
        material=dash_dark,
        name="bottom_bezel",
    )
    storage_tub.visual(
        Box((0.028, bezel_t, opening_h + 0.026)),
        origin=Origin(xyz=(-opening_w / 2.0 - 0.014, bezel_t / 2.0, 0.0)),
        material=dash_dark,
        name="left_bezel",
    )
    storage_tub.visual(
        Box((0.028, bezel_t, opening_h + 0.026)),
        origin=Origin(xyz=(opening_w / 2.0 + 0.014, bezel_t / 2.0, 0.0)),
        material=dash_dark,
        name="right_bezel",
    )
    storage_tub.visual(
        Box((0.004, 0.008, 0.020)),
        origin=Origin(xyz=(left_support_x - 0.006, support_y, support_z)),
        material=arm_steel,
        name="left_support_lug_outer",
    )
    storage_tub.visual(
        Box((0.004, 0.008, 0.020)),
        origin=Origin(xyz=(left_support_x + 0.006, support_y, support_z)),
        material=arm_steel,
        name="left_support_lug_inner",
    )
    storage_tub.visual(
        Box((0.020, 0.016, 0.018)),
        origin=Origin(xyz=(left_support_x, support_y - 0.012, support_z - 0.010)),
        material=tub_black,
        name="left_support_bracket",
    )
    storage_tub.visual(
        Box((0.020, 0.016, 0.018)),
        origin=Origin(xyz=(right_support_x, support_y - 0.012, support_z - 0.010)),
        material=tub_black,
        name="right_support_bracket",
    )
    storage_tub.visual(
        Box((0.004, 0.008, 0.020)),
        origin=Origin(xyz=(right_support_x - 0.006, support_y, support_z)),
        material=arm_steel,
        name="right_support_lug_inner",
    )
    storage_tub.visual(
        Box((0.004, 0.008, 0.020)),
        origin=Origin(xyz=(right_support_x + 0.006, support_y, support_z)),
        material=arm_steel,
        name="right_support_lug_outer",
    )
    storage_tub.inertial = Inertial.from_geometry(
        Box((opening_w + 0.072, tub_depth + 0.010, opening_h + 0.070)),
        mass=4.6,
        origin=Origin(xyz=(0.0, -0.090, 0.0)),
    )

    def build_hinge_arm(name: str) -> object:
        arm = model.part(name)
        arm.visual(
            Cylinder(radius=0.0055, length=0.010),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=arm_steel,
            name="upper_pivot_pin",
        )
        arm.visual(
            Box((0.010, 0.016, 0.060)),
            origin=Origin(xyz=(0.0, 0.011, -0.024)),
            material=arm_steel,
            name="arm_body",
        )
        arm.visual(
            Cylinder(radius=0.0055, length=0.010),
            origin=Origin(xyz=arm_drop, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=arm_steel,
            name="lower_pivot",
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.016, 0.032, arm_length + 0.014)),
            mass=0.18,
            origin=Origin(xyz=(0.0, arm_drop[1] / 2.0, arm_drop[2] / 2.0)),
        )
        return arm

    left_hinge_arm = build_hinge_arm("left_hinge_arm")
    right_hinge_arm = build_hinge_arm("right_hinge_arm")

    door = model.part("door")
    face_mesh = curved_face_mesh(pivot_spacing - 0.024, 0.166, "glovebox_door_face")
    door.visual(
        face_mesh,
        origin=Origin(xyz=(pivot_spacing / 2.0, 0.0, -0.004)),
        material=dash_dark,
        name="face_shell",
    )
    door.visual(
        Box((pivot_spacing - 0.064, 0.004, 0.074)),
        origin=Origin(xyz=(pivot_spacing / 2.0, -0.032, 0.040)),
        material=tub_black,
        name="tray_floor",
    )
    door.visual(
        Box((0.014, 0.060, 0.074)),
        origin=Origin(xyz=(0.036, -0.030, 0.040)),
        material=tub_black,
        name="left_cheek",
    )
    door.visual(
        Box((0.014, 0.060, 0.074)),
        origin=Origin(xyz=(pivot_spacing - 0.036, -0.030, 0.040)),
        material=tub_black,
        name="right_cheek",
    )
    door.visual(
        Box((pivot_spacing - 0.066, 0.060, 0.006)),
        origin=Origin(xyz=(pivot_spacing / 2.0, -0.030, 0.003)),
        material=tub_black,
        name="front_tray_lip",
    )
    door.visual(
        Box((0.112, 0.014, 0.018)),
        origin=Origin(xyz=(pivot_spacing / 2.0, 0.029, 0.090)),
        material=latch_black,
        name="handle_mount",
    )
    door.visual(
        Box((0.050, 0.024, 0.034)),
        origin=Origin(xyz=(0.029, -0.010, 0.018)),
        material=tub_black,
        name="left_pivot_block",
    )
    door.visual(
        Box((0.050, 0.024, 0.034)),
        origin=Origin(xyz=(pivot_spacing - 0.029, -0.010, 0.018)),
        material=tub_black,
        name="right_pivot_block",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_steel,
        name="left_pivot_boss",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(pivot_spacing, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=arm_steel,
        name="right_pivot_boss",
    )
    door.inertial = Inertial.from_geometry(
        Box((pivot_spacing, 0.078, 0.172)),
        mass=1.3,
        origin=Origin(xyz=(pivot_spacing / 2.0, -0.016, 0.076)),
    )

    handle = model.part("pull_handle")
    handle_loop = wire_from_points(
        [
            (-0.048, 0.000, 0.000),
            (-0.036, 0.010, -0.010),
            (-0.030, 0.018, -0.028),
            (0.030, 0.018, -0.028),
            (0.036, 0.010, -0.010),
            (0.048, 0.000, 0.000),
        ],
        radius=0.0045,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.010,
        corner_segments=8,
    )
    handle.visual(
        mesh_from_geometry(handle_loop, "pull_handle_loop"),
        material=trim_silver,
        name="handle_loop",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.110, 0.032, 0.034)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.009, -0.014)),
    )

    model.articulation(
        "left_arm_support",
        ArticulationType.REVOLUTE,
        parent=storage_tub,
        child=left_hinge_arm,
        origin=Origin(xyz=(left_support_x, support_y, support_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=-0.12,
            upper=1.05,
        ),
    )
    model.articulation(
        "right_arm_support",
        ArticulationType.REVOLUTE,
        parent=storage_tub,
        child=right_hinge_arm,
        origin=Origin(xyz=(right_support_x, support_y, support_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.5,
            lower=-0.12,
            upper=1.05,
        ),
    )
    model.articulation(
        "left_arm_to_door",
        ArticulationType.REVOLUTE,
        parent=left_hinge_arm,
        child=door,
        origin=Origin(xyz=arm_drop),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=-0.08,
            upper=0.78,
        ),
    )
    model.articulation(
        "door_to_handle",
        ArticulationType.REVOLUTE,
        parent=door,
        child=handle,
        origin=Origin(xyz=(pivot_spacing / 2.0, 0.036, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.0,
            lower=-0.22,
            upper=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    storage_tub = object_model.get_part("storage_tub")
    left_hinge_arm = object_model.get_part("left_hinge_arm")
    right_hinge_arm = object_model.get_part("right_hinge_arm")
    door = object_model.get_part("door")
    handle = object_model.get_part("pull_handle")

    left_arm_support = object_model.get_articulation("left_arm_support")
    right_arm_support = object_model.get_articulation("right_arm_support")
    left_arm_to_door = object_model.get_articulation("left_arm_to_door")
    door_to_handle = object_model.get_articulation("door_to_handle")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(
        door,
        left_hinge_arm,
        elem_a="left_pivot_boss",
        elem_b="lower_pivot",
        reason="The left lower door pivot and hinge-arm pivot share the same hinge pin axis.",
    )
    ctx.allow_overlap(
        door,
        right_hinge_arm,
        elem_a="right_pivot_boss",
        elem_b="lower_pivot",
        reason="The right lower door pivot and hinge-arm pivot share the same hinge pin axis.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "arm and door hinge axes aligned to x",
        left_arm_support.axis == (1.0, 0.0, 0.0)
        and right_arm_support.axis == (1.0, 0.0, 0.0)
        and left_arm_to_door.axis == (1.0, 0.0, 0.0)
        and door_to_handle.axis == (1.0, 0.0, 0.0),
        "All user-facing pivots should rotate about the vehicle width axis.",
    )

    ctx.expect_contact(
        left_hinge_arm,
        storage_tub,
        elem_a="upper_pivot_pin",
        elem_b="left_support_lug_inner",
        name="left arm is carried by tub support",
    )
    ctx.expect_contact(
        right_hinge_arm,
        storage_tub,
        elem_a="upper_pivot_pin",
        elem_b="right_support_lug_inner",
        name="right arm is carried by tub support",
    )
    ctx.expect_contact(
        door,
        left_hinge_arm,
        elem_a="left_pivot_boss",
        elem_b="lower_pivot",
        name="door is clipped to left hinge arm",
    )
    ctx.expect_overlap(
        handle,
        door,
        axes="x",
        min_overlap=0.09,
        name="handle spans the center of the door",
    )
    ctx.expect_gap(
        handle,
        door,
        axis="y",
        max_penetration=0.004,
        max_gap=0.030,
        name="handle sits just proud of the door face",
    )

    with ctx.pose(
        {
            left_arm_support: 0.92,
            right_arm_support: 0.92,
            left_arm_to_door: 0.60,
        }
    ):
        ctx.expect_contact(
            door,
            right_hinge_arm,
            elem_a="right_pivot_boss",
            elem_b="lower_pivot",
            name="open door stays clipped to right hinge arm",
        )
        tray_floor_aabb = ctx.part_element_world_aabb(door, elem="tray_floor")
        if tray_floor_aabb is None:
            ctx.fail("tray floor aabb available in open pose", "Tray floor bounds were unavailable.")
        else:
            tray_floor_z = tray_floor_aabb[1][2] - tray_floor_aabb[0][2]
            ctx.check(
                "open door forms a tray",
                tray_floor_z < 0.018,
                f"Open tray floor should be nearly horizontal; z extent was {tray_floor_z:.4f} m.",
            )

    handle_rest = ctx.part_world_aabb(handle)
    if handle_rest is None:
        ctx.fail("handle visible at rest", "Handle bounds were unavailable.")
    else:
        with ctx.pose({door_to_handle: 0.20}):
            handle_pulled = ctx.part_world_aabb(handle)
            if handle_pulled is None:
                ctx.fail("handle visible when pulled", "Pulled handle bounds were unavailable.")
            else:
                ctx.check(
                    "handle rotates slightly outward",
                    handle_pulled[1][1] > handle_rest[1][1] + 0.004,
                    "Pull handle should swing slightly outward from the door face.",
                )
                ctx.expect_gap(
                    handle,
                    door,
                    axis="y",
                    max_penetration=0.004,
                    max_gap=0.038,
                    name="handle stays close to door when pulled",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
