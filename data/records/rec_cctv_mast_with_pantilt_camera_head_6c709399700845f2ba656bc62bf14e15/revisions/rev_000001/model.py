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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _camera_body_mesh(name: str):
    geom = CapsuleGeometry(radius=0.019, length=0.060, radial_segments=24, height_segments=8)
    geom.rotate_y(math.pi / 2.0).translate(0.038, 0.0, 0.0)
    return mesh_from_geometry(geom, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[index] + high[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_arm_corner_cctv_bracket")

    bracket_gray = model.material("bracket_gray", rgba=(0.74, 0.76, 0.79, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    camera_off_white = model.material("camera_off_white", rgba=(0.88, 0.89, 0.86, 1.0))
    lens_black = model.material("lens_black", rgba=(0.06, 0.07, 0.08, 1.0))

    plate = model.part("wall_plate")
    plate_profile = [
        (-0.115, 0.0),
        (-0.060, -0.118),
        (0.0, -0.118),
        (0.0, 0.118),
        (-0.060, 0.118),
    ]
    plate.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(plate_profile, 0.180),
            "corner_wall_plate",
        ),
        material=bracket_gray,
        name="plate_shell",
    )
    plate.visual(
        Box((0.012, 0.028, 0.042)),
        origin=Origin(xyz=(0.006, -0.072, 0.0)),
        material=bracket_gray,
        name="left_hinge_block",
    )
    plate.visual(
        Box((0.012, 0.028, 0.042)),
        origin=Origin(xyz=(0.006, 0.072, 0.0)),
        material=bracket_gray,
        name="right_hinge_block",
    )
    plate.visual(
        Cylinder(radius=0.014, length=0.002),
        origin=Origin(xyz=(0.013, -0.072, -0.017)),
        material=hinge_dark,
        name="left_hinge_cap_bottom",
    )
    plate.visual(
        Cylinder(radius=0.014, length=0.002),
        origin=Origin(xyz=(0.013, -0.072, 0.017)),
        material=hinge_dark,
        name="left_hinge_cap_top",
    )
    plate.visual(
        Cylinder(radius=0.014, length=0.002),
        origin=Origin(xyz=(0.013, 0.072, -0.017)),
        material=hinge_dark,
        name="right_hinge_cap_bottom",
    )
    plate.visual(
        Cylinder(radius=0.014, length=0.002),
        origin=Origin(xyz=(0.013, 0.072, 0.017)),
        material=hinge_dark,
        name="right_hinge_cap_top",
    )

    def add_side_assembly(prefix: str, y_offset: float, outward_axis: float) -> None:
        arm = model.part(f"{prefix}_arm")
        arm.visual(
            Box((0.024, 0.030, 0.024)),
            origin=Origin(xyz=(0.012, 0.0, 0.0)),
            material=bracket_gray,
            name="hinge_leaf",
        )
        arm.visual(
            Cylinder(radius=0.010, length=0.026),
            origin=Origin(xyz=(0.014, 0.0, 0.0)),
            material=hinge_dark,
            name="hinge_collar",
        )
        arm.visual(
            Box((0.028, 0.030, 0.022)),
            origin=Origin(xyz=(0.030, 0.0, 0.0)),
            material=bracket_gray,
            name="hinge_bridge",
        )
        arm.visual(
            Box((0.160, 0.030, 0.020)),
            origin=Origin(xyz=(0.124, 0.0, 0.0)),
            material=bracket_gray,
            name="arm_tube",
        )
        arm.visual(
            Box((0.090, 0.018, 0.008)),
            origin=Origin(xyz=(0.112, 0.0, -0.014)),
            material=bracket_gray,
            name="arm_stiffener",
        )
        arm.visual(
            Box((0.048, 0.038, 0.020)),
            origin=Origin(xyz=(0.212, 0.0, -0.010)),
            material=bracket_gray,
            name="tip_block",
        )
        arm.visual(
            Cylinder(radius=0.013, length=0.008),
            origin=Origin(xyz=(0.224, 0.0, -0.004)),
            material=hinge_dark,
            name="pan_cap_bottom",
        )
        arm.visual(
            Box((0.022, 0.028, 0.012)),
            origin=Origin(xyz=(0.220, 0.0, -0.014)),
            material=bracket_gray,
            name="pan_mount_web",
        )

        model.articulation(
            f"wall_plate_to_{prefix}_arm",
            ArticulationType.REVOLUTE,
            parent=plate,
            child=arm,
            origin=Origin(xyz=(0.012, y_offset, 0.0)),
            axis=(0.0, 0.0, outward_axis),
            motion_limits=MotionLimits(
                effort=15.0,
                velocity=1.5,
                lower=-0.55,
                upper=1.10,
            ),
        )

        yoke = model.part(f"{prefix}_yoke")
        yoke.visual(
            Cylinder(radius=0.011, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=hinge_dark,
            name="pan_collar",
        )
        yoke.visual(
            Box((0.020, 0.026, 0.028)),
            origin=Origin(xyz=(0.010, 0.0, 0.014)),
            material=bracket_gray,
            name="pan_pedestal",
        )
        yoke.visual(
            Box((0.014, 0.060, 0.026)),
            origin=Origin(xyz=(0.007, 0.0, 0.041)),
            material=bracket_gray,
            name="rear_strap",
        )
        yoke.visual(
            Box((0.030, 0.070, 0.006)),
            origin=Origin(xyz=(0.027, 0.0, 0.057)),
            material=bracket_gray,
            name="roof_bridge",
        )
        yoke.visual(
            Box((0.036, 0.004, 0.042)),
            origin=Origin(xyz=(0.028, -0.029, 0.033)),
            material=bracket_gray,
            name="left_ear",
        )
        yoke.visual(
            Box((0.036, 0.004, 0.042)),
            origin=Origin(xyz=(0.028, 0.029, 0.033)),
            material=bracket_gray,
            name="right_ear",
        )

        model.articulation(
            f"{prefix}_arm_to_{prefix}_yoke",
            ArticulationType.REVOLUTE,
            parent=arm,
            child=yoke,
            origin=Origin(xyz=(0.224, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=2.5,
                lower=-1.80,
                upper=1.80,
            ),
        )

        camera = model.part(f"{prefix}_camera")
        camera.visual(
            Cylinder(radius=0.008, length=0.054),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_dark,
            name="tilt_axle",
        )
        camera.visual(
            _camera_body_mesh(f"{prefix}_camera_body"),
            material=camera_off_white,
            name="camera_shell",
        )
        camera.visual(
            Cylinder(radius=0.013, length=0.010),
            origin=Origin(xyz=(0.089, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=lens_black,
            name="lens_bezel",
        )
        camera.visual(
            Box((0.070, 0.046, 0.004)),
            origin=Origin(xyz=(0.042, 0.0, 0.018)),
            material=camera_off_white,
            name="sunshield",
        )

        model.articulation(
            f"{prefix}_yoke_to_{prefix}_camera",
            ArticulationType.REVOLUTE,
            parent=yoke,
            child=camera,
            origin=Origin(xyz=(0.028, 0.0, 0.033)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=2.0,
                lower=-1.00,
                upper=0.70,
            ),
        )

    add_side_assembly("left", -0.072, -1.0)
    add_side_assembly("right", 0.072, 1.0)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plate = object_model.get_part("wall_plate")
    left_arm = object_model.get_part("left_arm")
    right_arm = object_model.get_part("right_arm")
    left_yoke = object_model.get_part("left_yoke")
    right_yoke = object_model.get_part("right_yoke")
    left_camera = object_model.get_part("left_camera")
    right_camera = object_model.get_part("right_camera")

    left_arm_joint = object_model.get_articulation("wall_plate_to_left_arm")
    right_arm_joint = object_model.get_articulation("wall_plate_to_right_arm")
    left_pan_joint = object_model.get_articulation("left_arm_to_left_yoke")
    right_pan_joint = object_model.get_articulation("right_arm_to_right_yoke")
    left_tilt_joint = object_model.get_articulation("left_yoke_to_left_camera")
    right_tilt_joint = object_model.get_articulation("right_yoke_to_right_camera")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    ctx.expect_contact(left_arm, plate, name="left arm hinge collar is supported by the wall plate")
    ctx.expect_contact(right_arm, plate, name="right arm hinge collar is supported by the wall plate")
    ctx.expect_contact(left_yoke, left_arm, name="left pan yoke is supported by the arm tip")
    ctx.expect_contact(right_yoke, right_arm, name="right pan yoke is supported by the arm tip")
    ctx.expect_contact(left_camera, left_yoke, name="left camera trunnion is captured by the yoke")
    ctx.expect_contact(right_camera, right_yoke, name="right camera trunnion is captured by the yoke")

    ctx.expect_origin_gap(
        left_camera,
        plate,
        axis="x",
        min_gap=0.20,
        name="left camera projects out in front of the corner plate",
    )
    ctx.expect_origin_gap(
        right_camera,
        plate,
        axis="x",
        min_gap=0.20,
        name="right camera projects out in front of the corner plate",
    )

    left_yoke_rest = ctx.part_world_position(left_yoke)
    right_yoke_rest = ctx.part_world_position(right_yoke)
    with ctx.pose(
        {
            left_arm_joint: left_arm_joint.motion_limits.upper,
            right_arm_joint: right_arm_joint.motion_limits.upper,
        }
    ):
        left_yoke_open = ctx.part_world_position(left_yoke)
        right_yoke_open = ctx.part_world_position(right_yoke)
    ctx.check(
        "side arms swing outward from the plate face",
        left_yoke_rest is not None
        and right_yoke_rest is not None
        and left_yoke_open is not None
        and right_yoke_open is not None
        and left_yoke_open[1] < left_yoke_rest[1] - 0.05
        and right_yoke_open[1] > right_yoke_rest[1] + 0.05,
        details=(
            f"left rest/open={left_yoke_rest}/{left_yoke_open}, "
            f"right rest/open={right_yoke_rest}/{right_yoke_open}"
        ),
    )

    left_camera_rest = ctx.part_world_position(left_camera)
    right_camera_rest = ctx.part_world_position(right_camera)
    with ctx.pose({left_pan_joint: 0.75, right_pan_joint: 0.75}):
        left_camera_panned = ctx.part_world_position(left_camera)
        right_camera_panned = ctx.part_world_position(right_camera)
    ctx.check(
        "pan joints rotate the camera heads around vertical axes",
        left_camera_rest is not None
        and right_camera_rest is not None
        and left_camera_panned is not None
        and right_camera_panned is not None
        and left_camera_panned[1] > left_camera_rest[1] + 0.01
        and right_camera_panned[1] > right_camera_rest[1] + 0.01,
        details=(
            f"left rest/panned={left_camera_rest}/{left_camera_panned}, "
            f"right rest/panned={right_camera_rest}/{right_camera_panned}"
        ),
    )

    left_lens_rest = _aabb_center(ctx.part_element_world_aabb(left_camera, elem="lens_bezel"))
    right_lens_rest = _aabb_center(ctx.part_element_world_aabb(right_camera, elem="lens_bezel"))
    with ctx.pose({left_tilt_joint: 0.55, right_tilt_joint: 0.55}):
        left_lens_tilted = _aabb_center(ctx.part_element_world_aabb(left_camera, elem="lens_bezel"))
        right_lens_tilted = _aabb_center(ctx.part_element_world_aabb(right_camera, elem="lens_bezel"))
    ctx.check(
        "tilt joints pitch the camera noses upward",
        left_lens_rest is not None
        and right_lens_rest is not None
        and left_lens_tilted is not None
        and right_lens_tilted is not None
        and left_lens_tilted[2] > left_lens_rest[2] + 0.02
        and right_lens_tilted[2] > right_lens_rest[2] + 0.02,
        details=(
            f"left lens rest/tilted={left_lens_rest}/{left_lens_tilted}, "
            f"right lens rest/tilted={right_lens_rest}/{right_lens_tilted}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
