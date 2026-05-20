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
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stadium_floodlight_pole_cctv_arm")

    concrete = model.material("concrete", rgba=(0.71, 0.71, 0.73, 1.0))
    galvanized = model.material("galvanized", rgba=(0.58, 0.60, 0.63, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    lamp_face = model.material("lamp_face", rgba=(0.83, 0.85, 0.88, 1.0))
    optics = model.material("optics", rgba=(0.25, 0.30, 0.36, 1.0))

    def straight_tube(
        name: str,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        radius: float,
    ):
        return mesh_from_geometry(
            wire_from_points(
                [start, end],
                radius=radius,
                radial_segments=20,
                cap_ends=True,
            ),
            name,
        )

    mast = model.part("mast")
    mast.visual(
        Box((1.8, 1.8, 0.9)),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=concrete,
        name="foundation_block",
    )
    mast.visual(
        Cylinder(radius=0.20, length=18.1),
        origin=Origin(xyz=(0.0, 0.0, 9.95)),
        material=galvanized,
        name="mast_shaft",
    )
    mast.visual(
        Cylinder(radius=0.34, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.96)),
        material=painted_steel,
        name="base_flange",
    )
    mast.visual(
        Cylinder(radius=0.24, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 19.10)),
        material=galvanized,
        name="top_cap",
    )
    mast.visual(
        Cylinder(radius=0.03, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 19.45)),
        material=galvanized,
        name="lightning_spike",
    )
    mast.inertial = Inertial.from_geometry(
        Box((1.8, 1.8, 19.6)),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, 9.8)),
    )

    crossbar = model.part("crossbar")
    crossbar.visual(
        Box((0.42, 0.14, 0.32)),
        origin=Origin(xyz=(0.0, 0.27, 0.0)),
        material=painted_steel,
        name="crossbar_mount_box",
    )
    crossbar.visual(
        Cylinder(radius=0.08, length=3.8),
        origin=Origin(xyz=(0.0, 0.34, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="crossbar_tube",
    )
    for side, x_sign in (("left", -1.0), ("right", 1.0)):
        x_pos = 1.05 * x_sign
        crossbar.visual(
            Box((0.08, 0.12, 0.16)),
            origin=Origin(xyz=(x_pos, 0.31, -0.16)),
            material=painted_steel,
            name=f"{side}_lamp_yoke",
        )
        crossbar.visual(
            Box((0.42, 0.20, 0.28)),
            origin=Origin(xyz=(x_pos, 0.44, -0.24), rpy=(-0.35, 0.0, 0.0)),
            material=painted_steel,
            name=f"{side}_lamp_housing",
        )
        crossbar.visual(
            Box((0.38, 0.015, 0.22)),
            origin=Origin(xyz=(x_pos, 0.53, -0.26), rpy=(-0.35, 0.0, 0.0)),
            material=lamp_face,
            name=f"{side}_lamp_face",
        )
    crossbar.inertial = Inertial.from_geometry(
        Box((3.9, 0.8, 0.9)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.34, -0.12)),
    )

    model.articulation(
        "mast_to_crossbar",
        ArticulationType.FIXED,
        parent=mast,
        child=crossbar,
        origin=Origin(xyz=(0.0, 0.0, 17.6)),
    )

    camera_arm = model.part("camera_arm")
    camera_arm.visual(
        Box((0.30, 0.24, 0.30)),
        origin=Origin(xyz=(0.0, 0.32, 0.0)),
        material=painted_steel,
        name="arm_mount_box",
    )
    camera_arm.visual(
        straight_tube("camera_arm_main_tube", (0.0, 0.44, 0.06), (0.0, 1.55, 0.10), 0.045),
        material=painted_steel,
        name="arm_main_tube",
    )
    camera_arm.visual(
        straight_tube("camera_arm_brace_tube", (0.0, 0.30, -0.18), (0.0, 1.08, 0.02), 0.030),
        material=painted_steel,
        name="arm_brace_tube",
    )
    camera_arm.visual(
        Box((0.18, 0.16, 0.12)),
        origin=Origin(xyz=(0.0, 1.55, 0.10)),
        material=painted_steel,
        name="arm_tip_plate",
    )
    camera_arm.visual(
        Box((0.18, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, 0.62, 0.19)),
        material=painted_steel,
        name="service_box",
    )
    camera_arm.visual(
        Box((0.06, 0.08, 0.10)),
        origin=Origin(xyz=(0.0, 0.62, 0.11)),
        material=painted_steel,
        name="service_bracket",
    )
    camera_arm.inertial = Inertial.from_geometry(
        Box((0.36, 1.75, 0.55)),
        mass=90.0,
        origin=Origin(xyz=(0.0, 0.95, 0.04)),
    )

    model.articulation(
        "mast_to_camera_arm",
        ArticulationType.FIXED,
        parent=mast,
        child=camera_arm,
        origin=Origin(xyz=(0.0, 0.0, 15.2)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.11, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=painted_steel,
        name="rotary_base",
    )
    pan_head.visual(
        Box((0.20, 0.14, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=painted_steel,
        name="pedestal_block",
    )
    pan_head.visual(
        Box((0.44, 0.06, 0.20)),
        origin=Origin(xyz=(0.0, -0.03, 0.18)),
        material=painted_steel,
        name="rear_yoke_block",
    )
    pan_head.visual(
        Box((0.025, 0.18, 0.22)),
        origin=Origin(xyz=(-0.221, 0.05, 0.18)),
        material=painted_steel,
        name="left_yoke_plate",
    )
    pan_head.visual(
        Box((0.025, 0.18, 0.22)),
        origin=Origin(xyz=(0.221, 0.05, 0.18)),
        material=painted_steel,
        name="right_yoke_plate",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.48, 0.24, 0.32)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.02, 0.16)),
    )

    model.articulation(
        "camera_arm_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=camera_arm,
        child=pan_head,
        origin=Origin(xyz=(0.0, 1.55, 0.16)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5),
    )

    camera_head = model.part("camera_head")
    camera_head.visual(
        Box((0.32, 0.34, 0.22)),
        origin=Origin(xyz=(0.0, 0.25, 0.0)),
        material=painted_steel,
        name="body_shell",
    )
    camera_head.visual(
        Box((0.34, 0.22, 0.02)),
        origin=Origin(xyz=(0.0, 0.26, 0.12)),
        material=painted_steel,
        name="sunshade",
    )
    camera_head.visual(
        Box((0.26, 0.08, 0.18)),
        origin=Origin(xyz=(0.0, 0.14, 0.0)),
        material=painted_steel,
        name="rear_cowl",
    )
    camera_head.visual(
        Box((0.06, 0.08, 0.16)),
        origin=Origin(xyz=(-0.14, 0.04, 0.0)),
        material=painted_steel,
        name="left_cheek",
    )
    camera_head.visual(
        Box((0.06, 0.08, 0.16)),
        origin=Origin(xyz=(0.14, 0.04, 0.0)),
        material=painted_steel,
        name="right_cheek",
    )
    camera_head.visual(
        Box((0.18, 0.10, 0.16)),
        origin=Origin(xyz=(0.0, 0.41, 0.0)),
        material=painted_steel,
        name="lens_shroud",
    )
    camera_head.visual(
        Cylinder(radius=0.058, length=0.12),
        origin=Origin(xyz=(0.0, 0.46, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=optics,
        name="lens_barrel",
    )
    camera_head.visual(
        Cylinder(radius=0.03, length=0.065),
        origin=Origin(xyz=(-0.176, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="left_trunnion",
    )
    camera_head.visual(
        Cylinder(radius=0.03, length=0.065),
        origin=Origin(xyz=(0.176, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="right_trunnion",
    )
    camera_head.inertial = Inertial.from_geometry(
        Box((0.36, 0.48, 0.30)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.22, 0.0)),
    )

    model.articulation(
        "pan_head_to_camera_head",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera_head,
        origin=Origin(xyz=(0.0, 0.04, 0.18)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=-0.35,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    crossbar = object_model.get_part("crossbar")
    camera_arm = object_model.get_part("camera_arm")
    pan_head = object_model.get_part("pan_head")
    camera_head = object_model.get_part("camera_head")

    pan_joint = object_model.get_articulation("camera_arm_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_camera_head")

    ctx.expect_contact(
        camera_arm,
        mast,
        elem_a="arm_mount_box",
        elem_b="mast_shaft",
        name="camera arm mount box contacts mast",
    )
    ctx.expect_contact(
        crossbar,
        mast,
        elem_a="crossbar_mount_box",
        elem_b="mast_shaft",
        name="crossbar mount box contacts mast",
    )
    ctx.expect_contact(
        camera_arm,
        pan_head,
        elem_a="arm_tip_plate",
        elem_b="rotary_base",
        name="pan bearing sits on arm tip plate",
    )
    ctx.expect_gap(
        pan_head,
        camera_head,
        axis="x",
        positive_elem="right_yoke_plate",
        negative_elem="right_trunnion",
        max_gap=0.002,
        max_penetration=0.0,
        name="right trunnion seats against the tilt yoke",
    )

    pan_limits = pan_joint.motion_limits
    tilt_limits = tilt_joint.motion_limits
    ctx.check(
        "pan joint is continuous vertical bearing",
        pan_joint.articulation_type == ArticulationType.CONTINUOUS
        and pan_joint.axis == (0.0, 0.0, 1.0)
        and pan_limits is not None
        and pan_limits.lower is None
        and pan_limits.upper is None,
        details=(
            f"type={pan_joint.articulation_type}, axis={pan_joint.axis}, "
            f"limits=({None if pan_limits is None else pan_limits.lower}, "
            f"{None if pan_limits is None else pan_limits.upper})"
        ),
    )
    ctx.check(
        "tilt joint is horizontal revolute axis",
        tilt_joint.articulation_type == ArticulationType.REVOLUTE
        and tilt_joint.axis == (-1.0, 0.0, 0.0)
        and tilt_limits is not None
        and tilt_limits.lower is not None
        and tilt_limits.upper is not None
        and tilt_limits.lower < 0.0 < tilt_limits.upper,
        details=(
            f"type={tilt_joint.articulation_type}, axis={tilt_joint.axis}, "
            f"limits=({None if tilt_limits is None else tilt_limits.lower}, "
            f"{None if tilt_limits is None else tilt_limits.upper})"
        ),
    )

    def elem_center(part_obj, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    rest_lens = elem_center(camera_head, "lens_barrel")
    with ctx.pose({pan_joint: math.pi / 2.0}):
        panned_lens = elem_center(camera_head, "lens_barrel")
    ctx.check(
        "pan motion swings the camera around the arm tip",
        rest_lens is not None
        and panned_lens is not None
        and abs(panned_lens[2] - rest_lens[2]) < 0.01
        and abs(panned_lens[0] - rest_lens[0]) > 0.25,
        details=f"rest_lens={rest_lens}, panned_lens={panned_lens}",
    )

    with ctx.pose({tilt_joint: 1.0}):
        tilted_lens = elem_center(camera_head, "lens_barrel")
    ctx.check(
        "positive tilt drives the camera to look downward",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[2] < rest_lens[2] - 0.15,
        details=f"rest_lens={rest_lens}, tilted_lens={tilted_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
