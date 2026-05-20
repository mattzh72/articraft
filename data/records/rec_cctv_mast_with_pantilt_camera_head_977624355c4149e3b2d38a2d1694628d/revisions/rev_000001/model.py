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
    DomeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _axis_close(actual: tuple[float, float, float], expected: tuple[float, float, float], tol: float = 1e-6) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(actual, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fence_top_cctv_mount")

    galvanized = model.material("galvanized_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    coated_steel = model.material("coated_steel", rgba=(0.31, 0.33, 0.36, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.55, 0.58, 0.62, 1.0))
    housing_white = model.material("housing_white", rgba=(0.91, 0.92, 0.93, 1.0))
    dome_smoke = model.material("dome_smoke", rgba=(0.18, 0.20, 0.23, 0.34))
    camera_black = model.material("camera_black", rgba=(0.08, 0.09, 0.10, 1.0))

    dome_bubble_geometry = DomeGeometry(
        radius=0.05,
        radial_segments=40,
        height_segments=20,
        closed=False,
    )
    dome_bubble_geometry.rotate_x(math.pi)
    dome_bubble_geometry.translate(0.0, 0.0, -0.028)
    dome_bubble_mesh = mesh_from_geometry(dome_bubble_geometry, "camera_dome_bubble")

    mount_base = model.part("mount_base")
    mount_base.visual(
        Box((0.03, 0.16, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=galvanized,
        name="t_bar_crossbar",
    )
    mount_base.visual(
        Box((0.026, 0.095, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=galvanized,
        name="clamp_plate",
    )
    for side, label in ((1.0, "left"), (-1.0, "right")):
        y_pos = 0.028 * side
        mount_base.visual(
            Cylinder(radius=0.004, length=0.04),
            origin=Origin(xyz=(0.0, y_pos, 0.048)),
            material=coated_steel,
            name=f"{label}_clamp_rod",
        )
        mount_base.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(0.0, y_pos, 0.081)),
            material=galvanized,
            name=f"{label}_top_nut",
        )
        mount_base.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(0.0, y_pos, 0.017)),
            material=galvanized,
            name=f"{label}_bottom_nut",
        )
    mount_base.visual(
        Box((0.016, 0.046, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=galvanized,
        name="riser_web",
    )
    mount_base.visual(
        Cylinder(radius=0.017, length=0.13),
        origin=Origin(xyz=(0.0, 0.0, 0.143)),
        material=coated_steel,
        name="riser_tube",
    )
    mount_base.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=coated_steel,
        name="riser_collar",
    )
    mount_base.visual(
        Cylinder(radius=0.03, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.224)),
        material=coated_steel,
        name="bearing_plate",
    )
    mount_base.inertial = Inertial.from_geometry(
        Box((0.16, 0.16, 0.228)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.029, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=coated_steel,
        name="turntable_collar",
    )
    pan_yoke.visual(
        Box((0.05, 0.05, 0.032)),
        origin=Origin(xyz=(-0.018, 0.0, 0.028)),
        material=coated_steel,
        name="pan_motor_pod",
    )
    pan_yoke.visual(
        Box((0.018, 0.024, 0.022)),
        origin=Origin(xyz=(-0.022, 0.0, 0.046)),
        material=coated_steel,
        name="pan_spine",
    )
    pan_yoke.visual(
        Box((0.018, 0.112, 0.012)),
        origin=Origin(xyz=(-0.022, 0.0, 0.058)),
        material=coated_steel,
        name="rear_bridge",
    )
    for side, label in ((1.0, "left"), (-1.0, "right")):
        arm_y = 0.056 * side
        pan_yoke.visual(
            Box((0.018, 0.008, 0.064)),
            origin=Origin(xyz=(-0.022, arm_y, 0.026)),
            material=coated_steel,
            name=f"{label}_arm",
        )
        pan_yoke.visual(
            Box((0.028, 0.008, 0.012)),
            origin=Origin(xyz=(-0.011, arm_y, 0.054)),
            material=coated_steel,
            name=f"{label}_arm_gusset",
        )
        pan_yoke.visual(
            Cylinder(radius=0.013, length=0.008),
            origin=Origin(xyz=(0.0, arm_y, 0.054), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=trim_gray,
            name=f"{label}_pivot_cap",
        )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.12, 0.13, 0.09)),
        mass=1.2,
        origin=Origin(xyz=(-0.01, 0.0, 0.03)),
    )

    camera_head = model.part("camera_head")
    camera_head.visual(
        Box((0.018, 0.094, 0.016)),
        origin=Origin(xyz=(0.009, 0.0, -0.002)),
        material=housing_white,
        name="trunnion_block",
    )
    camera_head.visual(
        Box((0.05, 0.022, 0.018)),
        origin=Origin(xyz=(0.026, 0.0, -0.004)),
        material=housing_white,
        name="connector_spine",
    )
    camera_head.visual(
        Cylinder(radius=0.052, length=0.032),
        origin=Origin(xyz=(0.086, 0.0, -0.012)),
        material=housing_white,
        name="cap_shell",
    )
    camera_head.visual(
        Cylinder(radius=0.056, length=0.008),
        origin=Origin(xyz=(0.086, 0.0, -0.024)),
        material=trim_gray,
        name="seam_ring",
    )
    camera_head.visual(
        dome_bubble_mesh,
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        material=dome_smoke,
        name="dome_bubble",
    )
    camera_head.visual(
        Cylinder(radius=0.011, length=0.02),
        origin=Origin(xyz=(0.086, 0.0, -0.038)),
        material=camera_black,
        name="sensor_neck",
    )
    camera_head.visual(
        Cylinder(radius=0.019, length=0.034),
        origin=Origin(xyz=(0.094, 0.0, -0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_black,
        name="sensor_pod",
    )
    camera_head.visual(
        Cylinder(radius=0.01, length=0.02),
        origin=Origin(xyz=(0.116, 0.0, -0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="lens_barrel",
    )
    for side, label in ((1.0, "left"), (-1.0, "right")):
        camera_head.visual(
            Cylinder(radius=0.011, length=0.01),
            origin=Origin(xyz=(0.0, 0.047 * side, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=housing_white,
            name=f"{label}_trunnion",
        )
    camera_head.inertial = Inertial.from_geometry(
        Box((0.15, 0.104, 0.082)),
        mass=0.9,
        origin=Origin(xyz=(0.075, 0.0, -0.032)),
    )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=mount_base,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.226)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=-2.8,
            upper=2.8,
        ),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=camera_head,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-1.2,
            upper=0.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount_base = object_model.get_part("mount_base")
    pan_yoke = object_model.get_part("pan_yoke")
    camera_head = object_model.get_part("camera_head")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_axis = object_model.get_articulation("tilt_axis")

    bearing_plate = mount_base.get_visual("bearing_plate")
    turntable_collar = pan_yoke.get_visual("turntable_collar")
    rear_bridge = pan_yoke.get_visual("rear_bridge")
    left_pivot_cap = pan_yoke.get_visual("left_pivot_cap")
    right_pivot_cap = pan_yoke.get_visual("right_pivot_cap")
    cap_shell = camera_head.get_visual("cap_shell")
    left_trunnion = camera_head.get_visual("left_trunnion")
    right_trunnion = camera_head.get_visual("right_trunnion")

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

    ctx.check(
        "pan axis is vertical",
        _axis_close(pan_axis.axis, (0.0, 0.0, 1.0)),
        details=f"axis={pan_axis.axis}",
    )
    ctx.check(
        "tilt axis is horizontal",
        _axis_close(tilt_axis.axis, (0.0, 1.0, 0.0)),
        details=f"axis={tilt_axis.axis}",
    )
    pan_limits = pan_axis.motion_limits
    tilt_limits = tilt_axis.motion_limits
    ctx.check(
        "pan travel covers a wide surveillance sweep",
        pan_limits is not None
        and pan_limits.lower is not None
        and pan_limits.upper is not None
        and pan_limits.lower <= -2.7
        and pan_limits.upper >= 2.7,
        details=f"limits={pan_limits}",
    )
    ctx.check(
        "tilt travel covers up and down aim",
        tilt_limits is not None
        and tilt_limits.lower is not None
        and tilt_limits.upper is not None
        and tilt_limits.lower <= -1.1
        and tilt_limits.upper >= 0.7,
        details=f"limits={tilt_limits}",
    )

    with ctx.pose({pan_axis: 0.0, tilt_axis: 0.0}):
        ctx.expect_contact(
            pan_yoke,
            mount_base,
            elem_a=turntable_collar,
            elem_b=bearing_plate,
            name="pan turntable seats on riser bearing",
        )
        ctx.expect_overlap(
            pan_yoke,
            mount_base,
            axes="xy",
            min_overlap=0.05,
            elem_a=turntable_collar,
            elem_b=bearing_plate,
            name="pan bearing stays centered on riser",
        )
        ctx.expect_contact(
            camera_head,
            pan_yoke,
            elem_a=left_trunnion,
            elem_b=left_pivot_cap,
            name="left trunnion is supported by left yoke pivot",
        )
        ctx.expect_contact(
            camera_head,
            pan_yoke,
            elem_a=right_trunnion,
            elem_b=right_pivot_cap,
            name="right trunnion is supported by right yoke pivot",
        )

    with ctx.pose({pan_axis: 1.1, tilt_axis: 0.8}):
        ctx.expect_gap(
            pan_yoke,
            camera_head,
            axis="z",
            min_gap=0.015,
            positive_elem=rear_bridge,
            negative_elem=cap_shell,
            name="rear bridge clears camera cap at high up-tilt",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
