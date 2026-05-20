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
    model = ArticulatedObject(name="tracked_missile_launcher")

    body_green = model.material("body_green", rgba=(0.33, 0.39, 0.25, 1.0))
    shadow_green = model.material("shadow_green", rgba=(0.24, 0.29, 0.18, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.25, 1.0))
    track_rubber = model.material("track_rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    glass = model.material("vision_glass", rgba=(0.15, 0.19, 0.20, 0.95))
    hatch_dark = model.material("hatch_dark", rgba=(0.16, 0.17, 0.16, 1.0))

    vehicle_body = model.part("vehicle_body")
    vehicle_body.inertial = Inertial.from_geometry(
        Box((3.30, 6.60, 2.10)),
        mass=26500.0,
        origin=Origin(xyz=(0.0, -0.10, 1.05)),
    )

    vehicle_body.visual(
        Box((0.46, 5.35, 0.92)),
        origin=Origin(xyz=(1.23, -0.05, 0.46)),
        material=track_rubber,
        name="left_track_band",
    )
    vehicle_body.visual(
        Box((0.46, 5.35, 0.92)),
        origin=Origin(xyz=(-1.23, -0.05, 0.46)),
        material=track_rubber,
        name="right_track_band",
    )
    vehicle_body.visual(
        Box((2.08, 5.30, 0.62)),
        origin=Origin(xyz=(0.0, -0.05, 0.77)),
        material=body_green,
        name="lower_hull",
    )
    vehicle_body.visual(
        Box((0.16, 4.60, 0.42)),
        origin=Origin(xyz=(1.03, -0.20, 1.05)),
        material=shadow_green,
    )
    vehicle_body.visual(
        Box((0.16, 4.60, 0.42)),
        origin=Origin(xyz=(-1.03, -0.20, 1.05)),
        material=shadow_green,
    )
    vehicle_body.visual(
        Box((2.30, 3.20, 0.46)),
        origin=Origin(xyz=(0.0, 0.25, 1.31)),
        material=body_green,
        name="upper_hull",
    )
    vehicle_body.visual(
        Box((2.34, 1.55, 0.54)),
        origin=Origin(xyz=(0.0, 1.58, 1.73)),
        material=body_green,
        name="cab_block",
    )
    vehicle_body.visual(
        Box((2.20, 0.92, 0.38)),
        origin=Origin(xyz=(0.0, 2.03, 1.31), rpy=(-0.48, 0.0, 0.0)),
        material=body_green,
        name="front_glacis",
    )
    vehicle_body.visual(
        Box((2.18, 0.92, 0.32)),
        origin=Origin(xyz=(0.0, 2.34, 0.98)),
        material=body_green,
        name="nose_lower",
    )
    vehicle_body.visual(
        Box((2.20, 2.18, 0.16)),
        origin=Origin(xyz=(0.0, -1.50, 1.46)),
        material=body_green,
        name="rear_deck",
    )
    vehicle_body.visual(
        Box((0.58, 0.95, 0.24)),
        origin=Origin(xyz=(0.76, -0.72, 1.55)),
        material=shadow_green,
    )
    vehicle_body.visual(
        Box((0.58, 0.95, 0.24)),
        origin=Origin(xyz=(-0.76, -0.72, 1.55)),
        material=shadow_green,
    )
    vehicle_body.visual(
        Box((0.18, 0.92, 0.34)),
        origin=Origin(xyz=(1.26, -1.52, 1.71)),
        material=shadow_green,
        name="left_cradle_cheek",
    )
    vehicle_body.visual(
        Box((0.18, 0.92, 0.34)),
        origin=Origin(xyz=(-1.26, -1.52, 1.71)),
        material=shadow_green,
        name="right_cradle_cheek",
    )
    vehicle_body.visual(
        Box((2.52, 0.28, 0.14)),
        origin=Origin(xyz=(0.0, -1.86, 1.61)),
        material=dark_steel,
        name="cradle_crossbeam",
    )
    vehicle_body.visual(
        Box((0.28, 0.78, 0.12)),
        origin=Origin(xyz=(0.0, -2.00, 1.52)),
        material=dark_steel,
    )

    for x in (0.72, 0.0, -0.72):
        vehicle_body.visual(
            Box((0.34, 0.05, 0.14)),
            origin=Origin(xyz=(x, 2.18, 1.63), rpy=(-0.18, 0.0, 0.0)),
            material=glass,
        )

    for side_x in (1.23, -1.23):
        for y in (-1.75, -1.00, -0.25, 0.50, 1.25):
            vehicle_body.visual(
                Cylinder(radius=0.23, length=0.18),
                origin=Origin(xyz=(side_x, y, 0.36), rpy=(0.0, pi / 2.0, 0.0)),
                material=dark_steel,
            )
        for y in (-2.10, 1.95):
            vehicle_body.visual(
                Cylinder(radius=0.32, length=0.20),
                origin=Origin(xyz=(side_x, y, 0.48), rpy=(0.0, pi / 2.0, 0.0)),
                material=dark_steel,
            )
        for y in (-0.95, 0.15, 1.20):
            vehicle_body.visual(
                Cylinder(radius=0.09, length=0.12),
                origin=Origin(xyz=(side_x, y, 0.82), rpy=(0.0, pi / 2.0, 0.0)),
                material=dark_steel,
            )

    launcher_pod = model.part("launcher_pod")
    launcher_pod.inertial = Inertial.from_geometry(
        Box((2.34, 1.90, 0.84)),
        mass=3200.0,
        origin=Origin(xyz=(0.0, 0.78, 0.30)),
    )
    launcher_pod.visual(
        Cylinder(radius=0.11, length=2.34),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="pod_trunnion",
    )
    launcher_pod.visual(
        Box((2.28, 1.82, 0.66)),
        origin=Origin(xyz=(0.0, 0.78, 0.27)),
        material=body_green,
        name="launcher_shell",
    )
    launcher_pod.visual(
        Box((1.64, 0.38, 0.16)),
        origin=Origin(xyz=(0.0, 0.16, 0.06)),
        material=shadow_green,
        name="pod_saddle",
    )
    launcher_pod.visual(
        Box((0.20, 0.34, 0.18)),
        origin=Origin(xyz=(0.88, 0.10, 0.07)),
        material=shadow_green,
    )
    launcher_pod.visual(
        Box((0.20, 0.34, 0.18)),
        origin=Origin(xyz=(-0.88, 0.10, 0.07)),
        material=shadow_green,
    )
    launcher_pod.visual(
        Box((1.92, 1.40, 0.06)),
        origin=Origin(xyz=(0.0, 0.82, 0.60)),
        material=shadow_green,
    )
    launcher_pod.visual(
        Box((0.048, 0.70, 0.60)),
        origin=Origin(xyz=(1.092, 0.85, 0.33)),
        material=shadow_green,
    )
    launcher_pod.visual(
        Box((0.012, 0.66, 0.50)),
        origin=Origin(xyz=(1.134, 0.85, 0.33)),
        material=hatch_dark,
        name="hatch_recess",
    )
    launcher_pod.visual(
        Box((0.016, 0.12, 0.46)),
        origin=Origin(xyz=(1.136, 1.20, 0.33)),
        material=dark_steel,
        name="hinge_mount_strip",
    )
    launcher_pod.visual(
        Cylinder(radius=0.012, length=0.14),
        origin=Origin(xyz=(1.156, 1.20, 0.13)),
        material=dark_steel,
        name="pod_lower_hinge_knuckle",
    )
    launcher_pod.visual(
        Cylinder(radius=0.012, length=0.14),
        origin=Origin(xyz=(1.156, 1.20, 0.53)),
        material=dark_steel,
        name="pod_upper_hinge_knuckle",
    )

    access_hatch = model.part("access_hatch")
    access_hatch.inertial = Inertial.from_geometry(
        Box((0.05, 0.74, 0.58)),
        mass=45.0,
        origin=Origin(xyz=(0.035, -0.35, 0.0)),
    )
    access_hatch.visual(
        Box((0.028, 0.70, 0.56)),
        origin=Origin(xyz=(0.024, -0.35, 0.0)),
        material=body_green,
        name="hatch_panel",
    )
    access_hatch.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(),
        material=dark_steel,
        name="hatch_hinge_barrel",
    )
    access_hatch.visual(
        Box((0.02, 0.14, 0.05)),
        origin=Origin(xyz=(0.018, -0.48, 0.0)),
        material=dark_steel,
        name="hatch_handle",
    )

    model.articulation(
        "body_to_launcher_elevation",
        ArticulationType.REVOLUTE,
        parent=vehicle_body,
        child=launcher_pod,
        origin=Origin(xyz=(0.0, -1.52, 1.74)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14000.0,
            velocity=0.40,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "launcher_to_access_hatch",
        ArticulationType.REVOLUTE,
        parent=launcher_pod,
        child=access_hatch,
        origin=Origin(xyz=(1.156, 1.20, 0.33)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.20,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    vehicle_body = object_model.get_part("vehicle_body")
    launcher_pod = object_model.get_part("launcher_pod")
    access_hatch = object_model.get_part("access_hatch")
    elevate = object_model.get_articulation("body_to_launcher_elevation")
    hatch_joint = object_model.get_articulation("launcher_to_access_hatch")

    ctx.expect_contact(
        launcher_pod,
        vehicle_body,
        elem_a="pod_trunnion",
        elem_b="left_cradle_cheek",
        name="left trunnion seats against left cradle cheek",
    )
    ctx.expect_contact(
        launcher_pod,
        vehicle_body,
        elem_a="pod_trunnion",
        elem_b="right_cradle_cheek",
        name="right trunnion seats against right cradle cheek",
    )

    with ctx.pose({elevate: 0.0, hatch_joint: 0.0}):
        ctx.expect_gap(
            launcher_pod,
            vehicle_body,
            axis="z",
            positive_elem="launcher_shell",
            negative_elem="rear_deck",
            min_gap=0.10,
            max_gap=0.20,
            name="stowed launcher shell clears the rear deck",
        )
        ctx.expect_gap(
            access_hatch,
            launcher_pod,
            axis="x",
            positive_elem="hatch_panel",
            negative_elem="launcher_shell",
            min_gap=0.015,
            max_gap=0.035,
            name="closed access hatch sits slightly proud of pod wall",
        )

    closed_pod_aabb = ctx.part_element_world_aabb(launcher_pod, elem="launcher_shell")
    with ctx.pose({elevate: elevate.motion_limits.upper}):
        raised_pod_aabb = ctx.part_element_world_aabb(launcher_pod, elem="launcher_shell")
    ctx.check(
        "launcher pod elevates upward",
        closed_pod_aabb is not None
        and raised_pod_aabb is not None
        and raised_pod_aabb[1][2] > closed_pod_aabb[1][2] + 1.1,
        details=f"closed={closed_pod_aabb}, raised={raised_pod_aabb}",
    )

    closed_hatch_aabb = ctx.part_element_world_aabb(access_hatch, elem="hatch_panel")
    with ctx.pose({hatch_joint: hatch_joint.motion_limits.upper}):
        open_hatch_aabb = ctx.part_element_world_aabb(access_hatch, elem="hatch_panel")
    ctx.check(
        "access hatch swings outward from the pod wall",
        closed_hatch_aabb is not None
        and open_hatch_aabb is not None
        and open_hatch_aabb[1][0] > closed_hatch_aabb[1][0] + 0.20,
        details=f"closed={closed_hatch_aabb}, open={open_hatch_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
