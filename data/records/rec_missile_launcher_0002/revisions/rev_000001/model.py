from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_missile_launcher", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.34, 0.39, 0.35, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.20, 0.20, 1.0))
    olive_drab = model.material("olive_drab", rgba=(0.30, 0.36, 0.24, 1.0))
    access_panel = model.material("access_panel", rgba=(0.42, 0.47, 0.39, 1.0))
    hazard_gray = model.material("hazard_gray", rgba=(0.46, 0.49, 0.47, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.72, 0.60, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=painted_steel,
        name="foundation_plinth",
    )
    base.visual(
        Cylinder(radius=0.10, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=dark_steel,
        name="pedestal_column",
    )
    base.visual(
        Box((0.34, 0.28, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=painted_steel,
        name="top_plate",
    )
    base.visual(
        Box((0.18, 0.16, 0.12)),
        origin=Origin(xyz=(-0.18, 0.0, 0.14)),
        material=dark_steel,
        name="service_box",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.72, 0.60, 0.36)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.15, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="slew_ring",
    )
    turntable.visual(
        Box((0.18, 0.20, 0.12)),
        origin=Origin(xyz=(-0.07, 0.0, 0.11)),
        material=painted_steel,
        name="drive_housing",
    )
    turntable.visual(
        Box((0.05, 0.03, 0.26)),
        origin=Origin(xyz=(0.0, 0.16, 0.18)),
        material=painted_steel,
        name="support_left",
    )
    turntable.visual(
        Box((0.05, 0.03, 0.26)),
        origin=Origin(xyz=(0.0, -0.16, 0.18)),
        material=painted_steel,
        name="support_right",
    )
    turntable.visual(
        Box((0.16, 0.22, 0.04)),
        origin=Origin(xyz=(0.03, 0.0, 0.07)),
        material=painted_steel,
        name="deck_plate",
    )
    turntable.inertial = Inertial.from_geometry(
        Box((0.32, 0.38, 0.31)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.045, length=0.015),
        origin=Origin(xyz=(0.0, 0.1375, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hazard_gray,
        name="trunnion_left",
    )
    cradle.visual(
        Cylinder(radius=0.045, length=0.015),
        origin=Origin(xyz=(0.0, -0.1375, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hazard_gray,
        name="trunnion_right",
    )
    cradle.visual(
        Box((0.40, 0.16, 0.03)),
        origin=Origin(xyz=(0.20, 0.0, 0.015)),
        material=painted_steel,
        name="tray",
    )
    cradle.visual(
        Box((0.40, 0.05, 0.08)),
        origin=Origin(xyz=(0.20, 0.105, 0.04)),
        material=painted_steel,
        name="side_left",
    )
    cradle.visual(
        Box((0.40, 0.05, 0.08)),
        origin=Origin(xyz=(0.20, -0.105, 0.04)),
        material=painted_steel,
        name="side_right",
    )
    cradle.visual(
        Box((0.06, 0.25, 0.08)),
        origin=Origin(xyz=(0.03, 0.0, 0.055)),
        material=dark_steel,
        name="rear_bulkhead",
    )
    cradle.visual(
        Box((0.24, 0.07, 0.015)),
        origin=Origin(xyz=(0.27, 0.085, 0.0875)),
        material=hazard_gray,
        name="mount_left",
    )
    cradle.visual(
        Box((0.24, 0.07, 0.015)),
        origin=Origin(xyz=(0.27, -0.085, 0.0875)),
        material=hazard_gray,
        name="mount_right",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.40, 0.29, 0.12)),
        mass=140.0,
        origin=Origin(xyz=(0.20, 0.0, 0.03)),
    )

    pod = model.part("launch_pod")
    pod.visual(
        Box((0.24, 0.04, 0.02)),
        origin=Origin(xyz=(0.09, 0.08, 0.01)),
        material=hazard_gray,
        name="skid_left",
    )
    pod.visual(
        Box((0.24, 0.04, 0.02)),
        origin=Origin(xyz=(0.09, -0.08, 0.01)),
        material=hazard_gray,
        name="skid_right",
    )
    pod.visual(
        Box((0.48, 0.30, 0.20)),
        origin=Origin(xyz=(0.12, 0.0, 0.12)),
        material=olive_drab,
        name="pod_shell",
    )
    pod.visual(
        Box((0.30, 0.24, 0.018)),
        origin=Origin(xyz=(0.13, 0.0, 0.219)),
        material=access_panel,
        name="roof_panel",
    )
    pod.visual(
        Box((0.10, 0.22, 0.10)),
        origin=Origin(xyz=(-0.17, 0.0, 0.08)),
        material=painted_steel,
        name="rear_service_housing",
    )
    door_y = (-0.073, 0.073)
    door_z = (0.058, 0.115, 0.172)
    door_index = 0
    for y in door_y:
        for z in door_z:
            pod.visual(
                Box((0.008, 0.125, 0.05)),
                origin=Origin(xyz=(0.364, y, z)),
                material=access_panel,
                name=f"cell_door_{door_index}",
            )
            door_index += 1
    pod.inertial = Inertial.from_geometry(
        Box((0.58, 0.30, 0.22)),
        mass=260.0,
        origin=Origin(xyz=(0.08, 0.0, 0.11)),
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8500.0, velocity=1.2),
    )
    model.articulation(
        "turntable_to_cradle",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6400.0,
            velocity=0.9,
            lower=math.radians(-10.0),
            upper=math.radians(60.0),
        ),
    )
    model.articulation(
        "cradle_to_pod",
        ArticulationType.FIXED,
        parent=cradle,
        child=pod,
        origin=Origin(xyz=(0.18, 0.0, 0.095)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    cradle = object_model.get_part("cradle")
    pod = object_model.get_part("launch_pod")

    yaw = object_model.get_articulation("base_to_turntable")
    pitch = object_model.get_articulation("turntable_to_cradle")

    base_top = base.get_visual("top_plate")
    slew_ring = turntable.get_visual("slew_ring")
    support_left = turntable.get_visual("support_left")
    support_right = turntable.get_visual("support_right")
    trunnion_left = cradle.get_visual("trunnion_left")
    trunnion_right = cradle.get_visual("trunnion_right")
    mount_left = cradle.get_visual("mount_left")
    mount_right = cradle.get_visual("mount_right")
    skid_left = pod.get_visual("skid_left")
    skid_right = pod.get_visual("skid_right")
    front_door = pod.get_visual("cell_door_4")
    rear_service = pod.get_visual("rear_service_housing")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=32)

    ctx.expect_contact(turntable, base, elem_a=slew_ring, elem_b=base_top)
    ctx.expect_overlap(turntable, base, axes="xy", elem_a=slew_ring, elem_b=base_top, min_overlap=0.28)

    ctx.expect_contact(cradle, turntable, elem_a=trunnion_left, elem_b=support_left)
    ctx.expect_contact(cradle, turntable, elem_a=trunnion_right, elem_b=support_right)
    ctx.expect_overlap(cradle, turntable, axes="xz", elem_a=trunnion_left, elem_b=support_left, min_overlap=0.05)
    ctx.expect_overlap(cradle, turntable, axes="xz", elem_a=trunnion_right, elem_b=support_right, min_overlap=0.05)

    ctx.expect_contact(pod, cradle, elem_a=skid_left, elem_b=mount_left)
    ctx.expect_contact(pod, cradle, elem_a=skid_right, elem_b=mount_right)
    ctx.expect_overlap(pod, cradle, axes="xy", elem_a=skid_left, elem_b=mount_left, min_overlap=0.03)
    ctx.expect_overlap(pod, cradle, axes="xy", elem_a=skid_right, elem_b=mount_right, min_overlap=0.03)
    ctx.expect_overlap(pod, cradle, axes="x", min_overlap=0.20)

    ctx.expect_origin_distance(turntable, base, axes="xy", max_dist=1e-6)
    ctx.expect_origin_distance(cradle, turntable, axes="xy", max_dist=1e-6)

    def _center_of_elem(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        assert aabb is not None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    rest_front = _center_of_elem(pod, front_door)
    rest_rear = _center_of_elem(pod, rear_service)
    rest_radius = math.hypot(rest_front[0], rest_front[1])

    with ctx.pose({yaw: math.pi / 2.0}):
        yaw_front = _center_of_elem(pod, front_door)
        yaw_radius = math.hypot(yaw_front[0], yaw_front[1])
        ctx.check(
            "yaw rotates pod about vertical centerline",
            abs(yaw_front[2] - rest_front[2]) < 1e-4
            and abs(yaw_radius - rest_radius) < 1e-4
            and abs(yaw_front[0]) < rest_radius * 0.25
            and abs(yaw_front[1]) > rest_radius * 0.75,
            details=(
                f"rest_front={rest_front}, yaw_front={yaw_front}, "
                f"rest_radius={rest_radius:.5f}, yaw_radius={yaw_radius:.5f}"
            ),
        )
        ctx.expect_contact(cradle, turntable, elem_a=trunnion_left, elem_b=support_left)
        ctx.expect_contact(pod, cradle, elem_a=skid_left, elem_b=mount_left)

    with ctx.pose({pitch: math.radians(45.0)}):
        pitched_front = _center_of_elem(pod, front_door)
        pitched_rear = _center_of_elem(pod, rear_service)
        ctx.check(
            "pitch lifts front of launch pod",
            pitched_front[2] > rest_front[2] + 0.14
            and pitched_front[2] - pitched_rear[2] > rest_front[2] - rest_rear[2] + 0.16,
            details=(
                f"rest_front={rest_front}, pitched_front={pitched_front}, "
                f"rest_rear={rest_rear}, pitched_rear={pitched_rear}"
            ),
        )
        ctx.expect_contact(cradle, turntable, elem_a=trunnion_right, elem_b=support_right)
        ctx.expect_contact(pod, cradle, elem_a=skid_right, elem_b=mount_right)

    with ctx.pose({pitch: math.radians(-10.0)}):
        ctx.expect_contact(cradle, turntable, elem_a=trunnion_left, elem_b=support_left)
        ctx.expect_contact(cradle, turntable, elem_a=trunnion_right, elem_b=support_right)
        ctx.expect_contact(pod, cradle, elem_a=skid_left, elem_b=mount_left)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
